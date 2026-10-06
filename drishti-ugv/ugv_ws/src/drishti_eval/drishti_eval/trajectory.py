# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
"""Timestamped pose tracks and the association between two of them.

No ROS import anywhere in this module. The bag reader converts messages into
these containers and everything downstream is plain numpy, so the metrics can
be developed and tested on a machine with no ROS installed (STATUS.md D17).

Conventions, fixed once here so they are not re-decided per call site:

* translations are metres, shape (N, 3)
* rotations are unit quaternions in **(x, y, z, w)** order, shape (N, 4) --
  the ROS convention, so no reordering happens at the boundary
* timestamps are float seconds on the clock the bag was recorded with
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def quat_to_matrix(q: np.ndarray) -> np.ndarray:
    """(x, y, z, w) unit quaternion -> 3x3 rotation matrix."""
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ])


def to_se3(t: np.ndarray, q: np.ndarray) -> np.ndarray:
    """Translation + quaternion -> 4x4 homogeneous transform."""
    T = np.eye(4)
    T[:3, :3] = quat_to_matrix(q)
    T[:3, 3] = t
    return T


def se3_inverse(T: np.ndarray) -> np.ndarray:
    """Exact inverse of a rigid transform; never call np.linalg.inv on these."""
    R = T[:3, :3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ T[:3, 3]
    return out


def rotation_angle(R: np.ndarray) -> float:
    """Geodesic angle of a rotation matrix, radians, in [0, pi].

    Uses atan2(sin, cos) rather than arccos(cos). The arccos form loses
    catastrophic precision near zero: its derivative is unbounded there, so a
    trace perturbed by 1e-13 of float noise reports an angle near 5e-7 rad
    instead of 0. That is three orders of magnitude of fictitious rotation
    error on a perfect trajectory, which is exactly the regime a well-behaved
    RPE run sits in.

    sin comes from the skew-symmetric part, cos from the trace; atan2 is
    well-conditioned at both ends of [0, pi].
    """
    cos = (np.trace(R) - 1.0) / 2.0
    axis = np.array([R[2, 1] - R[1, 2],
                     R[0, 2] - R[2, 0],
                     R[1, 0] - R[0, 1]])
    sin = np.linalg.norm(axis) / 2.0
    return float(np.arctan2(sin, np.clip(cos, -1.0, 1.0)))


@dataclass
class Trajectory:
    """A time-ordered pose track."""

    stamps: np.ndarray          # (N,) float seconds
    positions: np.ndarray       # (N, 3) metres
    quaternions: np.ndarray     # (N, 4) xyzw

    def __post_init__(self) -> None:
        self.stamps = np.asarray(self.stamps, dtype=float).reshape(-1)
        self.positions = np.asarray(self.positions, dtype=float).reshape(-1, 3)
        self.quaternions = np.asarray(self.quaternions, dtype=float).reshape(-1, 4)
        n = len(self.stamps)
        if len(self.positions) != n or len(self.quaternions) != n:
            raise ValueError(
                "Trajectory arrays disagree: %d stamps, %d positions, %d quaternions"
                % (n, len(self.positions), len(self.quaternions)))
        if n and not np.all(np.diff(self.stamps) >= 0):
            order = np.argsort(self.stamps, kind="stable")
            self.stamps = self.stamps[order]
            self.positions = self.positions[order]
            self.quaternions = self.quaternions[order]

    def __len__(self) -> int:
        return len(self.stamps)

    def poses(self) -> np.ndarray:
        """(N, 4, 4) homogeneous transforms."""
        return np.array([to_se3(p, q)
                         for p, q in zip(self.positions, self.quaternions)])

    def path_length(self) -> float:
        """Integrated path length in metres.

        EVALUATION.md 2.1 pins distance travelled to the integrated
        ground-truth path, never commanded velocity integrated over time --
        the two diverge badly whenever a wheel slips or the supervisor
        intervenes.
        """
        if len(self) < 2:
            return 0.0
        return float(np.sum(np.linalg.norm(np.diff(self.positions, axis=0), axis=1)))

    def duration(self) -> float:
        return float(self.stamps[-1] - self.stamps[0]) if len(self) > 1 else 0.0

    def subset(self, idx) -> "Trajectory":
        idx = np.asarray(idx, dtype=int)
        return Trajectory(self.stamps[idx], self.positions[idx], self.quaternions[idx])


def associate(estimate: Trajectory, reference: Trajectory,
              max_difference: float = 0.02):
    """Match estimate samples to their nearest reference sample in time.

    Returns (estimate_subset, reference_subset) of equal length, containing
    only pairs closer together than `max_difference` seconds.

    Each reference sample is used at most once. Estimates usually run slower
    than ground truth, so without that rule a stalled estimator would match the
    same reference pose repeatedly and quietly flatter its own error.
    """
    if len(estimate) == 0 or len(reference) == 0:
        empty = np.zeros((0,), dtype=int)
        return estimate.subset(empty), reference.subset(empty)

    ref_stamps = reference.stamps
    est_idx, ref_idx = [], []
    used = set()

    for i, t in enumerate(estimate.stamps):
        j = int(np.searchsorted(ref_stamps, t))
        best, best_dt = -1, np.inf
        for cand in (j - 1, j, j + 1):
            if 0 <= cand < len(ref_stamps) and cand not in used:
                dt = abs(ref_stamps[cand] - t)
                if dt < best_dt:
                    best, best_dt = cand, dt
        if best >= 0 and best_dt <= max_difference:
            est_idx.append(i)
            ref_idx.append(best)
            used.add(best)

    return estimate.subset(est_idx), reference.subset(ref_idx)
