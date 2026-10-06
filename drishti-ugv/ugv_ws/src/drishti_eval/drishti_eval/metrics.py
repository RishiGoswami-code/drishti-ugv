# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
"""Localisation error metrics: ATE, RPE and drift.

EVALUATION.md section 2 makes these the localisation score, and section 7.1
says a run without its ground-truth track is not evaluable. This module is the
implementation of those definitions and nothing else -- no ROS, no I/O, no
plotting, so it is testable anywhere (STATUS.md D17).

Two decisions that materially change the numbers, made explicit rather than
buried:

1. **Alignment.** ATE is meaningless until the estimate is brought into the
   ground-truth frame, because SLAM fixes its origin at whatever pose it
   started from. We align with Umeyama.

2. **Scale.** With stereo or RGB-D the scale is metric and observable, so
   scale is NOT estimated -- fitting it would hide real error. `with_scale` is
   available only for evaluating a monocular ablation, where scale genuinely
   is unobservable. SPEC.md 5.1 keeps monocular as a fallback, so the option
   exists, but the default is rigid alignment and the report always states
   which was used.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .trajectory import Trajectory, associate, rotation_angle, se3_inverse


@dataclass
class Alignment:
    """Similarity transform taking the estimate into the reference frame."""

    rotation: np.ndarray                       # (3, 3)
    translation: np.ndarray                    # (3,)
    scale: float = 1.0

    def apply(self, points: np.ndarray) -> np.ndarray:
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        return (self.scale * (self.rotation @ points.T)).T + self.translation

    def matrix(self) -> np.ndarray:
        T = np.eye(4)
        T[:3, :3] = self.scale * self.rotation
        T[:3, 3] = self.translation
        return T


def umeyama(source: np.ndarray, target: np.ndarray,
            with_scale: bool = False) -> Alignment:
    """Least-squares similarity transform mapping `source` onto `target`.

    Umeyama (1991), "Least-squares estimation of transformation parameters
    between two point patterns". Returns R, t, c minimising
    sum ||target_i - (c R source_i + t)||^2.

    The reflection guard matters: without it a degenerate or noisy
    configuration can yield det(R) = -1, a mirror rather than a rotation, and
    the resulting "error" is nonsense that looks plausible.
    """
    source = np.asarray(source, dtype=float).reshape(-1, 3)
    target = np.asarray(target, dtype=float).reshape(-1, 3)
    if len(source) != len(target):
        raise ValueError("umeyama: %d source points vs %d target points"
                         % (len(source), len(target)))
    n = len(source)
    if n < 3:
        raise ValueError("umeyama needs at least 3 correspondences, got %d" % n)

    mu_s = source.mean(axis=0)
    mu_t = target.mean(axis=0)
    src_c = source - mu_s
    tgt_c = target - mu_t

    var_s = float(np.mean(np.sum(src_c ** 2, axis=1)))
    cov = (tgt_c.T @ src_c) / n

    U, D, Vt = np.linalg.svd(cov)

    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0.0:
        S[2, 2] = -1.0

    R = U @ S @ Vt

    if with_scale:
        if var_s < 1e-12:
            raise ValueError("umeyama: source points are coincident; scale is "
                             "unobservable")
        c = float(np.trace(np.diag(D) @ S) / var_s)
    else:
        c = 1.0

    t = mu_t - c * (R @ mu_s)
    return Alignment(rotation=R, translation=t, scale=c)


@dataclass
class ErrorStats:
    """Summary of an error series. RMSE is the headline; the rest resist
    a single outlier being mistaken for a trend, or hidden by one."""

    rmse: float
    mean: float
    median: float
    std: float
    minimum: float
    maximum: float
    count: int

    @staticmethod
    def of(errors: np.ndarray) -> "ErrorStats":
        e = np.asarray(errors, dtype=float).reshape(-1)
        if e.size == 0:
            z = float("nan")
            return ErrorStats(z, z, z, z, z, z, 0)
        return ErrorStats(
            rmse=float(np.sqrt(np.mean(e ** 2))),
            mean=float(np.mean(e)),
            median=float(np.median(e)),
            std=float(np.std(e)),
            minimum=float(np.min(e)),
            maximum=float(np.max(e)),
            count=int(e.size),
        )

    def as_dict(self) -> dict:
        return {
            "rmse": self.rmse, "mean": self.mean, "median": self.median,
            "std": self.std, "min": self.minimum, "max": self.maximum,
            "count": self.count,
        }


@dataclass
class AteResult:
    translation: ErrorStats
    alignment: Alignment
    with_scale: bool
    errors: np.ndarray = field(repr=False)
    path_length: float = 0.0

    @property
    def drift_percent(self) -> float:
        """ATE RMSE as a percentage of distance travelled (EVALUATION.md 2.2).

        Normalising by path length is what makes a 40 m and a 400 m run
        comparable. NaN when the vehicle did not move: a drift ratio over zero
        distance is undefined, and returning 0.0 would read as a perfect score.
        """
        if self.path_length <= 1e-9:
            return float("nan")
        return 100.0 * self.translation.rmse / self.path_length


@dataclass
class RpeResult:
    translation: ErrorStats
    rotation_deg: ErrorStats
    delta: float
    delta_unit: str
    pairs: int


def absolute_trajectory_error(estimate: Trajectory, reference: Trajectory,
                              with_scale: bool = False,
                              max_difference: float = 0.02) -> AteResult:
    """ATE: RMSE of position error after aligning estimate to reference.

    `reference` is simulator ground truth (EVALUATION.md 7.1). Alignment is
    rigid by default; see the module docstring for why scale is not fitted.
    """
    est, ref = associate(estimate, reference, max_difference)
    if len(est) < 3:
        raise ValueError(
            "ATE needs at least 3 associated poses, found %d. Check that the "
            "estimate and ground truth share a clock and that max_difference "
            "(%.3f s) is not tighter than the sample interval." % (len(est), max_difference))

    align = umeyama(est.positions, ref.positions, with_scale=with_scale)
    aligned = align.apply(est.positions)
    errors = np.linalg.norm(aligned - ref.positions, axis=1)

    return AteResult(
        translation=ErrorStats.of(errors),
        alignment=align,
        with_scale=with_scale,
        errors=errors,
        # Path length comes from the REFERENCE: the estimate's own path length
        # is itself corrupted by the drift being measured.
        path_length=ref.path_length(),
    )


def relative_pose_error(estimate: Trajectory, reference: Trajectory,
                        delta: float = 1.0, delta_unit: str = "m",
                        max_difference: float = 0.02) -> RpeResult:
    """RPE over a fixed window: local consistency, independent of global drift.

    ATE and RPE answer different questions. A trajectory can have a large ATE
    from one early yaw error and still be locally excellent; RPE is what says
    whether odometry is sound right now. EVALUATION.md 2 wants both.

    `delta_unit`:
      "m"      window of `delta` metres along the reference path (default;
               comparable across runs at different speeds)
      "s"      window of `delta` seconds
      "frames" window of `delta` associated samples
    """
    est, ref = associate(estimate, reference, max_difference)
    n = len(est)
    if n < 2:
        raise ValueError("RPE needs at least 2 associated poses, found %d" % n)

    pairs = _rpe_pairs(ref, delta, delta_unit, n)
    if not pairs:
        raise ValueError(
            "RPE found no pose pairs separated by %g %s. The run is probably "
            "shorter than the window." % (delta, delta_unit))

    est_poses = est.poses()
    ref_poses = ref.poses()

    trans_err, rot_err = [], []
    for i, j in pairs:
        # Relative motion in each track, then the error between them.
        d_ref = se3_inverse(ref_poses[i]) @ ref_poses[j]
        d_est = se3_inverse(est_poses[i]) @ est_poses[j]
        err = se3_inverse(d_ref) @ d_est
        trans_err.append(float(np.linalg.norm(err[:3, 3])))
        rot_err.append(np.degrees(rotation_angle(err[:3, :3])))

    return RpeResult(
        translation=ErrorStats.of(np.array(trans_err)),
        rotation_deg=ErrorStats.of(np.array(rot_err)),
        delta=delta,
        delta_unit=delta_unit,
        pairs=len(pairs),
    )


def _rpe_pairs(ref: Trajectory, delta: float, unit: str, n: int):
    """Index pairs (i, j) separated by `delta` in the requested unit."""
    if unit == "frames":
        step = int(round(delta))
        if step < 1:
            raise ValueError("RPE frame delta must be >= 1, got %g" % delta)
        return [(i, i + step) for i in range(n - step)]

    if unit == "s":
        key = ref.stamps
    elif unit == "m":
        seg = np.linalg.norm(np.diff(ref.positions, axis=0), axis=1)
        key = np.concatenate([[0.0], np.cumsum(seg)])
    else:
        raise ValueError("unknown delta_unit %r; use 'm', 's' or 'frames'" % unit)

    pairs = []
    j = 0
    for i in range(n):
        if j < i + 1:
            j = i + 1
        while j < n and (key[j] - key[i]) < delta:
            j += 1
        if j < n:
            pairs.append((i, j))
    return pairs
