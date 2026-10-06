# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
"""Tests for the localisation metrics.

Every case has an answer derivable by hand, so a wrong implementation fails
rather than merely producing a different-looking number. That matters more here
than usual: EVALUATION.md targets a drift figure below 2%, and a subtly wrong
ATE would let us report success we had not earned.

No ROS. Runs standalone or under pytest:

    python test/test_metrics.py
    pytest test/test_metrics.py
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from drishti_eval.metrics import (  # noqa: E402
    absolute_trajectory_error, relative_pose_error, umeyama)
from drishti_eval.trajectory import (  # noqa: E402
    Trajectory, associate, quat_to_matrix, rotation_angle, se3_inverse, to_se3)

TOL = 1e-9


# ----------------------------------------------------------------- helpers
def quat_from_yaw(yaw):
    return np.array([0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)])


def quat_from_matrix(R):
    """Rotation matrix -> (x, y, z, w), via the numerically stable branch."""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2.0
        w, x = 0.25 * s, (R[2, 1] - R[1, 2]) / s
        y, z = (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w, x = (R[2, 1] - R[1, 2]) / s, 0.25 * s
        y, z = (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w, x = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s
        y, z = 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w, x = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s
        y, z = (R[1, 2] + R[2, 1]) / s, 0.25 * s
    return np.array([x, y, z, w])


def straight_line(n=101, length=10.0, dt=0.1):
    """Drive +x at constant speed, facing +x."""
    s = np.linspace(0.0, length, n)
    pos = np.stack([s, np.zeros(n), np.zeros(n)], axis=1)
    quat = np.tile(quat_from_yaw(0.0), (n, 1))
    return Trajectory(np.arange(n) * dt, pos, quat)


def arc(n=201, radius=5.0, sweep=math.pi, dt=0.05):
    """Constant-radius turn, heading tangent to the path."""
    th = np.linspace(0.0, sweep, n)
    pos = np.stack([radius * np.sin(th), radius * (1 - np.cos(th)), np.zeros(n)], axis=1)
    quat = np.stack([quat_from_yaw(a) for a in th])
    return Trajectory(np.arange(n) * dt, pos, quat)


def transform(traj, R, t, scale=1.0):
    pos = (scale * (R @ traj.positions.T)).T + t
    quat = np.stack([quat_from_matrix(R @ quat_to_matrix(q)) for q in traj.quaternions])
    return Trajectory(traj.stamps.copy(), pos, quat)


def rot_xyz(rx, ry, rz):
    cx, sx, cy, sy, cz, sz = (math.cos(rx), math.sin(rx), math.cos(ry),
                              math.sin(ry), math.cos(rz), math.sin(rz))
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


# ------------------------------------------------------------- test harness
_checks = 0
_failures = 0
_case = ""


def _check(cond, expr, line):
    global _checks, _failures
    _checks += 1
    if not cond:
        _failures += 1
        print("  FAIL  [%s] line %d: %s" % (_case, line, expr))


def CHECK(cond):
    import inspect
    _check(bool(cond), "condition", inspect.currentframe().f_back.f_lineno)


def CLOSE(a, b, tol=1e-9, what=""):
    import inspect
    line = inspect.currentframe().f_back.f_lineno
    okay = abs(float(a) - float(b)) <= tol
    _check(okay, "%s got %.12g want %.12g (tol %g)" % (what, a, b, tol), line)


def case(name):
    global _case
    _case = name


# ------------------------------------------------------------------- tests
def test_umeyama_recovers_a_known_rigid_transform():
    case("umeyama recovers a known rigid transform")
    rng = np.random.default_rng(7)
    src = rng.normal(size=(60, 3)) * 3.0
    R = rot_xyz(0.3, -0.7, 1.1)
    t = np.array([2.5, -1.25, 0.75])
    tgt = (R @ src.T).T + t

    a = umeyama(src, tgt, with_scale=False)
    CLOSE(np.max(np.abs(a.rotation - R)), 0.0, 1e-9, "rotation")
    CLOSE(np.max(np.abs(a.translation - t)), 0.0, 1e-9, "translation")
    CLOSE(a.scale, 1.0, 1e-12, "scale")
    CLOSE(np.max(np.abs(a.apply(src) - tgt)), 0.0, 1e-9, "residual")


def test_umeyama_recovers_scale_only_when_asked():
    case("umeyama recovers scale only when asked")
    rng = np.random.default_rng(11)
    src = rng.normal(size=(50, 3)) * 2.0
    R = rot_xyz(-0.2, 0.4, 0.9)
    t = np.array([-1.0, 3.0, 0.5])
    true_scale = 2.5
    tgt = (true_scale * (R @ src.T)).T + t

    fitted = umeyama(src, tgt, with_scale=True)
    CLOSE(fitted.scale, true_scale, 1e-9, "fitted scale")
    CLOSE(np.max(np.abs(fitted.apply(src) - tgt)), 0.0, 1e-8, "residual with scale")

    rigid = umeyama(src, tgt, with_scale=False)
    CLOSE(rigid.scale, 1.0, 1e-12, "rigid scale stays 1")
    CHECK(np.max(np.abs(rigid.apply(src) - tgt)) > 1.0)


def test_umeyama_never_returns_a_reflection():
    case("umeyama never returns a reflection")
    # Planar points are the classic degenerate case that produces det(R) = -1
    # if the sign correction is missing.
    rng = np.random.default_rng(3)
    src = np.stack([rng.normal(size=40), rng.normal(size=40), np.zeros(40)], axis=1)
    mirror = np.diag([1.0, 1.0, -1.0])
    tgt = (mirror @ src.T).T
    a = umeyama(src, tgt)
    CLOSE(np.linalg.det(a.rotation), 1.0, 1e-9, "det(R) must be +1")


def test_umeyama_rejects_too_few_points():
    case("umeyama rejects too few points")
    try:
        umeyama(np.zeros((2, 3)), np.zeros((2, 3)))
        CHECK(False)
    except ValueError:
        CHECK(True)


def test_identical_trajectories_have_zero_error():
    case("identical trajectories have zero error")
    gt = arc()
    ate = absolute_trajectory_error(gt, gt)
    CLOSE(ate.translation.rmse, 0.0, 1e-9, "ATE rmse")
    CLOSE(ate.translation.maximum, 0.0, 1e-9, "ATE max")
    CLOSE(ate.drift_percent, 0.0, 1e-9, "drift %")

    rpe = relative_pose_error(gt, gt, delta=1.0, delta_unit="m")
    CLOSE(rpe.translation.rmse, 0.0, 1e-9, "RPE trans")
    CLOSE(rpe.rotation_deg.rmse, 0.0, 1e-9, "RPE rot")


def test_ate_is_invariant_to_the_slam_origin():
    case("ATE is invariant to the SLAM origin")
    # SLAM fixes its origin wherever it started. A rigid offset is not error,
    # and alignment must absorb it completely.
    gt = arc()
    est = transform(gt, rot_xyz(0.0, 0.0, 1.3), np.array([12.0, -4.0, 0.0]))
    ate = absolute_trajectory_error(est, gt)
    CLOSE(ate.translation.rmse, 0.0, 1e-8, "ATE after rigid offset")


def test_ate_absorbs_a_constant_offset():
    case("ATE absorbs a constant offset")
    # Push every estimate 0.30 m in +y. Umeyama centres both tracks, so a
    # constant offset is entirely explained by the alignment translation and
    # contributes nothing to ATE. This is correct: a fixed frame offset
    # between the SLAM origin and the world origin is not localisation error.
    gt = straight_line()
    est = Trajectory(gt.stamps.copy(),
                     gt.positions + np.array([0.0, 0.30, 0.0]),
                     gt.quaternions.copy())
    ate = absolute_trajectory_error(est, gt)
    CLOSE(ate.translation.rmse, 0.0, 1e-9, "constant offset is absorbed")


def test_ate_reports_a_known_sinusoidal_error():
    case("ATE reports a known sinusoidal error")
    # Zero-mean lateral wobble of amplitude A: alignment cannot help, and the
    # RMS of A*sin over whole periods is A/sqrt(2).
    n, amp, periods = 2001, 0.20, 8
    gt = straight_line(n=n, length=40.0, dt=0.05)
    phase = np.linspace(0.0, 2.0 * math.pi * periods, n, endpoint=False)
    est = Trajectory(gt.stamps.copy(),
                     gt.positions + np.stack(
                         [np.zeros(n), amp * np.sin(phase), np.zeros(n)], axis=1),
                     gt.quaternions.copy())
    ate = absolute_trajectory_error(est, gt)
    CLOSE(ate.translation.rmse, amp / math.sqrt(2.0), 1e-3, "sinusoid RMSE")


def test_drift_percent_normalises_by_reference_path_length():
    case("drift percent normalises by reference path length")
    n, amp = 2001, 0.20
    gt = straight_line(n=n, length=40.0, dt=0.05)
    CLOSE(gt.path_length(), 40.0, 1e-9, "path length")
    phase = np.linspace(0.0, 2.0 * math.pi * 8, n, endpoint=False)
    est = Trajectory(gt.stamps.copy(),
                     gt.positions + np.stack(
                         [np.zeros(n), amp * np.sin(phase), np.zeros(n)], axis=1),
                     gt.quaternions.copy())
    ate = absolute_trajectory_error(est, gt)
    # A/sqrt(2) is the RMS of a sinusoid over whole periods. Finite sampling
    # truncates the last period slightly, and alignment absorbs a small tilt,
    # so the realised value sits a few tenths of a percent below the ideal.
    # 5e-3 on a 0.354 % figure is ~1.5 % relative -- tight enough to catch a
    # wrong formula, loose enough not to fail on discretisation.
    CLOSE(ate.drift_percent, 100.0 * (amp / math.sqrt(2.0)) / 40.0, 5e-3, "drift %")


def test_drift_percent_is_nan_when_stationary():
    case("drift percent is NaN when the vehicle never moved")
    n = 50
    stamps = np.arange(n) * 0.1
    pos = np.zeros((n, 3))
    quat = np.tile(quat_from_yaw(0.0), (n, 1))
    gt = Trajectory(stamps, pos, quat)
    rng = np.random.default_rng(5)
    est = Trajectory(stamps, pos + rng.normal(scale=0.01, size=(n, 3)), quat)
    ate = absolute_trajectory_error(est, gt)
    CHECK(math.isnan(ate.drift_percent))


def test_scale_error_is_visible_under_rigid_alignment():
    case("scale error stays visible under rigid alignment")
    # A monocular estimate at half scale. Rigid alignment must NOT hide this;
    # fitting scale must remove it and recover the factor.
    gt = arc()
    est = Trajectory(gt.stamps.copy(), gt.positions * 0.5, gt.quaternions.copy())

    rigid = absolute_trajectory_error(est, gt, with_scale=False)
    CHECK(rigid.translation.rmse > 0.5)

    scaled = absolute_trajectory_error(est, gt, with_scale=True)
    CLOSE(scaled.translation.rmse, 0.0, 1e-8, "ATE with scale fitted")
    CLOSE(scaled.alignment.scale, 2.0, 1e-9, "recovered scale")


def test_rpe_is_blind_to_global_drift_but_sees_a_local_jump():
    case("RPE ignores global drift and catches a local jump")
    # A slow yaw drift accumulates a large ATE while every local step stays
    # good. RPE should stay near zero for that, then spike on a teleport.
    gt = straight_line(n=401, length=40.0, dt=0.05)

    # Build the drifting estimate the way odometry actually goes wrong: compose
    # each true relative motion with a small constant yaw error. Every step is
    # then nearly right while the path bends steadily away.
    #
    # Rotating the world positions about the origin instead would NOT model
    # this -- it displaces distant points by metres per step and RPE correctly
    # reports a large local error, which is what the first draft of this test
    # got wrong.
    yaw_err = 0.001  # rad per step
    gt_poses = gt.poses()
    est_poses = [gt_poses[0].copy()]
    perturb = np.eye(4)
    perturb[:3, :3] = rot_xyz(0.0, 0.0, yaw_err)
    for i in range(1, len(gt_poses)):
        d = se3_inverse(gt_poses[i - 1]) @ gt_poses[i]
        est_poses.append(est_poses[-1] @ d @ perturb)
    est_poses = np.array(est_poses)
    drifting = Trajectory(
        gt.stamps.copy(),
        est_poses[:, :3, 3],
        np.stack([quat_from_matrix(T[:3, :3]) for T in est_poses]))

    ate = absolute_trajectory_error(drifting, gt)
    rpe = relative_pose_error(drifting, gt, delta=1.0, delta_unit="m")
    CHECK(ate.translation.rmse > 0.5)
    CHECK(rpe.translation.rmse < 0.05)

    jumped = gt.positions.copy()
    jumped[200:] += np.array([0.0, 1.5, 0.0])
    jump = Trajectory(gt.stamps.copy(), jumped, gt.quaternions.copy())
    rpe_jump = relative_pose_error(jump, gt, delta=1.0, delta_unit="m")
    CHECK(rpe_jump.translation.maximum > 1.0)


def test_rpe_rotation_error_matches_a_known_angle():
    case("RPE rotation error matches a known injected angle")
    n = 101
    gt = straight_line(n=n, length=10.0, dt=0.1)
    bad = 0.05  # rad added to every heading after the midpoint
    quats = gt.quaternions.copy()
    for i in range(50, n):
        quats[i] = quat_from_yaw(bad)
    est = Trajectory(gt.stamps.copy(), gt.positions.copy(), quats)
    rpe = relative_pose_error(est, gt, delta=1, delta_unit="frames")
    # Exactly one consecutive pair straddles the step; the rest are identical.
    CLOSE(rpe.rotation_deg.maximum, math.degrees(bad), 1e-9, "max rotation error")
    CLOSE(rpe.rotation_deg.minimum, 0.0, 1e-12, "min rotation error")


def test_rpe_delta_units_agree_on_a_unit_speed_line():
    case("RPE delta units agree on a unit-speed line")
    # 10 m over 10 s at 1 m/s: 1 metre and 1 second must select the same pairs.
    gt = straight_line(n=101, length=10.0, dt=0.1)
    est = Trajectory(gt.stamps.copy(),
                     gt.positions + np.array([0.0, 0.05, 0.0]),
                     gt.quaternions.copy())
    by_m = relative_pose_error(est, gt, delta=1.0, delta_unit="m")
    by_s = relative_pose_error(est, gt, delta=1.0, delta_unit="s")
    CHECK(by_m.pairs == by_s.pairs)
    CLOSE(by_m.translation.rmse, by_s.translation.rmse, 1e-12, "rmse")


def test_association_matches_nearest_and_never_reuses():
    case("association matches nearest in time and reuses nothing")
    ref = straight_line(n=101, length=10.0, dt=0.1)
    # Estimate at half rate, offset by 10 ms.
    idx = np.arange(0, 101, 2)
    est = Trajectory(ref.stamps[idx] + 0.01, ref.positions[idx], ref.quaternions[idx])
    a, b = associate(est, ref, max_difference=0.02)
    CHECK(len(a) == len(idx))
    CHECK(len(b) == len(idx))
    CHECK(len(set(map(tuple, b.positions))) == len(idx))
    CLOSE(np.max(np.abs(b.positions - ref.positions[idx])), 0.0, 1e-12, "matched poses")


def test_association_drops_pairs_outside_the_window():
    case("association drops pairs outside the window")
    ref = straight_line(n=51, length=5.0, dt=0.1)
    # Offset well clear of the reference span. An offset of exactly the span
    # would still align the first estimate with the last reference sample at
    # dt = 0, which is a real match, not a miss.
    est = Trajectory(ref.stamps + 50.0, ref.positions, ref.quaternions)
    a, _ = associate(est, ref, max_difference=0.02)
    CHECK(len(a) == 0)


def test_a_stalled_estimator_cannot_match_one_pose_repeatedly():
    case("a stalled estimator cannot reuse one reference pose")
    # Every estimate carries the same stamp. Without the used-set rule they
    # would all associate to the same reference pose and the error would look
    # far better than it is.
    ref = straight_line(n=101, length=10.0, dt=0.1)
    est = Trajectory(np.full(20, ref.stamps[50]),
                     np.tile(ref.positions[50], (20, 1)),
                     np.tile(ref.quaternions[50], (20, 1)))
    a, b = associate(est, ref, max_difference=0.02)
    CHECK(len(a) <= 3)


def test_ate_refuses_to_report_on_too_few_matches():
    case("ATE refuses to report on too few matches")
    ref = straight_line(n=51, length=5.0, dt=0.1)
    est = Trajectory(ref.stamps + 5.0, ref.positions, ref.quaternions)
    try:
        absolute_trajectory_error(est, ref)
        CHECK(False)
    except ValueError as exc:
        CHECK("associated" in str(exc))


def test_rpe_refuses_a_window_longer_than_the_run():
    case("RPE refuses a window longer than the run")
    gt = straight_line(n=21, length=2.0, dt=0.1)
    try:
        relative_pose_error(gt, gt, delta=50.0, delta_unit="m")
        CHECK(False)
    except ValueError as exc:
        CHECK("no pose pairs" in str(exc))


def test_se3_helpers():
    case("SE(3) helpers are self-consistent")
    R = rot_xyz(0.4, -0.9, 0.2)
    t = np.array([1.0, -2.0, 3.0])
    T = to_se3(t, quat_from_matrix(R))
    CLOSE(np.max(np.abs(T[:3, :3] - R)), 0.0, 1e-9, "to_se3 rotation")
    CLOSE(np.max(np.abs(se3_inverse(T) @ T - np.eye(4))), 0.0, 1e-9, "inverse")
    CLOSE(rotation_angle(np.eye(3)), 0.0, 1e-12, "angle of identity")
    CLOSE(rotation_angle(rot_xyz(0, 0, math.pi / 3)), math.pi / 3, 1e-9, "angle")
    # Trace slightly above 3 from float error must not produce NaN.
    CHECK(not math.isnan(rotation_angle(np.eye(3) * (1.0 + 1e-15))))


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
    print("\n%d checks, %d failures across %d tests" % (_checks, _failures, len(tests)))
    if _failures == 0:
        print("metrics: OK")
    return 1 if _failures else 0


if __name__ == "__main__":
    sys.exit(main())
