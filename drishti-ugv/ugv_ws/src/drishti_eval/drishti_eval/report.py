# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
"""Turn metric results into a report a human and a machine can both read.

EVALUATION.md 7.2: without the seed and the parameter set, a result is an
anecdote. So every report carries its provenance, and `evaluate_bag` refuses to
pretend a number means more than it does -- the alignment mode is always
stated, because an ATE with scale fitted and one without are different
quantities and must never be compared.

No ROS. `format_text` and `as_dict` are testable directly.
"""
from __future__ import annotations

import json
from typing import Optional

from .metrics import (AteResult, RpeResult, absolute_trajectory_error,
                      relative_pose_error)
from .trajectory import Trajectory

# EVALUATION.md 3
DRIFT_TARGET_PROTOTYPE = 2.0     # percent of distance travelled


def evaluate(estimate: Trajectory, truth: Trajectory,
             with_scale: bool = False,
             rpe_delta: float = 1.0,
             rpe_unit: str = "m",
             max_difference: float = 0.02) -> dict:
    """Compute the localisation section of a run report."""
    ate: AteResult = absolute_trajectory_error(
        estimate, truth, with_scale=with_scale, max_difference=max_difference)

    # RPE needs at least one pair separated by the window. A run that stopped
    # short has none -- and EVALUATION.md 2.1 makes `safe_abort` a first-class
    # outcome, so those runs are expected, not exceptional. Losing the whole
    # report (including a perfectly good ATE) because the vehicle halted early
    # would suppress exactly the runs the safety case depends on.
    rpe: Optional[RpeResult] = None
    rpe_unavailable = None
    try:
        rpe = relative_pose_error(
            estimate, truth, delta=rpe_delta, delta_unit=rpe_unit,
            max_difference=max_difference)
    except ValueError as exc:
        rpe_unavailable = str(exc)

    rpe_section = {
        "delta": rpe_delta,
        "delta_unit": rpe_unit,
        "available": rpe is not None,
    }
    if rpe is not None:
        rpe_section.update({
            "pairs": rpe.pairs,
            "translation_m": rpe.translation.as_dict(),
            "rotation_deg": rpe.rotation_deg.as_dict(),
        })
    else:
        rpe_section["reason"] = rpe_unavailable

    return {
        "samples": {
            "estimate": len(estimate),
            "ground_truth": len(truth),
            "associated": ate.translation.count,
            "max_association_difference_s": max_difference,
        },
        "path": {
            "length_m": ate.path_length,
            "duration_s": truth.duration(),
        },
        "ate": {
            "alignment": "sim3_scale_fitted" if with_scale else "se3_rigid",
            "recovered_scale": ate.alignment.scale,
            "translation_m": ate.translation.as_dict(),
            "drift_percent": ate.drift_percent,
            "target_percent": DRIFT_TARGET_PROTOTYPE,
            "meets_target": (ate.drift_percent == ate.drift_percent
                             and ate.drift_percent < DRIFT_TARGET_PROTOTYPE),
        },
        "rpe": rpe_section,
    }


def format_text(report: dict, title: Optional[str] = None) -> str:
    """Human-readable summary. Stable enough to diff between runs."""
    a, r, p, s = report["ate"], report["rpe"], report["path"], report["samples"]
    lines = []
    if title:
        lines += [title, "=" * len(title), ""]

    lines += [
        "samples      %d estimate, %d ground truth, %d associated (<= %g s)"
        % (s["estimate"], s["ground_truth"], s["associated"],
           s["max_association_difference_s"]),
        "path         %.2f m over %.1f s" % (p["length_m"], p["duration_s"]),
        "",
        "ATE          alignment: %s" % a["alignment"],
    ]
    if a["alignment"] != "se3_rigid":
        lines.append("             recovered scale: %.6f  "
                     "(NOT comparable with a rigid-aligned run)"
                     % a["recovered_scale"])
    t = a["translation_m"]
    lines += [
        "             rmse %.4f m   mean %.4f   median %.4f   max %.4f"
        % (t["rmse"], t["mean"], t["median"], t["max"]),
        "             drift %.3f %% of distance travelled  (target < %.1f %%)  %s"
        % (a["drift_percent"], a["target_percent"],
           "PASS" if a["meets_target"] else "FAIL"),
        "",
    ]

    if r.get("available"):
        lines += [
            "RPE          window %g %s, %d pairs"
            % (r["delta"], r["delta_unit"], r["pairs"]),
            "             translation rmse %.4f m   max %.4f"
            % (r["translation_m"]["rmse"], r["translation_m"]["max"]),
            "             rotation    rmse %.4f deg max %.4f"
            % (r["rotation_deg"]["rmse"], r["rotation_deg"]["max"]),
        ]
    else:
        lines.append("RPE          unavailable: %s" % r.get("reason", "unknown"))

    return "\n".join(lines)


def to_json(report: dict) -> str:
    return json.dumps(report, indent=2, sort_keys=True)
