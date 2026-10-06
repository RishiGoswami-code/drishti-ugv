# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
"""Tests for the run report.

The report is where a number becomes a claim, so the pass/fail logic and the
alignment disclosure get their own tests. EVALUATION.md 3 sets the prototype
drift target at 2 %; a report that rounded that the wrong way, or that quietly
compared a scale-fitted ATE against a rigid one, would be worse than no report.

    python test/test_report.py
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from drishti_eval.report import (  # noqa: E402
    DRIFT_TARGET_PROTOTYPE, evaluate, format_text, to_json)
from drishti_eval.trajectory import Trajectory  # noqa: E402

_checks = 0
_failures = 0
_case = ""


def case(name):
    global _case
    _case = name


def CHECK(cond, note=""):
    global _checks, _failures
    import inspect
    _checks += 1
    if not cond:
        _failures += 1
        print("  FAIL  [%s] line %d %s"
              % (_case, inspect.currentframe().f_back.f_lineno, note))


def line(n=201, length=20.0, dt=0.05, lateral=0.0):
    s = np.linspace(0.0, length, n)
    pos = np.stack([s, np.full(n, lateral), np.zeros(n)], axis=1)
    quat = np.tile(np.array([0.0, 0.0, 0.0, 1.0]), (n, 1))
    return Trajectory(np.arange(n) * dt, pos, quat)


def test_perfect_run_passes_the_drift_target():
    case("a perfect run passes the drift target")
    gt = line()
    rep = evaluate(gt, gt)
    CHECK(rep["ate"]["meets_target"], "should pass")
    CHECK(rep["ate"]["drift_percent"] < 1e-9)
    CHECK(rep["ate"]["alignment"] == "se3_rigid", "rigid by default")
    CHECK(abs(rep["path"]["length_m"] - 20.0) < 1e-9)


def test_a_bad_run_fails_the_drift_target():
    case("a run above 2 % drift fails")
    n = 201
    gt = line(n=n)
    # Zero-mean wobble of amplitude A gives rmse ~ A/sqrt(2); pick A so drift
    # lands clearly above the 2 % target on a 20 m path.
    amp = 1.5
    phase = np.linspace(0.0, 2 * math.pi * 6, n, endpoint=False)
    est = Trajectory(gt.stamps.copy(),
                     gt.positions + np.stack(
                         [np.zeros(n), amp * np.sin(phase), np.zeros(n)], axis=1),
                     gt.quaternions.copy())
    rep = evaluate(est, gt)
    CHECK(rep["ate"]["drift_percent"] > DRIFT_TARGET_PROTOTYPE, "should exceed target")
    CHECK(not rep["ate"]["meets_target"], "should fail")
    CHECK("FAIL" in format_text(rep), "text must say FAIL")


def test_a_stationary_run_never_reports_a_pass():
    case("a stationary run never reports a pass")
    # drift_percent is NaN over zero distance. NaN must not slip through the
    # comparison as a pass -- that would turn "we did not move" into a green
    # result.
    n = 60
    stamps = np.arange(n) * 0.05
    pos = np.zeros((n, 3))
    quat = np.tile(np.array([0.0, 0.0, 0.0, 1.0]), (n, 1))
    gt = Trajectory(stamps, pos, quat)
    rng = np.random.default_rng(2)
    est = Trajectory(stamps, pos + rng.normal(scale=0.02, size=(n, 3)), quat)
    rep = evaluate(est, gt)
    CHECK(math.isnan(rep["ate"]["drift_percent"]), "drift is NaN")
    CHECK(rep["ate"]["meets_target"] is False, "NaN must not pass")
    # RPE has no valid window over zero distance. That must degrade to a
    # stated absence, not take the whole report down with it -- safe_abort is
    # a first-class outcome (EVALUATION.md 2.1) and those runs must still be
    # reportable.
    CHECK(rep["rpe"]["available"] is False, "RPE marked unavailable")
    CHECK("reason" in rep["rpe"], "and says why")
    CHECK("unavailable" in format_text(rep), "text states the absence")
    CHECK(rep["ate"]["translation_m"]["count"] > 0, "ATE still computed")


def test_scale_fitted_reports_are_labelled_and_warned_about():
    case("scale-fitted reports are labelled and warned about")
    gt = line()
    est = Trajectory(gt.stamps.copy(), gt.positions * 0.5, gt.quaternions.copy())
    rep = evaluate(est, gt, with_scale=True)
    CHECK(rep["ate"]["alignment"] == "sim3_scale_fitted")
    CHECK(abs(rep["ate"]["recovered_scale"] - 2.0) < 1e-8, "recovered scale")
    text = format_text(rep)
    CHECK("NOT comparable" in text, "must warn against comparing")

    rigid = evaluate(est, gt, with_scale=False)
    CHECK("NOT comparable" not in format_text(rigid), "no warning when rigid")
    CHECK(rigid["ate"]["translation_m"]["rmse"] >
          rep["ate"]["translation_m"]["rmse"], "rigid must show the scale error")


def test_report_is_json_serialisable():
    case("report is JSON serialisable")
    gt = line()
    text = to_json(evaluate(gt, gt))
    CHECK(text.strip().startswith("{"))
    import json
    CHECK("drift_percent" in json.loads(text)["ate"])


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
    print("\n%d checks, %d failures across %d tests" % (_checks, _failures, len(tests)))
    if _failures == 0:
        print("report: OK")
    return 1 if _failures else 0


if __name__ == "__main__":
    sys.exit(main())
