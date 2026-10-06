# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
"""Tests for the Phase 7 gates: performance budgets and regression checking.

Both modules exist to stop a specific kind of self-deception.

`budgets` stops "we met the latency budget on average" -- which is not meeting
it, because the spikes correlate with the frames that mattered.

`regression` stops "the rate did not change significantly, so nothing broke" --
which, from a suite too small to detect the drop, is an admission dressed up as
a green light.

    python test/test_phase7.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from drishti_eval.budgets import (  # noqa: E402
    BUDGETS, Sense, check, check_all, format_results)
from drishti_eval.regression import (  # noqa: E402
    RegressionReport, Verdict, compare_budget, compare_rate, detectable_drop,
    format_report)

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


def CLOSE(a, b, tol=1e-9, note=""):
    global _checks, _failures
    import inspect
    _checks += 1
    if a is None or not abs(float(a) - float(b)) <= tol:
        _failures += 1
        print("  FAIL  [%s] line %d %s got %r want %r"
              % (_case, inspect.currentframe().f_back.f_lineno, note, a, b))


# ============================================================== budgets
def test_budget_constants_match_the_spec():
    case("budget constants match SPEC.md 8")
    CLOSE(BUDGETS["perception_latency_ms"].prototype, 100.0)
    CLOSE(BUDGETS["perception_latency_ms"].stretch, 60.0)
    CLOSE(BUDGETS["control_rate_hz"].prototype, 20.0)
    CLOSE(BUDGETS["planner_rate_hz"].prototype, 5.0)
    CLOSE(BUDGETS["stop_latency_ms"].prototype, 200.0)
    CLOSE(BUDGETS["localisation_drift_percent"].prototype, 2.0)


def test_a_good_tail_passes():
    case("a pipeline with a clean tail passes")
    samples = [45.0 + (i % 7) for i in range(200)]     # 45-51 ms
    r = check("perception_latency_ms", samples)
    CHECK(r.meets_prototype, r.note)
    CHECK(r.meets_stretch, "45-51 ms is inside the 60 ms budget too")

    # 55-61 ms passes the prototype budget but its p95 sits just over the
    # stretch one. The boundary is where the distinction earns its keep.
    marginal = check("perception_latency_ms",
                     [55.0 + (i % 7) for i in range(200)])
    CHECK(marginal.meets_prototype)
    CHECK(not marginal.meets_stretch, "p95 = %.1f" % marginal.value)


def test_a_good_mean_with_a_bad_tail_fails():
    case("a good mean with a bad tail FAILS")
    # THE point of this module. 90% of frames at 40 ms and 10% at 400 ms
    # averages 76 ms -- comfortably inside a 100 ms budget -- and misses it on
    # exactly the frames where the scene was busy.
    samples = [40.0] * 180 + [400.0] * 20
    r = check("perception_latency_ms", samples)
    CHECK(r.mean < 100.0, "the mean does pass: %.1f" % r.mean)
    CHECK(not r.meets_prototype, "but the p95 must not")
    CHECK(r.value > 100.0, "p95 = %.1f" % r.value)


def test_rates_are_judged_on_the_trough_not_the_average():
    case("rates are judged on the trough, not the average")
    # A control loop that averages 25 Hz but drops to 4 Hz under load is not a
    # 20 Hz loop, and the drop is when it mattered.
    samples = [30.0] * 180 + [4.0] * 20
    r = check("control_rate_hz", samples)
    CHECK(r.mean > 20.0, "mean passes: %.1f" % r.mean)
    CHECK(not r.meets_prototype, "p5 must not")


def test_a_steady_rate_passes():
    case("a steady rate passes")
    r = check("control_rate_hz", [24.0 + (i % 3) for i in range(200)])
    CHECK(r.meets_prototype)
    CHECK(not r.meets_stretch, "24-26 Hz is short of 30")


def test_too_few_samples_refuses_a_verdict():
    case("too few samples refuses a verdict")
    # A 95th percentile of five numbers is the largest of five numbers.
    r = check("perception_latency_ms", [10.0, 11.0, 12.0, 9.0, 10.5])
    CHECK(not r.meets_prototype, "must not pass on 5 samples")
    CHECK("at least" in r.note, r.note)


def test_no_samples_reports_nothing_measured():
    case("no samples reports nothing measured")
    r = check("perception_latency_ms", [])
    CHECK(not r.meets_prototype)
    CHECK(r.samples == 0)
    CHECK("nothing was measured" in r.note)


def test_non_finite_samples_are_discarded_and_counted():
    case("non-finite samples are discarded and counted")
    samples = [50.0] * 100 + [float("nan"), float("inf")]
    r = check("perception_latency_ms", samples)
    CHECK(r.samples == 100)
    CHECK("2 non-finite" in r.note, r.note)
    CHECK(r.meets_prototype)


def test_stop_latency_is_judged_on_the_maximum():
    case("stop latency is judged on the maximum")
    # A safety budget is not met on average, and not at p95 either. One slow
    # stop is a failure.
    b = BUDGETS["stop_latency_ms"]
    CLOSE(b.percentile, 100.0)
    samples = [40.0] * 199 + [350.0]
    r = check("stop_latency_ms", samples)
    CHECK(not r.meets_prototype, "one 350 ms stop must fail")


def test_an_unknown_budget_is_rejected_loudly():
    case("an unknown budget key is rejected loudly")
    try:
        check("frames_per_fortnight", [1.0] * 50)
        CHECK(False, "should have raised")
    except KeyError as exc:
        CHECK("known:" in str(exc).lower())


def test_check_all_and_formatting():
    case("check_all reports every budget")
    results = check_all({
        "perception_latency_ms": [50.0] * 100,
        "control_rate_hz": [25.0] * 100,
    })
    CHECK(len(results) == 2)
    text = format_results(results)
    CHECK("perception latency" in text)
    CHECK("PASS" in text)


# =========================================================== regression
def test_an_identical_suite_is_unchanged_when_large_enough():
    case("an identical suite is unchanged, given enough missions")
    c = compare_rate("collision_free", 1900, 2000, 1900, 2000)
    CHECK(c.verdict is Verdict.UNCHANGED, c.detail)
    CHECK(c.acceptable)


def test_identical_small_suites_are_still_underpowered():
    case("identical SMALL suites are underpowered, not unchanged")
    # Uncomfortable but correct. Two identical 200-mission suites still cannot
    # rule out a 2-point regression: the comparison could only have detected a
    # 5.4-point drop. Calling that "unchanged" would be the exact false
    # reassurance this module exists to refuse, and it is refused even when the
    # numbers happen to match perfectly.
    c = compare_rate("collision_free", 190, 200, 190, 200, min_effect=0.02)
    CHECK(c.verdict is Verdict.UNDERPOWERED, c.detail)
    CHECK(not c.acceptable)
    CLOSE(c.difference, 0.0, 1e-12, "the rates are identical")


def test_a_real_drop_is_caught():
    case("a real drop is caught")
    # 97% -> 85% over 400 missions each is unambiguous.
    c = compare_rate("collision_free", 388, 400, 340, 400)
    CHECK(c.verdict is Verdict.REGRESSION, c.detail)
    CHECK(not c.acceptable)
    CHECK(c.p_value < 0.05)


def test_a_real_improvement_is_recognised():
    case("a real improvement is recognised")
    c = compare_rate("goal_completion", 340, 400, 388, 400)
    CHECK(c.verdict is Verdict.IMPROVEMENT, c.detail)
    CHECK(c.acceptable, "an improvement passes the gate")


def test_an_underpowered_comparison_is_not_a_pass():
    case("an underpowered comparison is NOT a pass")
    # THE point of this module. 95% -> 92% over 100 missions each is not
    # statistically detectable, and reporting "no regression" would be an
    # admission dressed up as a green light.
    c = compare_rate("collision_free", 95, 100, 92, 100, min_effect=0.02)
    CHECK(c.verdict is Verdict.UNDERPOWERED, c.detail)
    CHECK(not c.acceptable, "underpowered must not pass the Phase 7 gate")
    CHECK(c.detectable_drop > 0.02, "MDE %.3f" % c.detectable_drop)
    CHECK("Run more missions" in c.detail)


def test_a_large_suite_can_reach_a_real_conclusion():
    case("a large enough suite reaches a real conclusion")
    # The same 3-point gap, with enough missions behind it, becomes decidable.
    c = compare_rate("collision_free", 9500, 10000, 9200, 10000,
                     min_effect=0.02)
    CHECK(c.verdict is Verdict.REGRESSION, c.detail)
    CHECK(c.detectable_drop < 0.02)


def test_tiny_suites_are_refused_outright():
    case("tiny suites are refused outright")
    c = compare_rate("collision_free", 9, 10, 8, 10)
    CHECK(c.verdict is Verdict.UNDERPOWERED, c.detail)
    CHECK("fewer than" in c.detail)


def test_an_empty_suite_yields_no_data():
    case("an empty suite yields no data, not a pass")
    c = compare_rate("collision_free", 0, 0, 95, 100)
    CHECK(c.verdict is Verdict.NO_DATA)
    CHECK(not c.acceptable)


def test_detectable_drop_shrinks_as_the_suite_grows():
    case("detectable drop shrinks as the suite grows")
    small = detectable_drop(0.95, 100, 100)
    medium = detectable_drop(0.95, 1000, 1000)
    large = detectable_drop(0.95, 10000, 10000)
    CHECK(small > medium > large, "%.4f %.4f %.4f" % (small, medium, large))

    # The numbers that matter for planning the suite, at a 95 % baseline:
    #
    #     100 per side -> 7.7 points
    #     200          -> 5.4
    #    1000          -> 2.4
    #    1470          -> 2.0
    #    2000          -> 1.7
    #
    # So detecting a 2-point regression needs about 1470 missions per side.
    # TASK.md Phase 6 says "scale toward 1000", which is short of it -- see
    # STATUS.md D20.
    CHECK(small > 0.05, "100 per side resolves only %.1f points" % (100 * small))
    CHECK(medium > 0.02, "even 1000 per side is short: %.2f points"
          % (100 * medium))
    CHECK(detectable_drop(0.95, 1470, 1470) <= 0.0201, "1470 should reach 2 points")
    CHECK(detectable_drop(0.95, 2000, 2000) < 0.02, "2000 per side clears it")


def test_detectable_drop_survives_a_degenerate_rate():
    case("detectable drop survives a 100 % rate")
    # p = 1.0 makes the variance term zero and the naive formula returns 0,
    # which would claim infinite sensitivity.
    d = detectable_drop(1.0, 100, 100)
    CHECK(math.isfinite(d) and d > 0.0, "got %r" % d)


# ------------------------------------------------------ continuous metrics
def test_a_latency_that_got_worse_is_a_regression_even_if_still_passing():
    case("spending headroom is a regression even if still passing")
    # 50 -> 80 ms is still inside the 100 ms budget, but the headroom is gone.
    # Without flagging it, headroom gets spent silently until one change tips
    # it over and no single commit looks responsible.
    c = compare_budget("perception_latency_ms", 50.0, 80.0)
    CHECK(c.verdict is Verdict.REGRESSION, c.detail)
    CHECK(not c.acceptable)
    CHECK("headroom was spent" in c.detail)


def test_a_latency_that_broke_the_budget_says_so():
    case("a latency that broke the budget says so")
    c = compare_budget("perception_latency_ms", 90.0, 140.0)
    CHECK(c.verdict is Verdict.REGRESSION)
    CHECK("outside the prototype budget" in c.detail)


def test_small_wobble_is_within_tolerance():
    case("small wobble is within tolerance")
    c = compare_budget("perception_latency_ms", 50.0, 53.0)
    CHECK(c.verdict is Verdict.UNCHANGED, c.detail)
    CHECK(c.acceptable)


def test_a_faster_pipeline_is_an_improvement():
    case("a faster pipeline is an improvement")
    c = compare_budget("perception_latency_ms", 90.0, 45.0)
    CHECK(c.verdict is Verdict.IMPROVEMENT, c.detail)
    CHECK(c.acceptable)


def test_rate_metrics_use_the_opposite_sense():
    case("rate metrics use the opposite sense")
    # For a rate, going DOWN is worse.
    worse = compare_budget("control_rate_hz", 30.0, 21.0)
    CHECK(worse.verdict is Verdict.REGRESSION, worse.detail)
    better = compare_budget("control_rate_hz", 21.0, 30.0)
    CHECK(better.verdict is Verdict.IMPROVEMENT, better.detail)


def test_an_unmeasured_metric_is_no_data():
    case("an unmeasured metric is no_data, not a pass")
    c = compare_budget("perception_latency_ms", float("nan"), 50.0)
    CHECK(c.verdict is Verdict.NO_DATA)
    CHECK(not c.acceptable)


# ------------------------------------------------------------- the gate
def test_the_gate_rejects_any_regression():
    case("the gate rejects if anything regressed")
    report = RegressionReport(
        rates=[compare_rate("collision_free", 9500, 10000, 9200, 10000)],
        metrics=[compare_budget("perception_latency_ms", 50.0, 52.0)])
    CHECK(not report.acceptable)
    CHECK("collision_free" in report.regressions)
    CHECK("REJECTED" in format_report(report))


def test_the_gate_rejects_an_underpowered_suite():
    case("the gate rejects an underpowered suite")
    report = RegressionReport(
        rates=[compare_rate("collision_free", 95, 100, 94, 100)],
        metrics=[])
    CHECK(not report.acceptable, "silence from a small suite is not a pass")
    CHECK(report.underpowered == ["collision_free"])
    CHECK("not evidence of no change" in format_report(report))


def test_the_gate_accepts_a_clean_comparison():
    case("the gate accepts a clean, powered comparison")
    report = RegressionReport(
        rates=[compare_rate("collision_free", 9500, 10000, 9510, 10000)],
        metrics=[compare_budget("perception_latency_ms", 50.0, 48.0)])
    CHECK(report.acceptable, format_report(report))
    CHECK("ACCEPTED" in format_report(report))


def test_an_empty_gate_is_not_a_pass():
    case("a gate with nothing in it is not a pass")
    # Running no comparisons must not read as "nothing regressed".
    CHECK(not RegressionReport(rates=[], metrics=[]).acceptable)


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
    print("\n%d checks, %d failures across %d tests"
          % (_checks, _failures, len(tests)))
    if _failures == 0:
        print("phase 7 gates: OK")
    return 1 if _failures else 0


if __name__ == "__main__":
    sys.exit(main())
