# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
"""Tests for the Phase 5 safety harness: fault schedules and stop latency.

Stop latency is the number the safety case rests on. SPEC.md §8 budgets
< 200 ms, the report quotes it, and this module decides whether we met
it. A measurement that is subtly generous would let us report passing a budget
we missed -- so most of what follows is about refusing to measure rather than
about measuring.

    python test/test_safety_harness.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from drishti_eval.faults import (  # noqa: E402
    FAILURE_SCENARIOS, Fault, FaultKind, FaultSchedule, scenario)
from drishti_eval.latency import (  # noqa: E402
    BUDGET_STRETCH_S, BUDGET_PROTOTYPE_S, Sample, format_summary,
    measure_all, measure_stop, summarise)

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
        print("  FAIL  [%s] line %d %s got %r want %.9g"
              % (_case, inspect.currentframe().f_back.f_lineno, note, a, b))


def driving(t0=0.0, t1=20.0, hz=50.0, speed=0.8):
    """A vehicle driving steadily for the whole window."""
    n = int((t1 - t0) * hz)
    return [Sample(t0 + i / hz, speed, 0.1) for i in range(n)]


def driving_then_stopping(fault_t, latency, hz=50.0, speed=0.8, end=20.0):
    """Driving, then commanded to zero exactly `latency` after the fault."""
    out = []
    n = int(end * hz)
    for i in range(n):
        t = i / hz
        if t < fault_t + latency:
            out.append(Sample(t, speed, 0.1))
        else:
            out.append(Sample(t, 0.0, 0.0))
    return out


# ======================================================== fault schedules
def test_a_fault_is_active_only_inside_its_window():
    case("a fault is active only inside its window")
    f = Fault(5.0, 2.0, "/topic", FaultKind.SILENCE)
    CHECK(not f.active_at(4.99))
    CHECK(f.active_at(5.0))
    CHECK(f.active_at(6.9))
    CHECK(not f.active_at(7.0), "end is exclusive")
    CLOSE(f.end_s, 7.0)


def test_an_open_ended_fault_never_lifts():
    case("an open-ended fault never lifts")
    f = Fault(5.0, None, "/topic", FaultKind.SILENCE)
    CHECK(f.active_at(5.0))
    CHECK(f.active_at(10_000.0))
    CHECK(f.end_s is None)


def test_a_schedule_rejects_nonsense():
    case("a schedule rejects nonsense")
    for bad in (Fault(-1.0, None, "/t", FaultKind.SILENCE),
                Fault(1.0, 0.0, "/t", FaultKind.SILENCE),
                Fault(1.0, -3.0, "/t", FaultKind.SILENCE),
                Fault(1.0, None, "", FaultKind.SILENCE)):
        try:
            FaultSchedule((bad,))
            CHECK(False, "accepted %r" % (bad,))
        except ValueError:
            CHECK(True)


def test_overlapping_faults_resolve_to_the_more_severe():
    case("overlapping faults resolve to the more severe")
    # Authoring mistake rather than a feature, but it must resolve the same
    # way every run or two identical-looking runs would differ.
    s = FaultSchedule((
        Fault(0.0, None, "/cam", FaultKind.STALE_STAMP),
        Fault(0.0, None, "/cam", FaultKind.SILENCE),
    ))
    CHECK(s.kind_for("/cam", 1.0) is FaultKind.SILENCE)
    CHECK(s.is_suppressed("/cam", 1.0))

    reversed_order = FaultSchedule((
        Fault(0.0, None, "/cam", FaultKind.SILENCE),
        Fault(0.0, None, "/cam", FaultKind.STALE_STAMP),
    ))
    CHECK(reversed_order.kind_for("/cam", 1.0) is FaultKind.SILENCE,
          "declaration order must not matter")


def test_an_untouched_topic_is_never_suppressed():
    case("an untouched topic is never suppressed")
    s = scenario("T16_camera_dropout")
    CHECK(s.kind_for("/imu/data", 100.0) is None)
    CHECK(not s.is_suppressed("/imu/data", 100.0))


def test_freeze_is_distinct_from_silence():
    case("freeze is distinct from silence")
    # The whole point of having both: a frozen camera keeps arriving with a
    # fresh stamp and is not caught by a liveness check.
    frozen = scenario("T16_camera_freeze")
    CHECK(frozen.kind_for("/camera/rgb/image_raw", 9.0) is FaultKind.FREEZE)
    CHECK(not frozen.is_suppressed("/camera/rgb/image_raw", 9.0),
          "a frozen stream still publishes")

    silent = scenario("T16_camera_dropout")
    CHECK(silent.is_suppressed("/camera/rgb/image_raw", 9.0))


def test_every_catalogued_scenario_is_well_formed():
    case("every catalogued scenario is well formed")
    for name, sched in FAILURE_SCENARIOS.items():
        CHECK(len(sched.faults) >= 1, name)
        CHECK(len(sched.injection_times()) == len(sched.faults), name)
        for f in sched.faults:
            # Fire well after the start so the baseline is genuinely moving;
            # latency.py rejects a measurement taken from rest.
            CHECK(f.start_s >= 5.0, "%s fires too early at %.1f s"
                  % (name, f.start_s))
            CHECK(f.duration_s is None,
                  "%s recovers; T16-T19 require a safe halt, not a recovery"
                  % name)


def test_t16_to_t19_are_all_present():
    case("T16-T19 are all covered")
    covered = {f.scenario for s in FAILURE_SCENARIOS.values() for f in s.faults}
    for t in ("T16", "T17", "T18", "T19"):
        CHECK(t in covered, "%s has no scenario" % t)


def test_injection_times_are_sorted():
    case("injection times come out sorted")
    times = scenario("T16_T18_combined").injection_times()
    CHECK([t for t, _ in times] == sorted(t for t, _ in times))
    CHECK(len(times) == 2)


def test_unknown_scenario_names_are_rejected_loudly():
    case("an unknown scenario name is rejected loudly")
    try:
        scenario("T99_nonsense")
        CHECK(False, "should have raised")
    except KeyError as exc:
        CHECK("Known:" in str(exc), "must list what is available")


# ========================================================= stop latency
def test_a_clean_stop_is_measured_exactly():
    case("a clean stop is measured exactly")
    # Latencies chosen well clear of both budgets. Asserting behaviour exactly
    # AT a threshold with sample times generated as i/hz tests float
    # representation rather than the code -- 405/50 is not exactly 8.1.
    fast = measure_stop(driving_then_stopping(8.0, 0.04), 8.0, "camera")
    CHECK(fast.valid, fast.reason)
    CLOSE(fast.latency_s, 0.04, 1e-6)
    CHECK(fast.meets_prototype_budget)
    CHECK(fast.meets_stretch_budget)

    middling = measure_stop(driving_then_stopping(8.0, 0.16), 8.0, "camera")
    CHECK(middling.valid, middling.reason)
    CLOSE(middling.latency_s, 0.16, 1e-6)
    CHECK(middling.meets_prototype_budget, "0.16 < 0.200")
    CHECK(not middling.meets_stretch_budget, "0.16 is not < 0.100")

    slow = measure_stop(driving_then_stopping(8.0, 0.30), 8.0, "camera")
    CHECK(slow.valid, slow.reason)
    CHECK(not slow.meets_prototype_budget, "0.30 exceeds 0.200")


def test_a_stop_from_rest_is_invalid_not_fast():
    case("a stop measured from rest is invalid, not fast")
    # THE trap. A stationary vehicle produces a near-zero latency that is
    # really just the publication period, and averaged into a suite it would
    # drag the reported figure far below the truth.
    stationary = [Sample(i / 50.0, 0.0, 0.0) for i in range(1000)]
    m = measure_stop(stationary, 8.0, "camera")
    CHECK(not m.valid)
    CHECK(m.latency_s is None, "must not report a number")
    CHECK("not moving" in m.reason, m.reason)


def test_a_decelerating_baseline_still_counts_as_moving():
    case("a decelerating baseline still counts as moving")
    # Peak, not mean: a vehicle slowing into the fault was still moving, and
    # the mean over the window would understate that.
    #
    # It must still be moving AT the fault -- an earlier draft decelerated to
    # exactly 0.0 at t = 8.0, which is a genuinely stopped vehicle, and the
    # already-stopped guard correctly rejected it.
    samples = []
    for i in range(1000):
        t = i / 50.0
        # Still 0.4 m/s at the fault; commanded to zero 0.12 s later.
        speed = max(0.0, 0.8 - 0.05 * t) if t < 8.12 else 0.0
        samples.append(Sample(t, speed, 0.0))
    m = measure_stop(samples, 8.0, "camera")
    CHECK(m.valid, m.reason)
    CHECK(m.baseline_speed > 0.3, "peak should reflect the earlier speed")
    CLOSE(m.latency_s, 0.12, 1e-6)


def test_a_run_with_no_stop_reports_failure_not_a_big_number():
    case("a run that never stops reports failure, not a large latency")
    m = measure_stop(driving(), 8.0, "camera")
    CHECK(not m.valid)
    CHECK(m.latency_s is None)
    CHECK("never commanded to stop" in m.reason, m.reason)


def test_a_stop_after_the_timeout_does_not_count():
    case("a stop after the timeout does not count")
    m = measure_stop(driving_then_stopping(8.0, 3.0), 8.0, "camera",
                     timeout_s=1.0)
    CHECK(not m.valid)
    CHECK(m.latency_s is None)


def test_an_empty_or_truncated_bag_is_refused():
    case("an empty or truncated bag is refused")
    CHECK(not measure_stop([], 8.0).valid)
    truncated = [Sample(i / 50.0, 0.8, 0.0) for i in range(300)]   # ends at 6 s
    m = measure_stop(truncated, 8.0)
    CHECK(not m.valid)
    CHECK("ends before" in m.reason, m.reason)


def test_a_non_finite_fault_time_is_refused():
    case("a non-finite fault time is refused")
    CHECK(not measure_stop(driving(), float("nan")).valid)


def test_angular_motion_alone_counts_as_not_stopped():
    case("spinning in place is not stopped")
    # A vehicle rotating with zero linear velocity is still moving, and a
    # command that only zeroes linear_x has not stopped it.
    samples = []
    for i in range(1000):
        t = i / 50.0
        samples.append(Sample(t, 0.0, 0.9) if t < 8.2 else Sample(t, 0.0, 0.0))
    m = measure_stop(samples, 8.0, "camera")
    CHECK(m.valid, m.reason)
    CLOSE(m.latency_s, 0.2, 1e-6)


def test_decision_latency_is_reported_separately():
    case("supervisor decision latency is reported separately")
    # Separating the decision from the command path is what tells us whether a
    # slow stop was the supervisor thinking or the transport.
    samples = driving_then_stopping(8.0, 0.12)
    flags = [(7.0, False), (8.04, True), (9.0, True)]
    m = measure_stop(samples, 8.0, "camera", stop_flags=flags)
    CLOSE(m.latency_s, 0.12, 1e-9)
    CLOSE(m.decision_latency_s, 0.04, 1e-9)


def test_out_of_order_samples_are_sorted_before_measuring():
    case("out-of-order samples are sorted before measuring")
    samples = driving_then_stopping(8.0, 0.10)
    shuffled = samples[::-1]
    CLOSE(measure_stop(shuffled, 8.0).latency_s, 0.10, 1e-9)


# ============================================================== summary
def test_the_summary_is_governed_by_the_worst_case():
    case("the summary is governed by the worst case, not the mean")
    # A safety budget is not met on average. One 400 ms stop is a failure even
    # among forty good ones.
    good = [measure_stop(driving_then_stopping(8.0, 0.05), 8.0) for _ in range(40)]
    bad = measure_stop(driving_then_stopping(8.0, 0.40), 8.0)
    s = summarise(good + [bad])
    CHECK(s.measured == 41)
    CLOSE(s.worst_s, 0.40, 1e-9)
    CHECK(not s.meets_prototype, "worst case exceeds 200 ms")
    CHECK(s.mean_s < BUDGET_PROTOTYPE_S, "the mean would have passed")
    CHECK("FAIL" in format_summary(s))


def test_invalid_measurements_are_counted_and_explained():
    case("invalid measurements are counted, never silently dropped")
    stationary = [Sample(i / 50.0, 0.0, 0.0) for i in range(1000)]
    s = summarise([measure_stop(driving_then_stopping(8.0, 0.05), 8.0),
                   measure_stop(stationary, 8.0),
                   measure_stop(driving(), 8.0)])
    CHECK(s.measured == 1)
    CHECK(s.invalid == 2)
    CHECK(len(s.invalid_reasons) == 2)
    text = format_summary(s)
    CHECK("excluded as invalid" in text)


def test_a_suite_with_nothing_measurable_never_passes():
    case("a suite with nothing measurable never passes")
    # The failure mode this guards: every fault produced no measurable stop,
    # and an empty list of latencies quietly summarises as "no violations".
    stationary = [Sample(i / 50.0, 0.0, 0.0) for i in range(500)]
    s = summarise([measure_stop(stationary, 8.0) for _ in range(5)])
    CHECK(s.measured == 0)
    CHECK(not s.meets_prototype)
    CHECK(not s.meets_stretch)
    CHECK("NO VALID MEASUREMENTS" in format_summary(s))


def test_measure_all_walks_a_schedule():
    case("measure_all walks a whole schedule")
    sched = scenario("T16_T18_combined")
    faults = sched.injection_times()
    samples = driving_then_stopping(8.0, 0.06, end=25.0)
    results = measure_all(samples, faults)
    CHECK(len(results) == 2)
    # The first fault produces a real stop; the second lands after the vehicle
    # is already stopped, so it must be rejected rather than counted as fast.
    CHECK(results[0].valid, results[0].reason)
    CHECK(not results[1].valid, "second fault measured from rest")


def test_budget_constants_match_the_spec():
    case("budget constants match SPEC.md 8")
    CLOSE(BUDGET_PROTOTYPE_S, 0.200)
    CLOSE(BUDGET_STRETCH_S, 0.100)


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
    print("\n%d checks, %d failures across %d tests"
          % (_checks, _failures, len(tests)))
    if _failures == 0:
        print("safety harness: OK")
    return 1 if _failures else 0


if __name__ == "__main__":
    sys.exit(main())
