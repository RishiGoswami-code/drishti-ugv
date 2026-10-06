# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
"""Tests for Phase 6: outcome classification and mission generation.

These two modules decide what the headline number IS. EVALUATION.md §3 sets
collision-free ≥ 95% and goal completion ≥ 97%, the report quotes those,
and a classifier that is generous in one branch moves the number without
anything looking wrong.

So most of what follows is about the cases where a run could plausibly be
called a success and must not be.

    python test/test_suite.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from drishti_eval.outcome import (  # noqa: E402
    Classification, MissionFacts, Outcome, PRECEDENCE, SCORABLE, classify,
    format_summary, summarise)
from drishti_eval.scenarios import (  # noqa: E402
    SPAWN_Z, WORLDS, coverage, generate, missing_scenarios, suite)

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


def clean_success(**kw):
    facts = MissionFacts(
        scenario="T01", seed=1,
        final_distance_to_goal_m=0.10, goal_tolerance_m=0.25,
        elapsed_s=42.0, mission_timeout_s=300.0)
    for k, v in kw.items():
        setattr(facts, k, v)
    return facts


# ================================================== outcome classification
def test_a_clean_run_is_a_success():
    case("a clean run is a success")
    c = classify(clean_success())
    CHECK(c.outcome is Outcome.SUCCESS, c.reason)
    CHECK(c.scorable)


def test_reaching_the_goal_after_a_collision_is_still_a_collision():
    case("reaching the goal after a collision is still a collision")
    # The single most important precedence rule. A stack that clipped a rock
    # and carried on must not be able to report a success.
    c = classify(clean_success(collisions=1))
    CHECK(c.outcome is Outcome.COLLISION, c.reason)


def test_outcomes_are_exhaustive_and_mutually_exclusive():
    case("outcomes are exhaustive and mutually exclusive")
    # EVALUATION.md 2.1. Every class must appear exactly once in the
    # precedence list, or some run has no home or two.
    CHECK(len(PRECEDENCE) == len(Outcome))
    CHECK(set(PRECEDENCE) == set(Outcome))
    CHECK(len(set(PRECEDENCE)) == len(PRECEDENCE), "a class appears twice")


def test_defaults_never_claim_success():
    case("default facts never classify as success")
    # A harness that failed to extract anything must not produce a green run.
    c = classify(MissionFacts())
    CHECK(c.outcome is Outcome.HARNESS_ERROR, c.reason)
    CHECK(not c.scorable)


def test_a_missing_ground_truth_distance_is_a_harness_error():
    case("a missing goal distance is a harness error, not a failure")
    # EVALUATION.md 7.1: a run without ground truth is not evaluable. Calling
    # it a timeout would blame the robot for our missing data.
    c = classify(clean_success(final_distance_to_goal_m=float("nan")))
    CHECK(c.outcome is Outcome.HARNESS_ERROR, c.reason)
    CHECK("not evaluable" in c.reason)


def test_a_missing_elapsed_time_is_a_harness_error():
    case("a missing elapsed time is a harness error")
    c = classify(clean_success(elapsed_s=float("nan")))
    CHECK(c.outcome is Outcome.HARNESS_ERROR)


def test_a_safe_halt_short_of_the_goal_is_a_safe_abort():
    case("a safe halt short of the goal is safe_abort, not failure")
    c = classify(clean_success(final_distance_to_goal_m=4.0,
                               safety_halted=True,
                               safety_reason="camera frozen"))
    CHECK(c.outcome is Outcome.SAFE_ABORT, c.reason)
    CHECK("camera frozen" in c.reason)
    CHECK(c.scorable, "safe aborts still count in the denominator")


def test_a_safe_halt_at_the_goal_is_still_a_success():
    case("halting safely once already at the goal is a success")
    # The supervisor stopping the vehicle after arrival is correct behaviour,
    # not an abort.
    c = classify(clean_success(safety_halted=True, safety_reason="arrived"))
    CHECK(c.outcome is Outcome.SUCCESS, c.reason)


def test_safe_abort_outranks_timeout():
    case("safe_abort outranks timeout")
    # A vehicle that halted deliberately and then ran out the clock stopped for
    # the safety reason; reporting a timeout would hide why.
    c = classify(clean_success(final_distance_to_goal_m=5.0,
                               safety_halted=True, elapsed_s=400.0))
    CHECK(c.outcome is Outcome.SAFE_ABORT, c.reason)


def test_collision_outranks_planner_failure_and_abort():
    case("collision outranks planner failure and safe abort")
    c = classify(clean_success(collisions=2, planner_failed=True,
                               safety_halted=True,
                               final_distance_to_goal_m=9.0))
    CHECK(c.outcome is Outcome.COLLISION)


def test_harness_error_outranks_everything():
    case("harness error outranks everything, including a collision")
    c = classify(clean_success(collisions=3, harness_error=True,
                               harness_error_detail="bag truncated"))
    CHECK(c.outcome is Outcome.HARNESS_ERROR)
    CHECK("bag truncated" in c.reason)


def test_a_planner_failure_is_its_own_class():
    case("a planner failure is its own class")
    c = classify(clean_success(final_distance_to_goal_m=6.0,
                               planner_failed=True))
    CHECK(c.outcome is Outcome.PLANNER_FAILURE)


def test_running_out_of_time_is_a_timeout():
    case("running out of time is a timeout")
    c = classify(clean_success(final_distance_to_goal_m=3.0, elapsed_s=301.0))
    CHECK(c.outcome is Outcome.TIMEOUT, c.reason)


def test_stopping_short_inside_the_budget_is_still_not_a_success():
    case("stopping short inside the time budget is not a success")
    # No collision, no halt, no planner failure, plenty of time left, and the
    # vehicle simply is not at the goal. It must not fall through to success.
    c = classify(clean_success(final_distance_to_goal_m=7.5, elapsed_s=50.0))
    CHECK(c.outcome is Outcome.TIMEOUT, c.reason)
    CHECK("without reaching it" in c.reason)


def test_the_goal_tolerance_boundary_is_inclusive():
    case("the goal tolerance boundary is inclusive")
    CHECK(classify(clean_success(final_distance_to_goal_m=0.25)).outcome
          is Outcome.SUCCESS)
    CHECK(classify(clean_success(final_distance_to_goal_m=0.2501)).outcome
          is Outcome.TIMEOUT)


def test_the_timeout_boundary_is_exclusive():
    case("exactly at the timeout is not yet a timeout")
    CHECK(classify(clean_success(elapsed_s=300.0)).outcome is Outcome.SUCCESS)
    CHECK(classify(clean_success(final_distance_to_goal_m=3.0,
                                 elapsed_s=300.01)).outcome is Outcome.TIMEOUT)


# ============================================================== summary
def run(**kw):
    f = clean_success(**kw)
    return (f, classify(f))


def test_rates_are_computed_over_scorable_runs_only():
    case("rates exclude harness errors from both sides")
    # A harness error is our bug. Counting it as a failure would understate the
    # robot; counting it as a success would overstate it. It belongs in
    # neither.
    results = [run() for _ in range(8)]
    results += [run(harness_error=True, harness_error_detail="crash")
                for _ in range(2)]
    s = summarise(results)
    CHECK(s.total == 10)
    CHECK(s.scorable == 8)
    CLOSE(s.collision_free_rate, 1.0)
    CLOSE(s.goal_completion_rate, 1.0)
    CHECK(s.counts["harness_error"] == 2)
    CHECK("2 harness error(s) excluded" in format_summary(s))


def test_an_empty_suite_is_never_a_pass():
    case("an empty suite is never a pass")
    # NaN rather than 1.0. Zero successes out of zero runs must not read as
    # perfect.
    s = summarise([])
    CHECK(s.total == 0)
    CHECK(math.isnan(s.collision_free_rate))
    CHECK(not s.meets_targets)


def test_a_suite_of_only_harness_errors_makes_no_claim():
    case("a suite of only harness errors makes no navigation claim")
    s = summarise([run(harness_error=True) for _ in range(20)])
    CHECK(s.scorable == 0)
    CHECK(not s.meets_targets)
    CHECK("NO SCORABLE RUNS" in format_summary(s))


def test_the_targets_are_the_evaluation_numbers():
    case("targets match EVALUATION.md 3")
    s = summarise([run()])
    CLOSE(s.target_collision_free, 0.95)
    CLOSE(s.target_goal_completion, 0.97)


def test_one_collision_in_twenty_fails_the_target():
    case("one collision in twenty fails the 95 % target")
    results = [run() for _ in range(19)] + [run(collisions=1)]
    s = summarise(results)
    CLOSE(s.collision_free_rate, 0.95)
    CHECK(s.collision_free_rate >= s.target_collision_free, "95 % exactly passes")

    results = [run() for _ in range(18)] + [run(collisions=1), run(collisions=1)]
    s2 = summarise(results)
    CLOSE(s2.collision_free_rate, 0.90)
    CHECK(not s2.meets_targets)
    CHECK("FAIL" in format_summary(s2))


def test_safe_aborts_lower_goal_completion_without_being_collisions():
    case("safe aborts lower goal completion but not collision-free rate")
    results = [run() for _ in range(9)]
    results.append(run(final_distance_to_goal_m=5.0, safety_halted=True))
    s = summarise(results)
    CLOSE(s.collision_free_rate, 1.0, 1e-9, "an abort is not a collision")
    CLOSE(s.goal_completion_rate, 0.9)
    CLOSE(s.safe_abort_rate, 0.1)


def test_worst_case_drift_and_latency_are_carried_up():
    case("worst-case drift and latency are carried up, not averaged")
    results = [run(drift_percent=0.4, worst_stop_latency_s=0.05),
               run(drift_percent=2.6, worst_stop_latency_s=0.31),
               run(drift_percent=0.9, worst_stop_latency_s=0.08)]
    s = summarise(results)
    CLOSE(s.worst_drift_percent, 2.6)
    CLOSE(s.worst_stop_latency_s, 0.31)


# =========================================================== scenarios
def test_a_seed_reproduces_a_mission_exactly():
    case("a seed reproduces a mission exactly")
    # EVALUATION.md 7.2: without the seed, a result is an anecdote. Run #417
    # must be regenerable by anyone.
    a = generate(417)
    b = generate(417)
    CHECK(a == b, "same seed gave different missions")
    CHECK(generate(418) != a, "different seeds should differ")


def test_generation_does_not_depend_on_call_order():
    case("generation does not depend on call order")
    # A shared global RNG would make mission 5 depend on whether 1-4 ran
    # first, and a suite would not be reproducible piecemeal.
    direct = generate(99)
    for s in (1, 2, 3, 50):
        generate(s)
    CHECK(generate(99) == direct)


def test_a_suite_uses_consecutive_seeds():
    case("a suite uses consecutive seeds")
    missions = suite(10, base_seed=1000)
    CHECK(len(missions) == 10)
    CHECK([m.seed for m in missions] == list(range(1000, 1010)))
    CHECK(missions[3] == generate(1003), "any run is regenerable alone")


def test_an_empty_suite_is_allowed_and_a_negative_one_is_not():
    case("an empty suite is allowed, a negative one is not")
    CHECK(suite(0) == [])
    try:
        suite(-1)
        CHECK(False, "should have raised")
    except ValueError:
        CHECK(True)


def test_every_world_has_a_spawn_height():
    case("every world has a spawn height")
    # hard.sdf drives on a raised platform; the default z drops the vehicle
    # into the ditch at t=0 and looks like a catastrophic failure.
    for world in WORLDS:
        CHECK(world in SPAWN_Z, "%s has no spawn height" % world)
    CHECK(SPAWN_Z["hard.sdf"] > SPAWN_Z["easy.sdf"], "hard.sdf sits higher")


def test_missions_are_generated_only_for_known_worlds():
    case("an unknown world is rejected loudly")
    try:
        generate(1, world="atlantis.sdf")
        CHECK(False, "should have raised")
    except KeyError as exc:
        CHECK("known:" in str(exc).lower())


def test_a_scenario_belongs_to_its_world():
    case("a generated scenario belongs to its world")
    for seed in range(200):
        m = generate(seed)
        CHECK(m.scenario in WORLDS[m.world],
              "%s is not in %s" % (m.scenario, m.world))


def test_hard_world_goals_lie_beyond_the_ditch():
    case("hard.sdf goals lie beyond the ditch")
    # The ditch spans x in [11.0, 12.6]. A goal short of it would let T07 pass
    # without ever facing the obstacle it exists to test.
    for seed in range(300):
        m = generate(seed, world="hard.sdf")
        CHECK(m.goal_xy[0] > 12.6, "goal at x=%.2f is before the ditch"
              % m.goal_xy[0])
        CLOSE(m.spawn_z, 0.60)


def test_randomisation_stays_inside_its_declared_envelope():
    case("randomisation stays inside its declared envelope")
    for seed in range(300):
        r = generate(seed).randomisation
        # Never below the horizon: night belongs to the Adversarial world, and
        # letting it leak in would make Easy and Medium incomparable.
        CHECK(25.0 <= r.sun_elevation_deg <= 80.0, "sun below the horizon")
        CHECK(0.0 <= r.sun_azimuth_deg <= 360.0)
        CHECK(0.0 < r.camera_noise_stddev < 0.05)
        CHECK(0.0 <= r.depth_dropout_fraction <= 0.05)
        CHECK(0.5 < r.ground_friction <= 1.0)


def test_randomisation_actually_varies():
    case("randomisation actually varies across seeds")
    # A generator that returned the same conditions every time would produce a
    # suite of 1000 identical runs and a very convincing, meaningless number.
    suns = {generate(s).randomisation.sun_azimuth_deg for s in range(100)}
    CHECK(len(suns) > 90, "only %d distinct sun angles in 100" % len(suns))


def test_missions_are_a_real_traverse():
    case("missions are a real traverse, not a nudge")
    for seed in range(200):
        m = generate(seed)
        CHECK(m.straight_line_distance_m > 5.0,
              "goal only %.1f m away" % m.straight_line_distance_m)


def test_launch_arguments_carry_the_spawn_height():
    case("launch arguments carry the spawn height")
    args = generate(7, world="hard.sdf").launch_arguments()
    CHECK(args["world"] == "hard.sdf")
    CHECK(abs(float(args["z"]) - 0.60) < 1e-6)
    CHECK(args["headless"] == "true")


def test_coverage_reports_what_a_suite_actually_exercised():
    case("coverage reports what a suite exercised")
    # A 500-run suite that never touched T07 has not tested the ditch, however
    # good its headline rate looks.
    missions = suite(200, base_seed=0)
    counts = coverage(missions)
    CHECK(sum(counts.values()) == 200)
    CHECK(len(counts) > 1, "a suite that only ran one scenario")

    only_easy = suite(20, base_seed=0, world="easy.sdf")
    missing = missing_scenarios(only_easy, ["T01", "T02", "T07"])
    CHECK(missing == ["T07"], "got %r" % missing)


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
    print("\n%d checks, %d failures across %d tests"
          % (_checks, _failures, len(tests)))
    if _failures == 0:
        print("suite harness: OK")
    return 1 if _failures else 0


if __name__ == "__main__":
    sys.exit(main())
