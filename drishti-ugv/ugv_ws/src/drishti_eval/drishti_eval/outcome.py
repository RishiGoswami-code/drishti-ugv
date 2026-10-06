# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
"""Mission outcome classification.

EVALUATION.md §2.1 fixes the definitions, and this module is where they stop
being prose:

    Success   = goal pose reached within goal_tolerance, ZERO collisions,
                within mission_timeout.
    Collision = any contact between the vehicle body and a non-ground object,
                from the simulator's contact sensor. Not a proximity threshold.
    Outcome classes are exhaustive: success, collision, safe_abort, timeout,
                planner_failure, harness_error. Every mission lands in exactly
                one.

No ROS. A mission is reduced to a handful of facts and classified from those,
so the rule can be tested and argued about without running anything.

---------------------------------------------------------------------------
WHY THE ORDER MATTERS MORE THAN THE RULES

A run can satisfy several descriptions at once. A vehicle that clips a rock and
then reaches the goal is both "reached the goal" and "collided". A vehicle that
halts safely and then runs out of time is both `safe_abort` and `timeout`.
Without a fixed precedence, two people classifying the same bag get different
answers and the suite number means nothing.

Precedence here, worst first:

    harness_error > collision > planner_failure > safe_abort > timeout > success

Success is LAST, and it is the only class that requires every other condition
to be absent. That asymmetry is deliberate: it is the direction in which being
wrong is expensive. A collision that reached the goal is a collision.
---------------------------------------------------------------------------
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Optional, Sequence, Tuple


class Outcome(str, Enum):
    """EVALUATION.md §2.1. Exhaustive and mutually exclusive."""

    SUCCESS = "success"
    COLLISION = "collision"
    SAFE_ABORT = "safe_abort"
    TIMEOUT = "timeout"
    PLANNER_FAILURE = "planner_failure"
    HARNESS_ERROR = "harness_error"


#: Worst first. classify() returns the first class whose condition holds.
PRECEDENCE: Tuple[Outcome, ...] = (
    Outcome.HARNESS_ERROR,
    Outcome.COLLISION,
    Outcome.PLANNER_FAILURE,
    Outcome.SAFE_ABORT,
    Outcome.TIMEOUT,
    Outcome.SUCCESS,
)

#: Only these count toward the headline navigation numbers. A harness error is
#: our bug, not the robot's behaviour, and folding it into either the numerator
#: or the denominator would corrupt the result.
SCORABLE: Tuple[Outcome, ...] = tuple(
    o for o in Outcome if o is not Outcome.HARNESS_ERROR)


@dataclass
class MissionFacts:
    """What a run produced, reduced to what classification needs.

    Every field defaults to the value that does NOT claim success. A fact the
    harness failed to extract must never make a run look better than it was.
    """

    scenario: str = ""
    seed: int = 0

    #: Metres from the final pose to the goal. NaN when it could not be
    #: computed, which is a harness error rather than a distant goal.
    final_distance_to_goal_m: float = float("nan")
    goal_tolerance_m: float = 0.25

    elapsed_s: float = float("nan")
    mission_timeout_s: float = 300.0

    #: From the simulator contact sensor. EVALUATION.md §2.1: contact, never
    #: proximity.
    collisions: int = 0

    #: The supervisor commanded a halt and never released it.
    safety_halted: bool = False
    #: Reason code from the last SafetyState, for triage.
    safety_reason: str = ""

    #: Nav2 gave up: no valid path, or recovery exhausted.
    planner_failed: bool = False

    #: The run itself was broken: a node crashed, the bag is short, ground
    #: truth is missing.
    harness_error: bool = False
    harness_error_detail: str = ""

    #: Populated by the caller from drishti_eval.metrics.
    drift_percent: float = float("nan")
    #: Worst emergency-stop latency observed, seconds.
    worst_stop_latency_s: Optional[float] = None

    def reached_goal(self) -> bool:
        d = self.final_distance_to_goal_m
        return math.isfinite(d) and d <= self.goal_tolerance_m

    def timed_out(self) -> bool:
        return (math.isfinite(self.elapsed_s) and
                self.elapsed_s > self.mission_timeout_s)


@dataclass
class Classification:
    outcome: Outcome
    reason: str

    @property
    def scorable(self) -> bool:
        return self.outcome in SCORABLE


def classify(facts: MissionFacts) -> Classification:
    """Assign exactly one outcome. See the module docstring for precedence."""
    # A run whose facts could not be established is a harness error, not a
    # navigation result. Judging it either way would be inventing data.
    if facts.harness_error:
        return Classification(
            Outcome.HARNESS_ERROR,
            facts.harness_error_detail or "harness reported an error")

    if not math.isfinite(facts.elapsed_s):
        return Classification(Outcome.HARNESS_ERROR,
                              "elapsed time is not finite; the run cannot be timed")

    if not math.isfinite(facts.final_distance_to_goal_m):
        return Classification(
            Outcome.HARNESS_ERROR,
            "final distance to goal could not be computed; without ground "
            "truth the run is not evaluable (EVALUATION.md §7.1)")

    # Contact outranks everything the robot did afterwards. Reaching the goal
    # after hitting something is not a success.
    if facts.collisions > 0:
        return Classification(
            Outcome.COLLISION,
            "%d contact event(s) with a non-ground object" % facts.collisions)

    if facts.planner_failed:
        return Classification(Outcome.PLANNER_FAILURE,
                              "planner reported no valid path or exhausted recovery")

    # A safe halt is a distinct outcome, not a failure and not a success
    # (EVALUATION.md §2.1). Ranked above timeout because a vehicle that stopped
    # deliberately and then ran out the clock stopped for the safety reason.
    if facts.safety_halted and not facts.reached_goal():
        return Classification(
            Outcome.SAFE_ABORT,
            "supervisor halted and did not release: %s"
            % (facts.safety_reason or "reason not recorded"))

    if facts.timed_out():
        return Classification(
            Outcome.TIMEOUT,
            "%.1f s exceeds the %.1f s budget" % (facts.elapsed_s,
                                                  facts.mission_timeout_s))

    if facts.reached_goal():
        return Classification(
            Outcome.SUCCESS,
            "goal reached within %.2f m in %.1f s"
            % (facts.goal_tolerance_m, facts.elapsed_s))

    # Inside the time budget, no collision, no halt, no planner failure, and
    # still not at the goal. The run simply stopped short; that is a timeout in
    # substance even if the clock had not run out.
    return Classification(
        Outcome.TIMEOUT,
        "run ended %.2f m from the goal without reaching it"
        % facts.final_distance_to_goal_m)


@dataclass
class SuiteSummary:
    """Headline numbers for a set of missions."""

    total: int
    scorable: int
    counts: Dict[str, int] = field(default_factory=dict)

    collision_free_rate: float = float("nan")
    goal_completion_rate: float = float("nan")
    safe_abort_rate: float = float("nan")

    worst_drift_percent: Optional[float] = None
    worst_stop_latency_s: Optional[float] = None

    #: EVALUATION.md §3 prototype targets.
    target_collision_free: float = 0.95
    target_goal_completion: float = 0.97

    @property
    def meets_targets(self) -> bool:
        if self.scorable == 0:
            return False
        return (self.collision_free_rate >= self.target_collision_free and
                self.goal_completion_rate >= self.target_goal_completion)

    def as_dict(self) -> dict:
        return {
            "total": self.total,
            "scorable": self.scorable,
            "counts": dict(self.counts),
            "collision_free_rate": self.collision_free_rate,
            "goal_completion_rate": self.goal_completion_rate,
            "safe_abort_rate": self.safe_abort_rate,
            "worst_drift_percent": self.worst_drift_percent,
            "worst_stop_latency_s": self.worst_stop_latency_s,
            "target_collision_free": self.target_collision_free,
            "target_goal_completion": self.target_goal_completion,
            "meets_targets": self.meets_targets,
        }


def summarise(results: Sequence[Tuple[MissionFacts, Classification]]
              ) -> SuiteSummary:
    """Roll missions up into the EVALUATION.md §2 headline rates.

    Harness errors are excluded from both numerator and denominator: they are
    our bugs, not the robot's behaviour. They are still counted and reported,
    because a suite that was 40% broken is a finding in itself.
    """
    counts: Dict[str, int] = {o.value: 0 for o in Outcome}
    for _, c in results:
        counts[c.outcome.value] += 1

    scorable = [(f, c) for f, c in results if c.scorable]
    n = len(scorable)

    summary = SuiteSummary(total=len(results), scorable=n, counts=counts)
    if n == 0:
        # No rate is defined over zero runs. NaN, not 1.0: an empty suite must
        # never look like a perfect one.
        return summary

    collision_free = sum(1 for _, c in scorable
                         if c.outcome is not Outcome.COLLISION)
    reached = sum(1 for _, c in scorable if c.outcome is Outcome.SUCCESS)
    aborted = sum(1 for _, c in scorable if c.outcome is Outcome.SAFE_ABORT)

    summary.collision_free_rate = collision_free / n
    summary.goal_completion_rate = reached / n
    summary.safe_abort_rate = aborted / n

    drifts = [f.drift_percent for f, _ in scorable
              if math.isfinite(f.drift_percent)]
    if drifts:
        summary.worst_drift_percent = max(drifts)

    latencies = [f.worst_stop_latency_s for f, _ in scorable
                 if f.worst_stop_latency_s is not None]
    if latencies:
        summary.worst_stop_latency_s = max(latencies)

    return summary


def format_summary(summary: SuiteSummary) -> str:
    lines = ["mission suite: %d run(s), %d scorable"
             % (summary.total, summary.scorable)]

    for outcome in Outcome:
        n = summary.counts.get(outcome.value, 0)
        if n:
            share = (100.0 * n / summary.total) if summary.total else 0.0
            lines.append("  %-16s %4d  (%5.1f %%)" % (outcome.value, n, share))

    if summary.scorable == 0:
        lines.append("")
        lines.append("  NO SCORABLE RUNS -- every mission was a harness error.")
        lines.append("  No navigation claim can be made from this suite.")
        return "\n".join(lines)

    lines += [
        "",
        "  collision-free   %6.2f %%   (target >= %.0f %%)   %s"
        % (100.0 * summary.collision_free_rate,
           100.0 * summary.target_collision_free,
           "PASS" if summary.collision_free_rate >= summary.target_collision_free
           else "FAIL"),
        "  goal completion  %6.2f %%   (target >= %.0f %%)   %s"
        % (100.0 * summary.goal_completion_rate,
           100.0 * summary.target_goal_completion,
           "PASS" if summary.goal_completion_rate >= summary.target_goal_completion
           else "FAIL"),
        "  safe abort       %6.2f %%" % (100.0 * summary.safe_abort_rate),
    ]
    if summary.worst_drift_percent is not None:
        lines.append("  worst drift      %6.2f %%" % summary.worst_drift_percent)
    if summary.worst_stop_latency_s is not None:
        lines.append("  worst stop       %6.1f ms"
                     % (summary.worst_stop_latency_s * 1000.0))
    if summary.counts.get(Outcome.HARNESS_ERROR.value):
        lines.append("")
        lines.append("  %d harness error(s) excluded from the rates above."
                     % summary.counts[Outcome.HARNESS_ERROR.value])
    return "\n".join(lines)
