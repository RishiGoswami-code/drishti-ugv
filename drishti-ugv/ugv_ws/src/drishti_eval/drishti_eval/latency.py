# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
"""Emergency-stop latency, measured from a recorded run.

EVALUATION.md §2.1 pins the definition:

    timestamp of the injected fault -> timestamp of the first /cmd_vel message
    with zero velocity, taken from the bag

SPEC.md §8 budgets < 200 ms for the prototype and < 100 ms for stretch.
This module is the only thing that decides whether we met it, so it is written
to refuse rather than to flatter.

No ROS. Plain sequences of (timestamp, value), so it is testable.

---------------------------------------------------------------------------
THE TRAP THIS MODULE EXISTS TO AVOID

If the vehicle was already stationary when the fault was injected, the "first
zero command after the fault" is the very next message, and the measured
latency is a few milliseconds. That number is not a stop latency -- it is the
publication period of a vehicle that was never moving. Averaged into a suite it
would drag the reported figure far below the truth, and the better the run
went, the more stationary samples it would contribute.

So a measurement whose baseline was not moving is INVALID, not fast. It is
reported as such and excluded from the summary rather than counted as a pass.
---------------------------------------------------------------------------
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

# SPEC.md §8
BUDGET_PROTOTYPE_S = 0.200
BUDGET_STRETCH_S = 0.100

#: A command counts as "stopped" below this. The supervisor publishes exact
#: zeros on STOP, so this only absorbs serialisation noise; it is deliberately
#: far tighter than any speed the vehicle would travel at.
ZERO_SPEED_EPS = 1e-6


@dataclass(frozen=True)
class Sample:
    """One /cmd_vel message: time, and the speeds it commanded."""

    t: float
    linear_x: float
    angular_z: float = 0.0

    @property
    def is_zero(self) -> bool:
        return (abs(self.linear_x) <= ZERO_SPEED_EPS and
                abs(self.angular_z) <= ZERO_SPEED_EPS)


@dataclass
class StopMeasurement:
    """One fault, and what happened after it."""

    fault_t: float
    fault_kind: str
    #: Seconds from fault to the first zero /cmd_vel. None when not measurable.
    latency_s: Optional[float]
    #: Seconds from fault to /safety/stop going true, when that series is
    #: available. Separates the supervisor's decision from the command path.
    decision_latency_s: Optional[float]
    valid: bool
    reason: str
    #: Peak speed seen in the window before the fault, for the record.
    baseline_speed: float = 0.0

    @property
    def meets_prototype_budget(self) -> bool:
        return (self.valid and self.latency_s is not None and
                self.latency_s < BUDGET_PROTOTYPE_S)

    @property
    def meets_stretch_budget(self) -> bool:
        return (self.valid and self.latency_s is not None and
                self.latency_s < BUDGET_STRETCH_S)


def _sorted(samples: Sequence[Sample]) -> List[Sample]:
    return sorted(samples, key=lambda s: s.t)


def baseline_speed(samples: Sequence[Sample], fault_t: float,
                   window_s: float = 1.0) -> float:
    """Peak commanded speed in the window immediately before the fault.

    Peak rather than mean: a vehicle that was decelerating into the fault was
    still moving, and the mean would understate that.
    """
    peak = 0.0
    for s in samples:
        if fault_t - window_s <= s.t <= fault_t:
            peak = max(peak, abs(s.linear_x), abs(s.angular_z))
    return peak


def measure_stop(samples: Sequence[Sample],
                 fault_t: float,
                 fault_kind: str = "unknown",
                 stop_flags: Optional[Sequence[Tuple[float, bool]]] = None,
                 baseline_window_s: float = 1.0,
                 min_baseline_speed: float = 0.05,
                 timeout_s: float = 5.0) -> StopMeasurement:
    """Latency from one injected fault to the vehicle being commanded to stop.

    `samples` is the /cmd_vel series; `stop_flags` is the optional
    /safety/stop series. Both use message header stamps, never bag receive
    time -- transport delay is not stop latency (SPEC.md §3.2 rule 3).
    """
    if not math.isfinite(fault_t):
        return StopMeasurement(fault_t, fault_kind, None, None, False,
                               "fault timestamp is not finite")

    ordered = _sorted(samples)
    if not ordered:
        return StopMeasurement(fault_t, fault_kind, None, None, False,
                               "no /cmd_vel samples in the run")

    peak = baseline_speed(ordered, fault_t, baseline_window_s)
    after = [s for s in ordered if s.t >= fault_t]
    if not after:
        return StopMeasurement(fault_t, fault_kind, None, None, False,
                               "no /cmd_vel samples after the fault; the bag "
                               "ends before the response could be observed",
                               peak)

    # See the module docstring. A stationary baseline makes the measurement
    # meaningless, not fast.
    if peak < min_baseline_speed:
        return StopMeasurement(
            fault_t, fault_kind, None, None, False,
            "vehicle was not moving before the fault (peak %.4f m/s < %.4f); "
            "a stop latency measured from rest is not a stop latency"
            % (peak, min_baseline_speed), peak)

    # The peak alone is not enough. When two faults land close together, the
    # window before the second one still contains the motion that preceded the
    # FIRST -- so a vehicle already halted by fault 1 would look like a moving
    # baseline for fault 2, and report a latency of about one publication
    # period. The state at the instant of the fault is what decides it.
    latest = next((s for s in reversed(ordered) if s.t <= fault_t), None)
    if latest is None:
        return StopMeasurement(
            fault_t, fault_kind, None, None, False,
            "no /cmd_vel samples before the fault; nothing establishes that "
            "the vehicle was moving", peak)
    if latest.is_zero:
        return StopMeasurement(
            fault_t, fault_kind, None, None, False,
            "vehicle was already stopped when the fault was injected "
            "(commonly a second fault arriving after the first has already "
            "halted the vehicle); there is no stop to time", peak)

    first_zero = next((s for s in after
                       if s.is_zero and s.t - fault_t <= timeout_s), None)
    if first_zero is None:
        return StopMeasurement(
            fault_t, fault_kind, None, None, False,
            "no zero-velocity command within %.1f s of the fault; the vehicle "
            "was never commanded to stop" % timeout_s, peak)

    latency = first_zero.t - fault_t

    decision = None
    if stop_flags:
        raised = next((t for t, flag in sorted(stop_flags)
                       if flag and t >= fault_t and t - fault_t <= timeout_s),
                      None)
        if raised is not None:
            decision = raised - fault_t

    return StopMeasurement(fault_t, fault_kind, latency, decision, True,
                           "ok", peak)


def measure_all(samples: Sequence[Sample],
                faults: Sequence[Tuple[float, str]],
                stop_flags: Optional[Sequence[Tuple[float, bool]]] = None,
                **kw) -> List[StopMeasurement]:
    return [measure_stop(samples, t, kind, stop_flags, **kw)
            for t, kind in faults]


@dataclass
class LatencySummary:
    measured: int
    invalid: int
    worst_s: Optional[float]
    mean_s: Optional[float]
    meets_prototype: bool
    meets_stretch: bool
    invalid_reasons: Tuple[str, ...] = ()

    def as_dict(self) -> dict:
        return {
            "measured": self.measured,
            "invalid": self.invalid,
            "worst_s": self.worst_s,
            "mean_s": self.mean_s,
            "budget_prototype_s": BUDGET_PROTOTYPE_S,
            "budget_stretch_s": BUDGET_STRETCH_S,
            "meets_prototype": self.meets_prototype,
            "meets_stretch": self.meets_stretch,
            "invalid_reasons": list(self.invalid_reasons),
        }


def summarise(measurements: Sequence[StopMeasurement]) -> LatencySummary:
    """Roll measurements up, reporting the WORST case, not the average.

    A safety budget is not met on average. One stop that took 400 ms is a
    failure even if forty others took 40 ms, so `meets_prototype` is governed
    by the worst valid measurement.

    Invalid measurements are counted and their reasons kept, never silently
    dropped: a suite where most faults produced no measurable stop is a
    finding, and one that quietly reported the few that worked would hide it.
    """
    valid = [m for m in measurements if m.valid and m.latency_s is not None]
    invalid = [m for m in measurements if not m.valid]

    if not valid:
        return LatencySummary(
            measured=0, invalid=len(invalid), worst_s=None, mean_s=None,
            meets_prototype=False, meets_stretch=False,
            invalid_reasons=tuple(sorted({m.reason for m in invalid})))

    latencies = [m.latency_s for m in valid]
    worst = max(latencies)
    return LatencySummary(
        measured=len(valid),
        invalid=len(invalid),
        worst_s=worst,
        mean_s=sum(latencies) / len(latencies),
        meets_prototype=worst < BUDGET_PROTOTYPE_S,
        meets_stretch=worst < BUDGET_STRETCH_S,
        invalid_reasons=tuple(sorted({m.reason for m in invalid})),
    )


def format_summary(summary: LatencySummary) -> str:
    lines = ["emergency-stop latency"]
    if summary.measured == 0:
        lines.append("  NO VALID MEASUREMENTS (%d invalid)" % summary.invalid)
    else:
        lines += [
            "  worst  %.1f ms   mean %.1f ms   over %d fault(s)"
            % (summary.worst_s * 1000.0, summary.mean_s * 1000.0, summary.measured),
            "  prototype budget < %.0f ms   %s"
            % (BUDGET_PROTOTYPE_S * 1000.0,
               "PASS" if summary.meets_prototype else "FAIL"),
            "  stretch budget < %.0f ms  %s"
            % (BUDGET_STRETCH_S * 1000.0,
               "PASS" if summary.meets_stretch else "FAIL"),
        ]
        if summary.invalid:
            lines.append("  %d measurement(s) excluded as invalid"
                         % summary.invalid)
    for reason in summary.invalid_reasons:
        lines.append("    - %s" % reason)
    return "\n".join(lines)
