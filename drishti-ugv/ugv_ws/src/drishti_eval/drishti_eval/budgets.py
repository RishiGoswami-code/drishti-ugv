# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
"""Performance budgets, checked against measured samples.

SPEC.md §8 states the budgets. This module decides whether a profiled run met
them, which is the first item of TASK.md Phase 7 and the thing every
optimisation is measured against afterwards.

No ROS. Sequences of numbers in, verdicts out.

---------------------------------------------------------------------------
BUDGETS ARE NOT MET ON AVERAGE

A perception pipeline that averages 60 ms and spikes to 400 ms twice a second
has not met a 100 ms budget. It missed it on exactly the frames where something
was happening, because the spikes correlate with scene complexity -- more
objects, more work, and that is precisely when a late detection matters.

So every budget here is judged on a high percentile and on the maximum, never
on the mean. The mean is reported because it is useful for tuning, and it is
never what decides pass or fail.

The same argument applies in reverse to rates: a control loop that averages
25 Hz while dropping to 4 Hz under load is not a 20 Hz loop. Rates are judged
on a LOW percentile and the minimum.
---------------------------------------------------------------------------
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional, Sequence


class Sense(str, Enum):
    """Which direction is good."""

    LOWER_IS_BETTER = "lower"      # latencies, drift
    HIGHER_IS_BETTER = "higher"    # loop rates


@dataclass(frozen=True)
class Budget:
    """One SPEC.md §8 line."""

    name: str
    unit: str
    prototype: float
    stretch: float
    sense: Sense
    #: Percentile the verdict is taken at. 95 for latencies (the tail is the
    #: point), 5 for rates (the trough is the point).
    percentile: float = 95.0

    def better(self, a: float, b: float) -> bool:
        """Is `a` better than `b`?"""
        return a < b if self.sense is Sense.LOWER_IS_BETTER else a > b


#: SPEC.md §8, verbatim. Perception latency and control rate are the two the
#: RTX 3050 is most likely to miss (STATUS.md D15).
BUDGETS: Dict[str, Budget] = {
    "perception_latency_ms": Budget(
        "perception latency", "ms", prototype=100.0, stretch=60.0,
        sense=Sense.LOWER_IS_BETTER, percentile=95.0),
    "control_rate_hz": Budget(
        "control loop", "Hz", prototype=20.0, stretch=30.0,
        sense=Sense.HIGHER_IS_BETTER, percentile=5.0),
    "planner_rate_hz": Budget(
        "planner update", "Hz", prototype=5.0, stretch=10.0,
        sense=Sense.HIGHER_IS_BETTER, percentile=5.0),
    "stop_latency_ms": Budget(
        "emergency stop", "ms", prototype=200.0, stretch=100.0,
        sense=Sense.LOWER_IS_BETTER, percentile=100.0),
    "localisation_drift_percent": Budget(
        "localisation drift", "%", prototype=2.0, stretch=1.5,
        sense=Sense.LOWER_IS_BETTER, percentile=95.0),
}


@dataclass
class BudgetResult:
    key: str
    budget: Budget
    samples: int
    value: float            #: the percentile the verdict is taken at
    mean: float
    worst: float
    meets_prototype: bool
    meets_stretch: bool
    note: str = ""

    def as_dict(self) -> dict:
        return {
            "key": self.key, "unit": self.budget.unit,
            "samples": self.samples, "percentile": self.budget.percentile,
            "value": self.value, "mean": self.mean, "worst": self.worst,
            "prototype": self.budget.prototype,
            "stretch": self.budget.stretch,
            "meets_prototype": self.meets_prototype,
            "meets_stretch": self.meets_stretch,
            "note": self.note,
        }


def _percentile(values: Sequence[float], p: float) -> float:
    """Linear-interpolated percentile. Deliberately not numpy: this module is
    imported by tooling that may run outside the ROS environment."""
    ordered = sorted(values)
    if not ordered:
        return float("nan")
    if len(ordered) == 1:
        return float(ordered[0])
    k = (len(ordered) - 1) * (min(max(p, 0.0), 100.0) / 100.0)
    lo = math.floor(k)
    hi = math.ceil(k)
    if lo == hi:
        return float(ordered[int(k)])
    return float(ordered[lo] + (ordered[hi] - ordered[lo]) * (k - lo))


def check(key: str, samples: Sequence[float],
          min_samples: int = 30) -> BudgetResult:
    """Judge one budget from measured samples.

    Refuses on too few samples rather than reporting a verdict from three
    readings. A tail percentile computed from a handful of numbers is not a
    tail; it is the largest of a handful of numbers, and treating it as a pass
    would be the easiest way to claim a budget we never measured.
    """
    if key not in BUDGETS:
        raise KeyError("unknown budget %r; known: %s"
                       % (key, ", ".join(sorted(BUDGETS))))
    budget = BUDGETS[key]

    clean = [float(v) for v in samples
             if isinstance(v, (int, float)) and math.isfinite(v)]
    discarded = len(samples) - len(clean)

    if not clean:
        return BudgetResult(key, budget, 0, float("nan"), float("nan"),
                            float("nan"), False, False,
                            "no finite samples; nothing was measured")

    value = _percentile(clean, budget.percentile)
    mean = sum(clean) / len(clean)
    worst = max(clean) if budget.sense is Sense.LOWER_IS_BETTER else min(clean)

    note = ""
    if discarded:
        note = "%d non-finite sample(s) discarded" % discarded

    if len(clean) < min_samples:
        return BudgetResult(
            key, budget, len(clean), value, mean, worst, False, False,
            ("only %d sample(s); at least %d are needed before a %.0fth "
             "percentile means anything" % (len(clean), min_samples,
                                            budget.percentile)))

    return BudgetResult(
        key, budget, len(clean), value, mean, worst,
        meets_prototype=budget.better(value, budget.prototype),
        meets_stretch=budget.better(value, budget.stretch),
        note=note)


def check_all(measurements: Dict[str, Sequence[float]],
              min_samples: int = 30) -> List[BudgetResult]:
    return [check(k, v, min_samples) for k, v in sorted(measurements.items())]


def format_results(results: Sequence[BudgetResult]) -> str:
    lines = ["performance budgets (SPEC.md §8)"]
    for r in results:
        verdict = "PASS" if r.meets_prototype else "FAIL"
        if r.meets_prototype and r.meets_stretch:
            verdict = "PASS+"          # also inside the stretch budget
        lines.append(
            "  %-22s p%-3.0f %8.2f %-3s  mean %8.2f  worst %8.2f  "
            "(proto %g / comp %g)  %s"
            % (r.budget.name, r.budget.percentile, r.value, r.budget.unit,
               r.mean, r.worst, r.budget.prototype, r.budget.stretch,
               verdict))
        if r.note:
            lines.append("      %s" % r.note)
    if not any(r.meets_prototype for r in results):
        lines.append("")
        lines.append("  Nothing met its prototype budget. Check that the "
                     "samples came from a loaded run, not an idle one.")
    return "\n".join(lines)
