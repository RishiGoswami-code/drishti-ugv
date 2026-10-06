# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
"""Did this optimisation make anything worse?

TASK.md Phase 7 acceptance: "post-optimisation suite results are no worse than
pre-optimisation". That sentence hides the whole difficulty, because a suite
result is a SAMPLE. Two runs of the identical stack give different numbers.

No ROS, no scipy. Pure arithmetic, so it is testable.

---------------------------------------------------------------------------
THE FAILURE MODE THIS MODULE EXISTS TO PREVENT

Naive comparison goes wrong in both directions:

  too strict   any drop at all is a regression, so every optimisation is
               blocked by noise and the check gets switched off

  too loose    "the rate went from 95% to 92%, that is within a few points"
               and a real regression ships

But the worse mistake is subtler than either. With 100 missions at a 95% rate,
a drop to 92% is NOT statistically detectable. Reporting "no regression
detected" from that comparison is not a pass -- it is an admission that the
suite was too small to answer the question, dressed up as a green light.

So there are four verdicts, not two, and UNDERPOWERED is distinct from
UNCHANGED. When a comparison cannot detect the effect we care about, it says
so and reports how many missions it would need.
---------------------------------------------------------------------------
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional, Sequence

from .budgets import BUDGETS, Budget, Sense

#: One-sided z for alpha = 0.05, and z for 80% power.
Z_ALPHA_95 = 1.6448536269514722
Z_POWER_80 = 0.8416212335729143


class Verdict(str, Enum):
    REGRESSION = "regression"
    IMPROVEMENT = "improvement"
    UNCHANGED = "unchanged"
    #: The comparison could not have detected the effect we care about.
    UNDERPOWERED = "underpowered"
    #: Not enough data to say anything at all.
    NO_DATA = "no_data"


def _normal_cdf(z: float) -> float:
    return 0.5 * (1.0 + math.erf(z / math.sqrt(2.0)))


@dataclass
class RateComparison:
    """Comparison of two success-style rates."""

    name: str
    before_successes: int
    before_n: int
    after_successes: int
    after_n: int
    before_rate: float
    after_rate: float
    difference: float
    p_value: float
    verdict: Verdict
    #: Smallest drop this comparison could have detected at 80% power.
    detectable_drop: float
    detail: str = ""

    @property
    def acceptable(self) -> bool:
        """Phase 7 acceptance: only a clear non-regression passes.

        UNDERPOWERED is NOT acceptable. A comparison that could not have seen
        the regression is not evidence there was none.
        """
        return self.verdict in (Verdict.UNCHANGED, Verdict.IMPROVEMENT)

    def as_dict(self) -> dict:
        return {
            "name": self.name,
            "before": {"successes": self.before_successes, "n": self.before_n,
                       "rate": self.before_rate},
            "after": {"successes": self.after_successes, "n": self.after_n,
                      "rate": self.after_rate},
            "difference": self.difference,
            "p_value": self.p_value,
            "verdict": self.verdict.value,
            "detectable_drop": self.detectable_drop,
            "acceptable": self.acceptable,
            "detail": self.detail,
        }


def detectable_drop(before_rate: float, n1: int, n2: int,
                    alpha_z: float = Z_ALPHA_95,
                    power_z: float = Z_POWER_80) -> float:
    """Smallest rate drop this pair of sample sizes could detect at 80% power.

    Standard two-proportion formula. It is reported on every comparison, not
    only failing ones, because it is the number that tells you whether the
    suite was big enough to have answered the question.
    """
    if n1 <= 0 or n2 <= 0:
        return float("nan")
    p = min(max(before_rate, 0.0), 1.0)
    # Variance is maximal near p = 0.5 and shrinks at the extremes; guard the
    # degenerate p = 0 or 1 case where the formula yields a meaningless zero.
    variance = max(p * (1.0 - p), 1e-6)
    return (alpha_z + power_z) * math.sqrt(variance * (1.0 / n1 + 1.0 / n2))


def compare_rate(name: str,
                 before_successes: int, before_n: int,
                 after_successes: int, after_n: int,
                 alpha: float = 0.05,
                 min_effect: float = 0.02,
                 min_samples: int = 30) -> RateComparison:
    """One-sided two-proportion test: is the new rate worse than the old?

    `min_effect` is the smallest drop we would care about -- 2 percentage
    points by default. If the comparison could not have detected a drop that
    size, the verdict is UNDERPOWERED regardless of what the point estimates
    did.

    The normal approximation is used, which is reasonable at these sample sizes
    and rates but degrades for very small n or rates very close to 0 or 1. The
    min_samples guard keeps it out of the worst of that.
    """
    if before_n <= 0 or after_n <= 0:
        return RateComparison(name, before_successes, before_n,
                              after_successes, after_n,
                              float("nan"), float("nan"), float("nan"),
                              float("nan"), Verdict.NO_DATA,
                              float("nan"), "one of the suites was empty")

    p1 = before_successes / before_n
    p2 = after_successes / after_n
    diff = p2 - p1
    mde = detectable_drop(p1, before_n, after_n)

    if before_n < min_samples or after_n < min_samples:
        return RateComparison(
            name, before_successes, before_n, after_successes, after_n,
            p1, p2, diff, float("nan"), Verdict.UNDERPOWERED, mde,
            "fewer than %d missions on one side; a rate from %d and %d runs "
            "cannot support a comparison" % (min_samples, before_n, after_n))

    # Pooled two-proportion z, one-sided in the "got worse" direction.
    pooled = (before_successes + after_successes) / (before_n + after_n)
    se = math.sqrt(max(pooled * (1.0 - pooled), 1e-12) *
                   (1.0 / before_n + 1.0 / after_n))
    z = diff / se if se > 0 else 0.0
    p_value = _normal_cdf(z)                    # P(observing a drop this big)

    if p_value < alpha:
        return RateComparison(
            name, before_successes, before_n, after_successes, after_n,
            p1, p2, diff, p_value, Verdict.REGRESSION, mde,
            "%.1f%% -> %.1f%% (p=%.4f); a real drop"
            % (100 * p1, 100 * p2, p_value))

    if (1.0 - p_value) < alpha:
        return RateComparison(
            name, before_successes, before_n, after_successes, after_n,
            p1, p2, diff, p_value, Verdict.IMPROVEMENT, mde,
            "%.1f%% -> %.1f%% (p=%.4f); a real rise"
            % (100 * p1, 100 * p2, p_value))

    # No significant change. But could this comparison have SEEN the effect we
    # care about? If not, silence is not evidence.
    if mde > min_effect:
        return RateComparison(
            name, before_successes, before_n, after_successes, after_n,
            p1, p2, diff, p_value, Verdict.UNDERPOWERED, mde,
            "no significant change, but this comparison could only have "
            "detected a drop of %.1f points or more, and we care about %.1f. "
            "Run more missions before calling this a pass."
            % (100 * mde, 100 * min_effect))

    return RateComparison(
        name, before_successes, before_n, after_successes, after_n,
        p1, p2, diff, p_value, Verdict.UNCHANGED, mde,
        "%.1f%% -> %.1f%% (p=%.4f); within noise, and a %.1f point drop would "
        "have been detected" % (100 * p1, 100 * p2, p_value, 100 * min_effect))


@dataclass
class BudgetComparison:
    """Comparison of a continuous metric against its previous value."""

    key: str
    budget: Budget
    before: float
    after: float
    verdict: Verdict
    detail: str = ""

    @property
    def acceptable(self) -> bool:
        return self.verdict in (Verdict.UNCHANGED, Verdict.IMPROVEMENT)


def compare_budget(key: str, before: float, after: float,
                   tolerance_fraction: float = 0.10) -> BudgetComparison:
    """Did a latency or rate get materially worse?

    A tolerance band rather than a significance test: these are percentiles of
    large sample sets, so run-to-run wobble is small and proportional. Anything
    beyond `tolerance_fraction` of the previous value is treated as real.

    A metric that got worse but is STILL inside its prototype budget is still a
    regression. Phase 7 says "no worse than pre-optimisation", not "still
    passing" -- otherwise headroom gets spent silently until one change tips it
    over and no single commit looks responsible.
    """
    budget = BUDGETS[key]
    if not (math.isfinite(before) and math.isfinite(after)):
        return BudgetComparison(key, budget, before, after, Verdict.NO_DATA,
                                "one side was not measured")

    if before == 0:
        return BudgetComparison(key, budget, before, after, Verdict.NO_DATA,
                                "previous value was zero; nothing to compare")

    change = (after - before) / abs(before)
    worse = change > 0 if budget.sense is Sense.LOWER_IS_BETTER else change < 0

    if abs(change) <= tolerance_fraction:
        return BudgetComparison(
            key, budget, before, after, Verdict.UNCHANGED,
            "%.2f -> %.2f %s (%.1f%%, inside the %.0f%% tolerance)"
            % (before, after, budget.unit, 100 * change,
               100 * tolerance_fraction))

    if worse:
        still_ok = budget.better(after, budget.prototype)
        return BudgetComparison(
            key, budget, before, after, Verdict.REGRESSION,
            "%.2f -> %.2f %s (%.1f%% worse)%s"
            % (before, after, budget.unit, 100 * abs(change),
               "; still inside the prototype budget, but the headroom was "
               "spent" if still_ok else "; now outside the prototype budget"))

    return BudgetComparison(
        key, budget, before, after, Verdict.IMPROVEMENT,
        "%.2f -> %.2f %s (%.1f%% better)"
        % (before, after, budget.unit, 100 * abs(change)))


@dataclass
class RegressionReport:
    rates: List[RateComparison]
    metrics: List[BudgetComparison]

    @property
    def acceptable(self) -> bool:
        """Phase 7 gate. Every comparison must be clearly acceptable."""
        return (all(r.acceptable for r in self.rates) and
                all(m.acceptable for m in self.metrics) and
                bool(self.rates or self.metrics))

    @property
    def regressions(self) -> List[str]:
        out = [r.name for r in self.rates if r.verdict is Verdict.REGRESSION]
        out += [m.key for m in self.metrics if m.verdict is Verdict.REGRESSION]
        return out

    @property
    def underpowered(self) -> List[str]:
        return [r.name for r in self.rates if r.verdict is Verdict.UNDERPOWERED]


def format_report(report: RegressionReport) -> str:
    lines = ["optimisation regression check (TASK.md Phase 7)"]

    for r in report.rates:
        lines.append("  %-18s %-13s %s" % (r.name, r.verdict.value, r.detail))
    for m in report.metrics:
        lines.append("  %-18s %-13s %s"
                     % (m.budget.name, m.verdict.value, m.detail))

    lines.append("")
    if report.acceptable:
        lines.append("  ACCEPTED: nothing got worse, and the suite was large "
                     "enough to have noticed.")
    else:
        lines.append("  REJECTED")
        for name in report.regressions:
            lines.append("    regression: %s" % name)
        for name in report.underpowered:
            lines.append("    underpowered: %s -- not evidence of no change"
                         % name)
    return "\n".join(lines)
