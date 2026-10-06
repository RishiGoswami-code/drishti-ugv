# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
"""Python port of drishti_traversability::TraversabilityCore.

SPEC.md §6.1. Same reasoning as supervisor.py: the demo must run the real cost
function, the shipping one is C++ (1217 tests), and a port is only trustworthy
if it is proven equal. tools/check_parity.py does that.

Mirrors ugv_ws/src/drishti_traversability/{include,src} exactly.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

NAN = float("nan")

FREE_SPACE = 0
MAX_NON_LETHAL = 252
INSCRIBED = 253
LETHAL = 254
NO_INFORMATION = 255


@dataclass
class Weights:
    slope: float = 1.0
    roughness: float = 1.0
    height_variance: float = 0.8
    obstacle: float = 1.5
    semantic: float = 1.2
    uncertainty: float = 1.0

    def sum(self) -> float:
        return (self.slope + self.roughness + self.height_variance +
                self.obstacle + self.semantic + self.uncertainty)


@dataclass
class Limits:
    slope_max: float = 0.35
    slope_lethal: float = 0.52
    roughness_max: float = 0.08
    height_variance_max: float = 0.010
    step_max: float = 0.12
    step_lethal: float = 0.25
    unknown_cost: float = 0.85


@dataclass
class Cell:
    observed: bool = False
    slope: float = NAN
    roughness: float = NAN
    height_variance: float = NAN
    step_height: float = NAN
    semantic_cost: float = 0.0
    semantic_lethal: bool = False
    visibility: float = 1.0
    confidence: float = 1.0


@dataclass
class CellCost:
    cost: float = 1.0
    lethal: bool = False
    unknown: bool = True
    slope_term: float = 0.0
    roughness_term: float = 0.0
    height_variance_term: float = 0.0
    obstacle_term: float = 0.0
    semantic_term: float = 0.0
    uncertainty_term: float = 1.0
    lethal_reason: str = ""


def normalise(value: float, limit: float, fallback: float) -> float:
    if not math.isfinite(value) or not math.isfinite(limit) or limit <= 0.0:
        return fallback
    return min(max(abs(value) / limit, 0.0), 1.0)


class TraversabilityCore:
    def __init__(self, weights: Weights, limits: Limits):
        self.weights = weights
        self.limits = limits

    def evaluate(self, cell: Cell) -> CellCost:
        w, l = self.weights, self.limits
        out = CellCost()

        # 1. lethal geometry and lethal classes saturate
        if math.isfinite(cell.slope) and cell.slope >= l.slope_lethal:
            return CellCost(1.0, True, False, lethal_reason="slope above slope_lethal")
        if math.isfinite(cell.step_height) and cell.step_height >= l.step_lethal:
            return CellCost(1.0, True, False, lethal_reason="step above step_lethal")
        if cell.semantic_lethal:
            return CellCost(1.0, True, False, lethal_reason="lethal semantic class")

        # 2. no usable observation
        geometry_usable = (math.isfinite(cell.slope) and
                           math.isfinite(cell.roughness) and
                           math.isfinite(cell.height_variance) and
                           math.isfinite(cell.step_height))
        if not cell.observed or not geometry_usable:
            out.cost = l.unknown_cost
            out.lethal = False
            out.unknown = True
            out.uncertainty_term = 1.0
            return out

        # 3. the weighted sum
        out.unknown = False
        out.slope_term = normalise(cell.slope, l.slope_max, 1.0)
        out.roughness_term = normalise(cell.roughness, l.roughness_max, 1.0)
        out.height_variance_term = normalise(
            cell.height_variance, l.height_variance_max, 1.0)
        out.obstacle_term = normalise(cell.step_height, l.step_max, 1.0)
        out.semantic_term = (min(max(cell.semantic_cost, 0.0), 1.0)
                             if math.isfinite(cell.semantic_cost) else 1.0)

        vis = min(max(cell.visibility, 0.0), 1.0) if math.isfinite(cell.visibility) else 0.0
        conf = min(max(cell.confidence, 0.0), 1.0) if math.isfinite(cell.confidence) else 0.0
        out.uncertainty_term = 1.0 - min(vis, conf)

        weighted = (w.slope * out.slope_term +
                    w.roughness * out.roughness_term +
                    w.height_variance * out.height_variance_term +
                    w.obstacle * out.obstacle_term +
                    w.semantic * out.semantic_term +
                    w.uncertainty * out.uncertainty_term)
        total = w.sum()
        out.cost = min(max(weighted / total, 0.0), 1.0) if total > 0.0 else 1.0
        return out

    @staticmethod
    def to_costmap(c: CellCost) -> int:
        if c.lethal:
            return LETHAL
        cost = min(max(c.cost, 0.0), 1.0) if math.isfinite(c.cost) else 1.0
        # C++ uses std::lround: half away from zero. Python's round() is
        # banker's rounding and would disagree on exact .5 values, which the
        # parity check would (correctly) flag.
        scaled = math.floor(cost * MAX_NON_LETHAL + 0.5)
        return int(min(max(scaled, 0), MAX_NON_LETHAL))
