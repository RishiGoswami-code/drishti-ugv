# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
"""Distance to the nearest hazard, from detections plus a depth image.

Publishes `/perception/nearest_obstacle` (SPEC.md §4.2). The safety supervisor
compares it against `d_emergency` and stops.

No ROS. numpy only, so it is testable.

---------------------------------------------------------------------------
WHY NOT JUST TAKE THE MINIMUM DEPTH IN THE BOX

Because one bad pixel would stop the vehicle. Simulated and real depth images
both carry speckle: a handful of pixels reading 0.05 m inside an otherwise
clean 6 m box. Taking the raw minimum turns each of those into an emergency
stop, the vehicle stutters constantly, and the first thing anyone does is raise
d_emergency until the problem goes away -- which disables the emergency stop
for real obstacles too.

So the distance for a detection is a low PERCENTILE of the valid depths inside
its box. That still reacts to the nearest real surface, because a genuine
obstacle occupies many pixels, while ignoring isolated speckle.

The percentile is a parameter, not a constant, because the right value depends
on the depth sensor and belongs in an experiment (EVALUATION.md §5).
---------------------------------------------------------------------------
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence

import numpy as np

from .taxonomy import ClassId, is_lethal, tier_of, Tier


@dataclass(frozen=True)
class Detection:
    """One detection, in pixel coordinates."""

    class_id: int
    confidence: float
    #: Inclusive-exclusive pixel box: x0 <= x < x1, y0 <= y < y1.
    x0: int
    y0: int
    x1: int
    y1: int


#: Tiers that count as a hazard for the emergency-stop distance. Terrain cost
#: is the traversability layer's job; this is only about things to not hit.
HAZARD_TIERS = (Tier.LETHAL, Tier.DYNAMIC)


def is_hazard(class_id: int) -> bool:
    return tier_of(class_id) in HAZARD_TIERS


def detection_distance(depth: np.ndarray, det: Detection,
                       percentile: float = 10.0,
                       min_valid_pixels: int = 12,
                       min_range_m: float = 0.10,
                       max_range_m: float = 40.0) -> float:
    """Distance to one detection, or NaN when it cannot be measured.

    Depth pixels that are zero, negative, non-finite, or outside
    [min_range_m, max_range_m] are discarded -- all of those are the sensor
    saying "no reading here", and treating a 0.0 as "touching the bumper"
    would be the single most dangerous misreading available.

    Returns NaN rather than a guess when too few pixels survive. NaN means "not
    measured"; the caller decides what that implies, and the supervisor already
    knows that an unmeasured distance is not a safe one.
    """
    if depth.ndim != 2:
        raise ValueError("depth must be a 2-D array, got shape %r" % (depth.shape,))

    h, w = depth.shape
    x0 = max(0, min(int(det.x0), w))
    x1 = max(0, min(int(det.x1), w))
    y0 = max(0, min(int(det.y0), h))
    y1 = max(0, min(int(det.y1), h))
    if x1 <= x0 or y1 <= y0:
        return float("nan")

    patch = np.asarray(depth[y0:y1, x0:x1], dtype=float)
    valid = patch[np.isfinite(patch) &
                  (patch >= min_range_m) &
                  (patch <= max_range_m)]
    if valid.size < min_valid_pixels:
        return float("nan")

    return float(np.percentile(valid, percentile))


def nearest_obstacle(depth: np.ndarray,
                     detections: Iterable[Detection],
                     min_confidence: float = 0.25,
                     percentile: float = 10.0,
                     min_valid_pixels: int = 12,
                     min_range_m: float = 0.10,
                     max_range_m: float = 40.0) -> float:
    """Distance to the closest hazardous detection, or NaN if there is none.

    NaN is the honest answer for "nothing hazardous found in range", and it is
    what `drishti_safety` expects: its step 4 reads a non-finite distance as
    absence rather than as a fault. That is only sound because the supervisor
    has already ruled out stale depth at step 2 -- the ordering is load-bearing
    across the two packages, not just within one.

    Detections below `min_confidence` are ignored here. That is not a safety
    relaxation: a low-confidence detection still raises the semantic cost
    through the traversability layer, and it still drags `mean_confidence` down
    so the supervisor slows. What it must not do is trigger a hard emergency
    stop on a maybe.
    """
    best = float("nan")
    for det in detections:
        if not is_hazard(det.class_id):
            continue
        conf = det.confidence
        if not (isinstance(conf, (int, float)) and math.isfinite(conf)):
            continue
        if conf < min_confidence:
            continue

        distance = detection_distance(
            depth, det, percentile=percentile,
            min_valid_pixels=min_valid_pixels,
            min_range_m=min_range_m, max_range_m=max_range_m)
        if math.isfinite(distance):
            if math.isnan(best) or distance < best:
                best = distance
    return best


def lethal_detections(detections: Iterable[Detection],
                      min_confidence: float = 0.25) -> Sequence[Detection]:
    """Detections that saturate a cell in the traversability layer."""
    return tuple(d for d in detections
                 if is_lethal(d.class_id)
                 and isinstance(d.confidence, (int, float))
                 and math.isfinite(d.confidence)
                 and d.confidence >= min_confidence)


def class_of(label: Optional[str], fallback: int = int(ClassId.UNKNOWN)) -> int:
    """Small convenience so callers need not import taxonomy for one lookup."""
    from .taxonomy import from_detector_label
    if label is None:
        return fallback
    return int(from_detector_label(label))
