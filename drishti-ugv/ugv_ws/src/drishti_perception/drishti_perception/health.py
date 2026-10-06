# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
"""Perception liveness and quality, as consumed by the safety supervisor.

SPEC.md §5.3 and §9.2. This computes what goes on `/perception/health`. The
supervisor reads it to decide whether perception can be trusted at all; it
never looks inside the model.

No ROS. Pure arithmetic on timestamps and confidences, so it is testable.

---------------------------------------------------------------------------
THE DECISION THIS MODULE EXISTS TO GET RIGHT

What is the confidence of a frame in which the detector found nothing?

Both obvious answers are wrong:

  1.0 -- "nothing to be unsure about". Then a detector that has silently died
         and returns an empty list every frame reports perfect confidence, and
         the supervisor runs at full speed into whatever it stopped seeing.

  0.0 -- "we saw nothing". Then open safe terrain, which is the common case
         and the one the vehicle should cross briskly, permanently pins the
         supervisor into SLOW. A safety mechanism that fires constantly gets
         turned off.

The distinction that actually matters is not how many objects were found but
whether the pipeline RAN. So:

  pipeline did not run, or errored     -> 0.0, and the supervisor stops
  a segmenter produced a mask          -> its coverage-weighted confidence
  detections exist, no mask            -> mean detection confidence
  ran fine, found nothing, no mask     -> `nominal_confidence`

Finding nothing is not evidence of danger. A pipeline that did not run is.
---------------------------------------------------------------------------
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence

#: Age reported when a stream has never produced a frame. Chosen over
#: `inf` so it survives serialisation into a float32 message field.
NEVER_SEEN_AGE = 1.0e9


class FrameChangeTracker:
    """How long the camera content has been unchanged.

    D18/D19. A camera that goes silent is caught by `t_camera_stale`. A camera
    that FREEZES -- republishing one image with a fresh timestamp -- is not:
    the age never grows, so it looks perfectly healthy while the view of the
    world becomes arbitrarily old. Liveness and freshness are different
    questions, and only this one answers the second.

    The caller supplies a cheap signature per frame (a downsampled hash, a
    checksum, anything deterministic). This class only tracks how long it has
    stayed the same.

    A real camera never produces two identical frames -- sensor noise alone
    guarantees that -- so in the field this is close to a pure freeze detector.
    In simulation a noiseless camera on a motionless vehicle CAN legitimately
    repeat frames, which is why the threshold in drishti.yaml is seconds rather
    than milliseconds, and why a spurious trip is acceptable: it produces a
    STOP, which is the safe direction to be wrong in.
    """

    def __init__(self) -> None:
        self._signature = None
        self._since: Optional[float] = None
        self._last_t: Optional[float] = None

    def update(self, signature, t: float) -> float:
        """Record a frame, and return how long content has been unchanged."""
        if not math.isfinite(t):
            return self.static_for(self._last_t if self._last_t else 0.0)

        if signature is None:
            # No signature computed for this frame. Do not claim the content
            # changed -- that would let a broken signature path mask a freeze --
            # but do not advance the clock on evidence we do not have either.
            return self.static_for(t)

        if self._signature is None or signature != self._signature:
            self._signature = signature
            self._since = t
        self._last_t = t
        return self.static_for(t)

    def static_for(self, now: float) -> float:
        if self._since is None or not math.isfinite(now):
            return 0.0
        return max(now - self._since, 0.0)

    def reset(self) -> None:
        self._signature = None
        self._since = None
        self._last_t = None


@dataclass
class FrameStats:
    """What the perception node knows at the moment it publishes health."""

    now: float                                  #: s, on the node's clock
    last_rgb_stamp: Optional[float] = None      #: s, sensor stamp
    last_depth_stamp: Optional[float] = None    #: s, sensor stamp

    pipeline_ok: bool = False                   #: inference ran without error
    detection_confidences: Sequence[float] = field(default_factory=tuple)
    #: Mean confidence over the semantic mask, if a segmenter ran.
    segmentation_confidence: Optional[float] = None
    #: Fraction of pixels the segmenter actually classified, [0, 1].
    segmentation_coverage: float = 0.0

    #: Wall time from frame arrival to outputs ready.
    latency_ms: float = float("nan")

    #: Seconds the RGB content has been unchanged, from FrameChangeTracker.
    #: Defaults to 0 so a pipeline that does not compute it cannot stop the
    #: vehicle by omission.
    rgb_static_for: float = 0.0


@dataclass
class Health:
    """The `/perception/health` payload, as plain numbers."""

    rgb_age: float
    depth_age: float
    rgb_ok: bool
    depth_ok: bool
    mean_confidence: float
    latency_ms: float
    detection_count: int
    rgb_static_for: float = 0.0
    #: Why the confidence took the value it did. Logged, not published: it is
    #: for the person reading a bag six weeks later.
    confidence_basis: str = ""


def stamp_age(now: float, stamp: Optional[float],
              future_tolerance: float = 0.05) -> float:
    """Age of a timestamped frame in seconds.

    Mirrors `drishti_safety::stamp_age`. Nothing received, non-finite values,
    or a stamp meaningfully in the future all return NEVER_SEEN_AGE: clocks
    that disagree are no evidence at all (SPEC.md §3.2 rule 4).
    """
    if stamp is None:
        return NEVER_SEEN_AGE
    if not math.isfinite(now) or not math.isfinite(stamp):
        return NEVER_SEEN_AGE
    age = now - stamp
    if age < -abs(future_tolerance):
        return NEVER_SEEN_AGE
    return max(age, 0.0)


def _clean(values: Sequence[float]) -> List[float]:
    """Finite values clamped to [0, 1]; anything else is discarded."""
    return [min(max(float(v), 0.0), 1.0)
            for v in values if isinstance(v, (int, float)) and math.isfinite(v)]


def frame_confidence(stats: FrameStats,
                     nominal_confidence: float = 0.80,
                     min_coverage: float = 0.20) -> tuple:
    """Confidence for this frame, and the reason for it.

    See the module docstring. Returns (confidence, basis).
    """
    if not stats.pipeline_ok:
        return 0.0, "pipeline did not run"

    seg = stats.segmentation_confidence
    if seg is not None and math.isfinite(seg):
        coverage = stats.segmentation_coverage
        if not math.isfinite(coverage):
            coverage = 0.0
        coverage = min(max(coverage, 0.0), 1.0)
        if coverage < min_coverage:
            # A mask covering a sliver of the image says almost nothing about
            # the frame. Scale the confidence by how much was actually
            # classified rather than trusting a confident opinion about 3% of
            # the pixels.
            return (min(max(seg, 0.0), 1.0) * coverage,
                    "segmentation coverage %.2f below %.2f" % (coverage, min_coverage))
        return min(max(seg, 0.0), 1.0), "segmentation"

    detections = _clean(stats.detection_confidences)
    if detections:
        # The mean, not the max: one certain detection does not make the rest
        # of the frame understood.
        return sum(detections) / len(detections), "detections"

    return (min(max(nominal_confidence, 0.0), 1.0),
            "pipeline ran, nothing detected")


def compute(stats: FrameStats,
            t_camera_stale: float = 0.30,
            t_depth_stale: float = 0.30,
            nominal_confidence: float = 0.80,
            min_coverage: float = 0.20) -> Health:
    """Build the health report for one frame.

    Staleness thresholds default to the same values as the supervisor's
    `t_camera_stale` and `t_depth_stale` so the two agree about what "fresh"
    means. They are passed in rather than imported because both sides read them
    from the shared params file at run time.
    """
    rgb_age = stamp_age(stats.now, stats.last_rgb_stamp)
    depth_age = stamp_age(stats.now, stats.last_depth_stamp)
    confidence, basis = frame_confidence(stats, nominal_confidence, min_coverage)

    return Health(
        rgb_age=rgb_age,
        depth_age=depth_age,
        rgb_ok=rgb_age <= t_camera_stale,
        depth_ok=depth_age <= t_depth_stale,
        mean_confidence=confidence,
        latency_ms=stats.latency_ms,
        detection_count=len(_clean(stats.detection_confidences)),
        rgb_static_for=(stats.rgb_static_for
                        if math.isfinite(stats.rgb_static_for) else 0.0),
        confidence_basis=basis,
    )
