# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
"""Thin wrapper around a pretrained YOLO detector.

SPEC.md §5.4 stage V1: a lightweight pretrained detector, small vocabulary, no
training. Do not start by collecting thousands of images -- establish the loop
first and let failure analysis justify each upgrade.

The wrapper exists so the rest of the package never imports ultralytics. All
policy about what a class *means* lives in taxonomy.py, which is tested; this
file only turns model output into `Detection` objects.

LICENCE: Ultralytics is AGPL-3.0 with a separate commercial licence, and
REFERENCES.md §3 flags it as licence-reviewed-before-use. It is imported
lazily and is not a hard dependency of this package, so nothing here forces
an AGPL obligation onto the rest of the stack. Settle the licence question
before shipping anything that bundles it.

!! UNVERIFIED !! Never executed; ultralytics is not installed anywhere on the
project. The model output shape below is the most likely thing to need fixing
on first contact.
"""
from __future__ import annotations

import time
from typing import List, Optional, Sequence, Tuple

import numpy as np

from .obstacle import Detection
from .taxonomy import ClassId, from_detector_label


class Detector:
    """Loads a YOLO model and returns Detections in our vocabulary."""

    def __init__(self, weights: str = "yolo11n.pt",
                 confidence: float = 0.25,
                 device: str = "cuda:0",
                 image_size: int = 640,
                 label_map: Optional[dict] = None) -> None:
        self.weights = weights
        self.confidence = confidence
        self.device = device
        self.image_size = image_size
        self.label_map = label_map
        self._model = None
        self._names: dict = {}

    def load(self) -> None:
        """Import and load the model. Kept out of __init__ so a node can
        construct a Detector, report what it intends to load, and fail loudly
        at a predictable moment rather than during the first callback."""
        try:
            from ultralytics import YOLO
        except ImportError as exc:  # pragma: no cover - environment dependent
            raise RuntimeError(
                "ultralytics is not installed. Perception cannot run. The rest "
                "of drishti_perception (taxonomy, health, obstacle) does not "
                "need it and is tested without it."
            ) from exc

        self._model = YOLO(self.weights)
        # names maps the model's own class index to its label string.
        self._names = dict(getattr(self._model, "names", {}) or {})

    @property
    def loaded(self) -> bool:
        return self._model is not None

    def infer(self, image: np.ndarray) -> Tuple[List[Detection], float]:
        """Run the detector on one BGR/RGB frame.

        Returns (detections, latency_ms). Raises if the model is not loaded --
        a silent empty result would be indistinguishable from "nothing there",
        which is exactly the failure health.py refuses to paper over.
        """
        if self._model is None:
            raise RuntimeError("Detector.load() was never called")

        started = time.perf_counter()
        results = self._model.predict(
            source=image, conf=self.confidence, imgsz=self.image_size,
            device=self.device, verbose=False)
        latency_ms = (time.perf_counter() - started) * 1000.0

        return self._to_detections(results), latency_ms

    def _to_detections(self, results: Sequence) -> List[Detection]:
        out: List[Detection] = []
        for result in results or ():
            boxes = getattr(result, "boxes", None)
            if boxes is None:
                continue
            for box in boxes:
                try:
                    x0, y0, x1, y1 = (float(v) for v in box.xyxy[0].tolist())
                    conf = float(box.conf[0])
                    model_class = int(box.cls[0])
                except (AttributeError, IndexError, TypeError, ValueError):
                    # A malformed box is dropped, but never silently: the node
                    # counts these and the count reaches the health report.
                    continue

                label = self._names.get(model_class, "")
                class_id = from_detector_label(label, self.label_map)
                out.append(Detection(
                    class_id=int(class_id), confidence=conf,
                    x0=int(x0), y0=int(y0), x1=int(x1), y1=int(y1)))
        return out

    def describe(self) -> str:
        return ("YOLO weights=%s conf=%.2f imgsz=%d device=%s"
                % (self.weights, self.confidence, self.image_size, self.device))


def unmapped_labels(detector: "Detector") -> List[str]:
    """Model labels that fall through to UNKNOWN.

    Worth logging once at startup: a detector whose entire vocabulary lands on
    UNKNOWN is technically safe -- everything becomes expensive -- but it is
    also useless, and the symptom (a uniformly costly map) is easy to
    misdiagnose as a terrain problem.
    """
    if not detector.loaded:
        return []
    return sorted(
        label for label in detector._names.values()
        if from_detector_label(label, detector.label_map) is ClassId.UNKNOWN)
