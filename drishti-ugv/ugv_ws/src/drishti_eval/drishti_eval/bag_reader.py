# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
"""Read pose tracks out of a rosbag2 recording.

This is the ONLY module in drishti_eval that imports ROS. Everything else is
plain numpy so the metrics can be developed and tested without an install
(STATUS.md D17). Keep it that way: if a metric needs a ROS type, convert here
and pass numpy onward.

EVALUATION.md 7.1: ground truth is recorded alongside every run, and a run
without its ground-truth track is not evaluable. This module is where that
rule is enforced in code -- `load_run` refuses rather than returning a
half-populated result.

!! UNVERIFIED !! Never executed; rosbag2_py is not installed anywhere on the
project. The message-to-numpy conversion below is the part most likely to need
a fix at first contact with a real bag.
"""
from __future__ import annotations

import os
from typing import Dict, List, Tuple

import numpy as np

from .trajectory import Trajectory

# SPEC.md 4.2 / drishti_sim/config/bridge.yaml.
DEFAULT_ESTIMATE_TOPIC = "/rtabmap/localization_pose"
DEFAULT_GROUND_TRUTH_TOPIC = "/ground_truth/pose"


def _require_rosbag2():
    try:
        import rosbag2_py  # noqa: F401
        from rclpy.serialization import deserialize_message  # noqa: F401
        from rosidl_runtime_py.utilities import get_message  # noqa: F401
    except ImportError as exc:  # pragma: no cover - depends on the environment
        raise RuntimeError(
            "rosbag2_py / rclpy are not available. Bag reading needs a sourced "
            "ROS 2 environment; the metrics themselves do not. Run the metric "
            "tests with plain python instead."
        ) from exc


def read_topics(bag_path: str, topics: List[str]) -> Dict[str, List[Tuple[int, object]]]:
    """Return {topic: [(timestamp_ns, message), ...]} for the requested topics."""
    _require_rosbag2()
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    if not os.path.exists(bag_path):
        raise FileNotFoundError("no such bag: %s" % bag_path)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id=""),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                    output_serialization_format="cdr"),
    )

    type_of = {t.name: t.type for t in reader.get_all_topics_and_types()}
    missing = [t for t in topics if t not in type_of]
    if missing:
        raise KeyError(
            "bag %s does not contain %s. Present: %s"
            % (bag_path, missing, sorted(type_of)))

    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))

    out: Dict[str, List[Tuple[int, object]]] = {t: [] for t in topics}
    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        msg = deserialize_message(data, get_message(type_of[topic]))
        out[topic].append((stamp_ns, msg))
    return out


def _header_seconds(msg, fallback_ns: int) -> float:
    """Prefer the message's own stamp over the bag's receive time.

    SPEC.md 3.2 rule 3 requires real sensor timestamps. The bag receive time
    includes transport delay, and using it would fold that delay into every
    localisation error.
    """
    header = getattr(msg, "header", None)
    if header is not None:
        stamp = header.stamp
        secs = stamp.sec + stamp.nanosec * 1e-9
        if secs > 0.0:
            return secs
    return fallback_ns * 1e-9


def _to_trajectory(entries) -> Trajectory:
    stamps, positions, quats = [], [], []
    for stamp_ns, msg in entries:
        # nav_msgs/Odometry and geometry_msgs/PoseWithCovarianceStamped both
        # nest the pose one level down; PoseStamped does not.
        pose = getattr(msg, "pose", None)
        if pose is not None and hasattr(pose, "pose"):
            pose = pose.pose
        if pose is None:
            continue
        p, q = pose.position, pose.orientation
        stamps.append(_header_seconds(msg, stamp_ns))
        positions.append([p.x, p.y, p.z])
        quats.append([q.x, q.y, q.z, q.w])

    if not stamps:
        return Trajectory(np.zeros(0), np.zeros((0, 3)), np.zeros((0, 4)))
    return Trajectory(np.array(stamps), np.array(positions), np.array(quats))


def load_run(bag_path: str,
             estimate_topic: str = DEFAULT_ESTIMATE_TOPIC,
             ground_truth_topic: str = DEFAULT_GROUND_TRUTH_TOPIC
             ) -> Tuple[Trajectory, Trajectory]:
    """Load (estimate, ground_truth) from one recorded run."""
    data = read_topics(bag_path, [estimate_topic, ground_truth_topic])
    estimate = _to_trajectory(data[estimate_topic])
    truth = _to_trajectory(data[ground_truth_topic])

    if len(truth) == 0:
        raise ValueError(
            "no ground-truth poses on %s. EVALUATION.md 7.1: a run without its "
            "ground-truth track is not evaluable and does not count."
            % ground_truth_topic)
    if len(estimate) == 0:
        raise ValueError("no estimated poses on %s" % estimate_topic)
    return estimate, truth
