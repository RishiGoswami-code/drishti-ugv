# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
"""Perception node: images in, detections + health + obstacle distance out.

    /camera/rgb/image_raw          -->  /perception/detections
    /camera/depth/image_rect_raw   -->  /perception/health
                                   -->  /perception/nearest_obstacle
                                   -->  /perception/semantic_mask  (V2)

Holds no policy. What a class means is taxonomy.py; what a frame's confidence
means is health.py; how a distance is measured is obstacle.py. All three are
tested without ROS (198 checks). This file is plumbing.

THE ONE THING THIS NODE MUST NOT DO is stay silent when inference fails.
`/perception/health` is published on a timer regardless of whether a frame
arrived or the model succeeded, because the supervisor treats a missing health
report as a dead camera and stops. A node that only publishes on success would
turn a crashed detector into a silent, confident-looking stall.

!! UNVERIFIED !! Never executed -- no machine on the project has ROS 2, and
ultralytics is not installed (STATUS.md D17).
"""
from __future__ import annotations

import threading
import time
from typing import List, Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from vision_msgs.msg import (BoundingBox2D, Detection2D, Detection2DArray,
                             ObjectHypothesisWithPose)

from drishti_msgs.msg import PerceptionHealth

from . import health as H
from .detector import Detector, unmapped_labels
from .obstacle import Detection, nearest_obstacle
from .taxonomy import name_of


class PerceptionNode(Node):

    def __init__(self) -> None:
        super().__init__("perception")

        self.declare_parameter("weights", "yolo11n.pt")
        self.declare_parameter("confidence", 0.25)
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("image_size", 640)
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("nominal_confidence", 0.80)
        self.declare_parameter("min_segmentation_coverage", 0.20)
        self.declare_parameter("obstacle_percentile", 10.0)
        self.declare_parameter("obstacle_min_confidence", 0.25)
        self.declare_parameter("t_camera_stale", 0.30)
        self.declare_parameter("t_depth_stale", 0.30)
        self.declare_parameter("frame_id", "camera_left_optical")
        self.declare_parameter("frame_signature_stride", 16)

        self.detector = Detector(
            weights=self.get_parameter("weights").value,
            confidence=float(self.get_parameter("confidence").value),
            device=self.get_parameter("device").value,
            image_size=int(self.get_parameter("image_size").value))

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frame_change = H.FrameChangeTracker()
        self.rgb_static_for = 0.0

        self.last_rgb_stamp: Optional[float] = None
        self.last_depth_stamp: Optional[float] = None
        self.depth: Optional[np.ndarray] = None
        self.detections: List[Detection] = []
        self.pipeline_ok = False
        self.latency_ms = float("nan")
        self.malformed_boxes = 0

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT       # sensor data
        reliable = QoSProfile(depth=10)
        reliable.reliability = ReliabilityPolicy.RELIABLE     # health must land

        self.create_subscription(Image, "/camera/rgb/image_raw", self.on_rgb, qos)
        self.create_subscription(
            Image, "/camera/depth/image_rect_raw", self.on_depth, qos)

        self.pub_detections = self.create_publisher(
            Detection2DArray, "/perception/detections", reliable)
        self.pub_health = self.create_publisher(
            PerceptionHealth, "/perception/health", reliable)
        self.pub_obstacle = self.create_publisher(
            Float32, "/perception/nearest_obstacle", reliable)

        # Health goes out on a timer, NOT on frame arrival. See the module
        # docstring: silence must remain a detectable failure.
        rate = float(self.get_parameter("publish_rate").value)
        self.create_timer(1.0 / max(rate, 1.0), self.publish_health)

        self.load_model()

    def load_model(self) -> None:
        try:
            self.detector.load()
            self.get_logger().info("perception up: %s" % self.detector.describe())
            unmapped = unmapped_labels(self.detector)
            if unmapped:
                self.get_logger().info(
                    "%d model labels fall through to UNKNOWN (expensive but "
                    "unnamed), including: %s"
                    % (len(unmapped), ", ".join(unmapped[:8])))
        except RuntimeError as exc:
            # Do not die. A node that exits stops publishing health, and the
            # supervisor would report a stale camera rather than a dead model.
            # Staying alive and honestly reporting pipeline_ok=False produces a
            # STOP with the right reason attached.
            self.get_logger().fatal("%s" % exc)

    # ------------------------------------------------------------ callbacks
    def on_rgb(self, msg: Image) -> None:
        stamp = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:                      # noqa: BLE001
            self.get_logger().warn("could not convert RGB frame: %s" % exc)
            with self.lock:
                self.last_rgb_stamp = stamp
                self.pipeline_ok = False
            return

        # Freeze detection runs BEFORE the model, and regardless of whether it
        # loaded. A frozen camera is a sensor fault, not a perception fault,
        # and it must still be caught when inference is unavailable (D19).
        static_for = self.frame_change.update(self.frame_signature(frame), stamp)

        if not self.detector.loaded:
            with self.lock:
                self.last_rgb_stamp = stamp
                self.rgb_static_for = static_for
                self.pipeline_ok = False
            return

        try:
            detections, latency_ms = self.detector.infer(frame)
            ok = True
        except Exception as exc:                      # noqa: BLE001
            self.get_logger().warn("inference failed: %s" % exc)
            detections, latency_ms, ok = [], float("nan"), False

        with self.lock:
            self.last_rgb_stamp = stamp
            self.rgb_static_for = static_for
            self.detections = detections
            self.latency_ms = latency_ms
            self.pipeline_ok = ok

        if ok:
            self.publish_detections(msg.header, detections)
            self.publish_obstacle()

    def on_depth(self, msg: Image) -> None:
        stamp = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:                      # noqa: BLE001
            self.get_logger().warn("could not convert depth frame: %s" % exc)
            return

        depth = np.asarray(depth, dtype=float)
        # 16UC1 depth is millimetres; 32FC1 is metres. Guessing wrong scales
        # every distance by 1000 and would make d_emergency meaningless.
        if msg.encoding in ("16UC1", "mono16"):
            depth = depth / 1000.0

        with self.lock:
            self.last_depth_stamp = stamp
            self.depth = depth

    def frame_signature(self, frame: np.ndarray):
        """A cheap, deterministic fingerprint of the frame's content.

        Subsampled rather than hashed over every pixel: this runs on every
        frame inside the perception budget, and a stride-16 sample of a
        640x480 image is 1200 values -- enough that two genuinely different
        scenes cannot collide in practice, cheap enough to ignore.

        Returns None on failure rather than a constant. A constant would look
        like frozen content and stop the vehicle on a bug in this function;
        None leaves the tracker's existing verdict untouched.
        """
        try:
            stride = max(1, int(self.get_parameter("frame_signature_stride").value))
            sample = np.asarray(frame)[::stride, ::stride]
            return hash(sample.tobytes())
        except Exception:                             # noqa: BLE001
            return None

    # ------------------------------------------------------------ publishing
    def publish_detections(self, header, detections: List[Detection]) -> None:
        out = Detection2DArray()
        out.header = header
        for det in detections:
            d = Detection2D()
            d.header = header
            box = BoundingBox2D()
            box.center.position.x = float((det.x0 + det.x1) / 2.0)
            box.center.position.y = float((det.y0 + det.y1) / 2.0)
            box.size_x = float(abs(det.x1 - det.x0))
            box.size_y = float(abs(det.y1 - det.y0))
            d.bbox = box

            hypothesis = ObjectHypothesisWithPose()
            # Our stable id, not the model's index: the model can be swapped
            # without changing what a bag means.
            hypothesis.hypothesis.class_id = str(int(det.class_id))
            hypothesis.hypothesis.score = float(det.confidence)
            d.results.append(hypothesis)
            out.detections.append(d)
        self.pub_detections.publish(out)

    def publish_obstacle(self) -> None:
        with self.lock:
            depth = self.depth
            detections = list(self.detections)
            percentile = float(self.get_parameter("obstacle_percentile").value)
            min_conf = float(self.get_parameter("obstacle_min_confidence").value)

        if depth is None:
            # No depth yet. NaN means "not measured"; the supervisor stops on
            # stale depth independently, so this cannot be read as "clear".
            distance = float("nan")
        else:
            distance = nearest_obstacle(
                depth, detections, min_confidence=min_conf, percentile=percentile)

        msg = Float32()
        msg.data = float(distance)
        self.pub_obstacle.publish(msg)

    def publish_health(self) -> None:
        with self.lock:
            stats = H.FrameStats(
                now=self.get_clock().now().nanoseconds * 1e-9,
                last_rgb_stamp=self.last_rgb_stamp,
                last_depth_stamp=self.last_depth_stamp,
                pipeline_ok=self.pipeline_ok,
                detection_confidences=tuple(d.confidence for d in self.detections),
                rgb_static_for=self.rgb_static_for,
                latency_ms=self.latency_ms)

        report = H.compute(
            stats,
            t_camera_stale=float(self.get_parameter("t_camera_stale").value),
            t_depth_stale=float(self.get_parameter("t_depth_stale").value),
            nominal_confidence=float(
                self.get_parameter("nominal_confidence").value),
            min_coverage=float(
                self.get_parameter("min_segmentation_coverage").value))

        msg = PerceptionHealth()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter("frame_id").value
        if self.last_rgb_stamp is not None:
            msg.last_rgb_stamp = rclpy.time.Time(
                seconds=self.last_rgb_stamp).to_msg()
        if self.last_depth_stamp is not None:
            msg.last_depth_stamp = rclpy.time.Time(
                seconds=self.last_depth_stamp).to_msg()
        msg.rgb_age = float(report.rgb_age)
        msg.depth_age = float(report.depth_age)
        msg.rgb_ok = bool(report.rgb_ok)
        msg.depth_ok = bool(report.depth_ok)
        msg.mean_confidence = float(report.mean_confidence)
        msg.latency_ms = float(
            0.0 if report.latency_ms != report.latency_ms else report.latency_ms)
        msg.detection_count = int(report.detection_count)
        msg.rgb_static_for = float(report.rgb_static_for)
        self.pub_health.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
