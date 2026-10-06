# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
"""Fault injector for the T16-T19 failure scenarios.

Sits between the simulator and the stack. Subscribes to a source topic,
republishes it onto the name the stack actually consumes, and applies the
scheduled fault in between:

    /camera/rgb/image_raw  -->  [injector]  -->  /camera/rgb/image_raw
         (from the bridge)                        (as the stack sees it)

Remapping is done in the launch file: the bridge is pointed at a `raw_` prefix
and the injector republishes onto the real name, so nothing downstream knows it
is there.

All schedule logic is in faults.py, which is tested (111 checks with
latency.py). This file only applies it.

TWO THINGS IT MUST GET RIGHT

1. It logs the exact injection time on /fault_events. That timestamp is t0 for
   the stop-latency measurement (EVALUATION.md 2.1), so an injector that
   suppressed a topic without recording precisely when would make the whole
   Phase 5 acceptance criterion unmeasurable.

2. FREEZE republishes the held message with a FRESH stamp. That is the point of
   the frozen-camera case: the stream stays live and starts lying, and a stack
   that only checks liveness sails past it. Restamping is the fault, not a bug.

!! UNVERIFIED !! Never executed -- no machine on the project has ROS 2.
"""
from __future__ import annotations

from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message

from std_msgs.msg import String

from .faults import FaultKind, FaultSchedule, scenario


class FaultInjectorNode(Node):

    def __init__(self) -> None:
        super().__init__("fault_injector")

        self.declare_parameter("scenario", "T16_camera_dropout")
        self.declare_parameter("source_prefix", "raw")
        self.declare_parameter("republish_rate", 30.0)

        name = self.get_parameter("scenario").value
        try:
            self.schedule: FaultSchedule = scenario(name)
        except KeyError as exc:
            self.get_logger().fatal(str(exc))
            raise

        self.prefix = self.get_parameter("source_prefix").value
        self.started = self.get_clock().now()
        self.held: Dict[str, object] = {}
        self.announced: Dict[str, bool] = {}

        self.events = self.create_publisher(
            String, "/fault_events", QoSProfile(depth=50))

        self.subs = {}
        self.pubs = {}
        for topic in self.schedule.topics():
            self._bridge_topic(topic)

        rate = float(self.get_parameter("republish_rate").value)
        self.create_timer(1.0 / max(rate, 1.0), self.on_tick)

        self.get_logger().warn(
            "FAULT INJECTOR ACTIVE: scenario %s, %d fault(s) on %s. "
            "This run is a failure test; do not read its mission outcome as a "
            "navigation result." % (name, len(self.schedule.faults),
                                    ", ".join(self.schedule.topics())))
        for t, label in self.schedule.injection_times():
            self.get_logger().info("  t+%.1f s  %s" % (t, label))

    def _bridge_topic(self, topic: str) -> None:
        """Subscribe to the shadowed source and republish onto `topic`.

        The message type is resolved lazily from the first publisher seen, so
        the injector does not need to know the type of everything it can break.
        """
        source = "/%s%s" % (self.prefix, topic)
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        types = self.get_publishers_info_by_topic(source)
        if not types:
            self.get_logger().warn(
                "no publisher on %s yet; the injector cannot bridge %s until "
                "one appears. Check the launch remapping." % (source, topic))
            return

        type_name = types[0].topic_type
        msg_type = get_message(type_name)
        self.pubs[topic] = self.create_publisher(msg_type, topic, qos)
        self.subs[topic] = self.create_subscription(
            msg_type, source,
            lambda msg, tp=topic: self.on_message(tp, msg), qos)
        self.get_logger().info("bridging %s -> %s (%s)"
                               % (source, topic, type_name))

    def elapsed(self) -> float:
        return (self.get_clock().now() - self.started).nanoseconds * 1e-9

    def announce(self, topic: str, kind: FaultKind, t: float) -> None:
        """Record the exact injection instant. This is t0 for stop latency."""
        key = "%s:%s" % (topic, kind.value)
        if self.announced.get(key):
            return
        self.announced[key] = True
        msg = String()
        msg.data = "%.6f %s %s" % (t, kind.value, topic)
        self.events.publish(msg)
        self.get_logger().warn("INJECTED t+%.3f s  %s on %s"
                               % (t, kind.value, topic))

    def on_message(self, topic: str, msg) -> None:
        t = self.elapsed()
        kind = self.schedule.kind_for(topic, t)

        if kind is None:
            self.held[topic] = msg
            if topic in self.pubs:
                self.pubs[topic].publish(msg)
            return

        self.announce(topic, kind, t)

        if kind is FaultKind.SILENCE:
            return                                  # drop it entirely

        if kind is FaultKind.FREEZE:
            return                                  # the timer republishes

        if kind is FaultKind.STALE_STAMP:
            held = self.held.get(topic)
            if held is not None and hasattr(held, "header") and hasattr(msg, "header"):
                # Fresh payload, frozen clock: age grows while messages keep
                # arriving.
                msg.header.stamp = held.header.stamp
            if topic in self.pubs:
                self.pubs[topic].publish(msg)
            return

        if kind is FaultKind.EMPTY:
            if topic in self.pubs:
                self.pubs[topic].publish(self._emptied(msg))
            return

        if kind is FaultKind.NAN:
            if topic in self.pubs:
                self.pubs[topic].publish(self._nanned(msg))
            return

    def on_tick(self) -> None:
        """Republish held messages for FREEZE faults, with a fresh stamp."""
        t = self.elapsed()
        for topic, pub in self.pubs.items():
            if self.schedule.kind_for(topic, t) is not FaultKind.FREEZE:
                continue
            held = self.held.get(topic)
            if held is None:
                continue
            if hasattr(held, "header"):
                # Deliberate: a frozen camera looks perfectly alive. This is
                # the fault, not an oversight.
                held.header.stamp = self.get_clock().now().to_msg()
            pub.publish(held)

    @staticmethod
    def _emptied(msg):
        """Structurally valid, semantically useless. T19's empty /plan."""
        if hasattr(msg, "poses"):
            msg.poses = []
        elif hasattr(msg, "detections"):
            msg.detections = []
        elif hasattr(msg, "data"):
            try:
                msg.data = type(msg.data)()
            except TypeError:
                pass
        return msg

    @staticmethod
    def _nanned(msg):
        """Non-finite values, to exercise the COMMAND_INVALID branch."""
        nan = float("nan")
        if hasattr(msg, "linear") and hasattr(msg, "angular"):
            msg.linear.x = nan
            msg.angular.z = nan
        elif hasattr(msg, "twist"):
            msg.twist.linear.x = nan
            msg.twist.angular.z = nan
        elif hasattr(msg, "data"):
            try:
                msg.data = nan
            except (TypeError, AssertionError):
                pass
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = FaultInjectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
