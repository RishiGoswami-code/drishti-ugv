#!/usr/bin/env python3
# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
"""TEST STUB ONLY -- not shipped code, not part of the phase gate.

bringup.launch.py does not include drishti_perception (SPEC.md Phase 4 is not
wired into it yet -- see the launch file's own comment). CONTRIBUTING.md rule 2 is
"loss of input is a stop condition", and the safety supervisor core does not
special-case a topic that has *never* been published on at all versus one
that has gone stale -- so without this stub, /cmd_vel never carries a nonzero
command and Nav2's plan never has consequence: the supervisor holds STOP
because /perception/health simply never arrived.

This node publishes a constantly-healthy PerceptionHealth so Phase 1 bring-up
(Nav2 reaching a goal) can be exercised before Phase 4 perception exists.

Delete this file, or stop launching it, the day drishti_perception is wired
into bringup.launch.py for real. Do not let it survive into a Phase 4 result.

!! UNVERIFIED !! Never executed -- written from the message definition in
drishti_msgs/msg/PerceptionHealth.msg, not from a running system.

    ros2 run drishti_bringup phase1_stub_perception_health   # if installed
    # or, without installing it as an entry point:
    python3 tools/phase1_stub_perception_health.py
"""
import rclpy
from rclpy.node import Node

from drishti_msgs.msg import PerceptionHealth


class StubPerceptionHealth(Node):

    def __init__(self) -> None:
        super().__init__("phase1_stub_perception_health")
        self.pub = self.create_publisher(PerceptionHealth, "/perception/health", 10)
        self.create_timer(0.1, self.tick)
        self.get_logger().warn(
            "STUB PerceptionHealth publisher active -- Phase 1 bring-up only. "
            "This is not the real perception stack. Do not read a Phase 5/6 "
            "result recorded while this node is running.")

    def tick(self) -> None:
        now = self.get_clock().now().to_msg()
        msg = PerceptionHealth()
        msg.header.stamp = now
        msg.last_rgb_stamp = now
        msg.last_depth_stamp = now
        msg.rgb_age = 0.0
        msg.depth_age = 0.0
        msg.rgb_ok = True
        msg.depth_ok = True
        msg.rgb_static_for = 0.0
        msg.mean_confidence = 1.0
        msg.latency_ms = 0.0
        msg.detection_count = 0
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = StubPerceptionHealth()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
