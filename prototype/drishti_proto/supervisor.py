# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
"""Python port of drishti_safety::SupervisorCore.

The prototype exists to show what the real system does, so it must run the real
decision logic rather than something that resembles it. The shipping supervisor
is C++ (SPEC.md §9, 388 tests) and cannot be imported from Python without ROS,
so this is a line-by-line port.

A port is a liability unless it is proven equal. tools/parity_oracle.cpp emits
the C++ decision for a large grid of inputs and tools/check_parity.py replays
the identical grid through this module and demands the same answer every time.
If they ever diverge, the parity check fails and this file is wrong — the
demo cannot quietly drift away from the thing it is demonstrating.

Mirrors ugv_ws/src/drishti_safety/{include,src} exactly, including the
evaluation order and the reason numbering.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import IntEnum

NEVER = -1.0e18
INF = float("inf")
NAN = float("nan")


class Action(IntEnum):
    PASS = 0
    SLOW = 1
    STOP = 2


class Reason(IntEnum):
    NONE = 0
    LOCALIZATION_LOST = 1
    DEPTH_STALE = 2
    CAMERA_STALE = 3
    OBSTACLE_EMERGENCY = 4
    LOW_CONFIDENCE = 5
    PATH_INVALID = 6
    COMMAND_INVALID = 7
    CAMERA_FROZEN = 8


REASON_TEXT = {
    Reason.NONE: "none",
    Reason.LOCALIZATION_LOST: "localisation lost",
    Reason.DEPTH_STALE: "depth stale",
    Reason.CAMERA_STALE: "camera stale",
    Reason.OBSTACLE_EMERGENCY: "obstacle inside emergency distance",
    Reason.LOW_CONFIDENCE: "perception confidence below floor",
    Reason.PATH_INVALID: "no valid path",
    Reason.COMMAND_INVALID: "planner command invalid",
    Reason.CAMERA_FROZEN: "camera frozen; frames unchanging",
}


@dataclass
class Params:
    t_camera_stale: float = 0.30
    t_depth_stale: float = 0.30
    d_emergency: float = 0.80
    c_critical: float = 0.40
    v_max: float = 1.20
    v_slow: float = 0.35
    cov_max: float = 0.50
    watchdog_period: float = 0.02
    t_frame_static: float = 2.0

    def valid(self):
        for name in ("t_camera_stale", "t_depth_stale", "d_emergency",
                     "v_max", "v_slow", "cov_max", "watchdog_period",
                     "t_frame_static"):
            v = getattr(self, name)
            if not (math.isfinite(v) and v > 0.0):
                return False, "%s must be finite and > 0" % name
        if not (math.isfinite(self.c_critical) and 0.0 <= self.c_critical <= 1.0):
            return False, "c_critical must be within [0, 1]"
        if self.v_slow > self.v_max:
            return False, "v_slow must not exceed v_max"
        if self.t_frame_static <= self.t_camera_stale:
            return False, "t_frame_static must exceed t_camera_stale"
        return True, ""


@dataclass
class Inputs:
    now: float = 0.0
    last_rgb_stamp: float = NEVER
    last_depth_stamp: float = NEVER
    rgb_static_for: float = 0.0
    pose_valid: bool = False
    pose_covariance_max: float = INF
    nearest_obstacle: float = NAN
    perception_confidence: float = 0.0
    path_valid: bool = False
    cmd_linear_x: float = 0.0
    cmd_angular_z: float = 0.0


@dataclass
class Decision:
    action: Action = Action.STOP
    reason: Reason = Reason.NONE
    v_limit: float = 0.0
    linear_x: float = 0.0
    angular_z: float = 0.0
    stop: bool = True
    rgb_age: float = INF
    depth_age: float = INF
    rgb_static_for: float = 0.0


def stamp_age(now: float, stamp: float, future_tolerance: float) -> float:
    if not math.isfinite(now) or not math.isfinite(stamp):
        return INF
    if stamp <= NEVER / 2.0:
        return INF
    age = now - stamp
    if age < -abs(future_tolerance):
        return INF
    return 0.0 if age < 0.0 else age


class SupervisorCore:
    def __init__(self, params: Params):
        self.params = params

    def evaluate(self, inp: Inputs) -> Decision:
        p = self.params
        d = Decision()
        d.action = Action.STOP
        d.reason = Reason.NONE
        d.v_limit = 0.0
        d.linear_x = 0.0
        d.angular_z = 0.0
        d.stop = True

        d.rgb_age = stamp_age(inp.now, inp.last_rgb_stamp, p.watchdog_period)
        d.depth_age = stamp_age(inp.now, inp.last_depth_stamp, p.watchdog_period)
        d.rgb_static_for = (inp.rgb_static_for
                            if math.isfinite(inp.rgb_static_for) else 0.0)

        # 1. localisation
        if (not inp.pose_valid or
                not math.isfinite(inp.pose_covariance_max) or
                inp.pose_covariance_max > p.cov_max):
            d.reason = Reason.LOCALIZATION_LOST
            return d

        # 2. depth freshness
        if d.depth_age > p.t_depth_stale:
            d.reason = Reason.DEPTH_STALE
            return d

        # 3. camera freshness
        if d.rgb_age > p.t_camera_stale:
            d.reason = Reason.CAMERA_STALE
            return d

        # 4. frozen camera
        if math.isfinite(inp.rgb_static_for) and inp.rgb_static_for >= p.t_frame_static:
            d.reason = Reason.CAMERA_FROZEN
            return d

        # 5. emergency geometry
        if math.isfinite(inp.nearest_obstacle) and inp.nearest_obstacle < p.d_emergency:
            d.reason = Reason.OBSTACLE_EMERGENCY
            return d

        # 6. planner has somewhere to go
        if not inp.path_valid:
            d.reason = Reason.PATH_INVALID
            return d

        # 7. the command itself
        if not math.isfinite(inp.cmd_linear_x) or not math.isfinite(inp.cmd_angular_z):
            d.reason = Reason.COMMAND_INVALID
            return d

        # 8. confidence: the only non-stop branch
        slow = (not math.isfinite(inp.perception_confidence) or
                inp.perception_confidence < p.c_critical)
        limit = p.v_slow if slow else p.v_max

        lin = inp.cmd_linear_x
        ang = inp.cmd_angular_z
        speed = abs(lin)
        if speed > limit:
            k = limit / speed
            lin *= k
            ang *= k

        d.action = Action.SLOW if slow else Action.PASS
        d.reason = Reason.LOW_CONFIDENCE if slow else Reason.NONE
        d.v_limit = limit
        d.linear_x = lin
        d.angular_z = ang
        d.stop = False
        return d
