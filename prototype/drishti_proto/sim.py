# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
"""The simulation loop: sensing, terrain cost, planning, safety, motion.

This is the same pipeline as the real stack, with the heavy parts replaced by
the smallest thing that produces the same KIND of input:

    real                              here
    ----------------------------------------------------------------
    Gazebo + stereo camera            raycast over a height field
    elevation_mapping_cupy            observed cells with slope/step/roughness
    drishti_traversability  (C++)     the SAME logic, ported and parity-checked
    Nav2 planner + MPPI               A* over the cost grid + pure pursuit
    drishti_safety          (C++)     the SAME logic, ported and parity-checked
    drishti_perception                the REAL taxonomy module, imported

So the terrain judgement and the stop decision you watch are the shipping
logic, not a re-creation of it. tools/check_parity.py proves that against the
C++ on 8000 cases.

What is NOT real here: physics, image formation, SLAM. The vehicle knows where
it is. That makes this a demonstration of the terrain and safety reasoning, not
evidence that visual localisation works -- see README.md.
"""
from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from . import traversability as T
from .planner import astar, pure_pursuit
from .supervisor import Action, Inputs, Params, Reason, REASON_TEXT, SupervisorCore
from .world import World

# Use the REAL taxonomy from the workspace rather than a copy: class ids, tier
# costs and the "unrecognised is expensive" rule all come from the shipping
# module, so the demo cannot drift from it.
_WS = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))), "drishti-ugv", "ugv_ws", "src")
if os.path.isdir(os.path.join(_WS, "drishti_perception")):
    sys.path.insert(0, os.path.join(_WS, "drishti_perception"))
    from drishti_perception.taxonomy import (  # noqa: E402
        is_lethal, semantic_cost)
    TAXONOMY_SOURCE = "drishti_perception (real module)"
else:                                                     # pragma: no cover
    def semantic_cost(_):
        return 0.85

    def is_lethal(_):
        return False
    TAXONOMY_SOURCE = "fallback (workspace not found)"


@dataclass
class Fault:
    """A fault to inject, mirroring drishti_eval.faults."""

    at_s: float
    kind: str        # "camera_freeze" | "camera_silence" | "depth_silence" | "slam_loss"
    label: str = ""


@dataclass
class Telemetry:
    """Everything the renderer needs for one frame."""

    t: float = 0.0
    pose: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    action: Action = Action.STOP
    reason: Reason = Reason.NONE
    reason_text: str = ""
    v_limit: float = 0.0
    cmd_in: Tuple[float, float] = (0.0, 0.0)
    cmd_out: Tuple[float, float] = (0.0, 0.0)
    rgb_age: float = 0.0
    depth_age: float = 0.0
    rgb_static_for: float = 0.0
    confidence: float = 0.0
    nearest_obstacle: float = float("nan")
    path: List[Tuple[float, float]] = field(default_factory=list)
    observed_fraction: float = 0.0
    lethal_cells: int = 0
    distance_to_goal: float = 0.0
    outcome: Optional[str] = None
    active_faults: List[str] = field(default_factory=list)


class Simulation:
    """One mission."""

    SENSOR_RANGE = 4.5          # m
    SENSOR_FOV = math.radians(80.0)
    DT = 0.1                    # s per step
    GOAL_TOLERANCE = 0.45       # m

    def __init__(self, world: World, faults: Optional[List[Fault]] = None,
                 seed: int = 0, timeout_s: float = 180.0):
        self.world = world
        self.faults = list(faults or [])
        self.timeout_s = timeout_s

        self.terrain = T.TraversabilityCore(T.Weights(), T.Limits())
        self.supervisor = SupervisorCore(Params())

        self.t = 0.0
        x, y = world.start
        self.pose = (x, y, 0.0)

        # The vehicle's belief. Starts entirely unobserved, which the real cost
        # function prices at unknown_cost -- expensive, never free (SPEC 6.2).
        self.observed = [[False] * world.width for _ in range(world.height)]
        self.cost = [[T.Limits().unknown_cost] * world.width
                     for _ in range(world.height)]
        self.lethal = [[False] * world.width for _ in range(world.height)]

        self.last_rgb = 0.0
        self.last_depth = 0.0
        self.rgb_static_for = 0.0
        self.path: List[Tuple[float, float]] = []
        self.outcome: Optional[str] = None
        self.telemetry = Telemetry()
        self._last_replan = -99.0
        self._collisions = 0
        self._stop_since: Optional[float] = None
        self.plan_cost = [row[:] for row in self.cost]
        self.plan_blocked = [row[:] for row in self.lethal]

    # ------------------------------------------------------------ faults
    def active_faults(self) -> List[Fault]:
        return [f for f in self.faults if self.t >= f.at_s]

    def _fault_kinds(self):
        return {f.kind for f in self.active_faults()}

    # ------------------------------------------------------------ sensing
    def sense(self) -> None:
        """Raycast the field of view and update the belief.

        Standing in for a stereo camera plus elevation mapping. Cells outside
        the cone, or behind something taller, stay unobserved -- which is what
        makes "unknown is expensive" visible in the demo rather than
        theoretical.
        """
        kinds = self._fault_kinds()
        x, y, th = self.pose

        if "camera_silence" in kinds:
            # RGB stops; depth is a separate stream and keeps arriving. An
            # earlier version let this suppress depth too, and the demo then
            # reported DEPTH_STALE for a camera fault -- the right action for
            # the wrong reason, which is worse than a wrong action because it
            # sends you debugging the wrong sensor.
            if "depth_silence" not in kinds:
                self.last_depth = self.t
            return
        if "camera_freeze" in kinds:
            # Frames keep coming with fresh stamps and stale content. The age
            # never grows; only rgb_static_for does. This is D18/D19.
            self.last_rgb = self.t
            self.rgb_static_for += self.DT
            if "depth_silence" not in kinds:
                self.last_depth = self.t
            return

        self.last_rgb = self.t
        self.rgb_static_for = 0.0
        if "depth_silence" not in kinds:
            self.last_depth = self.t

        w = self.world
        rays = 61
        for i in range(rays):
            a = th - self.SENSOR_FOV / 2 + self.SENSOR_FOV * i / (rays - 1)
            ca, sa = math.cos(a), math.sin(a)
            blocked_height = -9.9
            steps = int(self.SENSOR_RANGE / (w.resolution * 0.5))
            for s in range(1, steps + 1):
                d = s * w.resolution * 0.5
                cx, cy = w.to_cell(x + ca * d, y + sa * d)
                if not w.in_bounds(cx, cy):
                    break
                self._observe(cx, cy)
                # Crude occlusion: something taller than the line of sight
                # hides what is behind it.
                hgt = w.height_at(cx, cy)
                if hgt > blocked_height + 0.35:
                    blocked_height = hgt
                    if hgt > 0.8:
                        break

    def _observe(self, cx: int, cy: int) -> None:
        # Terrain does not change, so a cell only needs evaluating once. Before
        # this the raycast re-evaluated every cell in the cone on every tick --
        # about 2200 cost evaluations per step, almost all of them recomputing
        # an answer already known. Caching cut a 60-step run from 16.8 s to
        # well under a second, which is the difference between a demo that
        # animates and one that stutters.
        if self.observed[cy][cx]:
            return

        w = self.world
        self.observed[cy][cx] = True

        cell = T.Cell(
            observed=True,
            slope=w.slope_at(cx, cy),
            roughness=w.roughness_at(cx, cy),
            height_variance=w.variance_at(cx, cy),
            step_height=w.step_at(cx, cy),
            semantic_cost=semantic_cost(w.class_at(cx, cy)),
            semantic_lethal=is_lethal(w.class_at(cx, cy)),
            visibility=1.0,
            confidence=0.9,
        )
        result = self.terrain.evaluate(cell)
        self.cost[cy][cx] = result.cost
        self.lethal[cy][cx] = result.lethal

    def nearest_obstacle(self) -> float:
        """Distance to the closest lethal cell inside the sensor cone.

        NaN when nothing is found, which is what the supervisor reads as
        "nothing in range" -- sound only because it rules out stale depth
        first.
        """
        if "depth_silence" in self._fault_kinds():
            return float("nan")
        x, y, th = self.pose
        w = self.world
        best = float("nan")
        reach = int(self.SENSOR_RANGE / w.resolution)
        cx0, cy0 = w.to_cell(x, y)
        for dy in range(-reach, reach + 1):
            for dx in range(-reach, reach + 1):
                cx, cy = cx0 + dx, cy0 + dy
                if not w.in_bounds(cx, cy) or not self.lethal[cy][cx]:
                    continue
                wx, wy = w.to_world(cx, cy)
                d = math.hypot(wx - x, wy - y)
                if d > self.SENSOR_RANGE:
                    continue
                bearing = abs(math.atan2(wy - y, wx - x) - th)
                bearing = min(bearing, 2 * math.pi - bearing)
                if bearing > self.SENSOR_FOV / 2:
                    continue
                if math.isnan(best) or d < best:
                    best = d
        return best

    # ----------------------------------------------------------- planning
    # Mirrors the inflation_layer in nav2.yaml. Without it the planner routes
    # straight past a lethal cell, the supervisor emergency-stops at
    # d_emergency, and the vehicle deadlocks a metre from a rock it planned to
    # shave. Nav2 carries an inflation layer for exactly this reason, and
    # leaving it out here reproduced the failure immediately.
    INSCRIBED_RADIUS = 0.40     # m, vehicle half-width plus margin: blocked
    INFLATION_RADIUS = 1.10     # m, cost decays to zero by here

    def _inflate(self) -> None:
        # Grow every lethal cell into a blocked core and a costly halo.
        w = self.world
        self.plan_cost = [row[:] for row in self.cost]
        self.plan_blocked = [row[:] for row in self.lethal]

        reach = int(self.INFLATION_RADIUS / w.resolution) + 1
        sources = [(cx, cy)
                   for cy in range(w.height) for cx in range(w.width)
                   if self.lethal[cy][cx]]

        for sx, sy in sources:
            for dy in range(-reach, reach + 1):
                for dx in range(-reach, reach + 1):
                    cx, cy = sx + dx, sy + dy
                    if not w.in_bounds(cx, cy) or self.lethal[cy][cx]:
                        continue
                    d = math.hypot(dx, dy) * w.resolution
                    if d <= self.INSCRIBED_RADIUS:
                        self.plan_blocked[cy][cx] = True
                    elif d <= self.INFLATION_RADIUS:
                        decay = 1.0 - (d - self.INSCRIBED_RADIUS) / (
                            self.INFLATION_RADIUS - self.INSCRIBED_RADIUS)
                        self.plan_cost[cy][cx] = max(
                            self.plan_cost[cy][cx], 0.85 * decay)

    def replan(self) -> None:
        w = self.world
        self._inflate()
        start = w.to_cell(self.pose[0], self.pose[1])
        goal = w.to_cell(*w.goal)

        # Never let inflation block the cell the vehicle is standing in, or it
        # can trap itself against something it has already driven past.
        if w.in_bounds(*start):
            self.plan_blocked[start[1]][start[0]] = self.lethal[start[1]][start[0]]

        cells = astar(self.plan_cost, self.plan_blocked, start, goal,
                      w.width, w.height)
        self.path = [w.to_world(cx, cy) for cx, cy in cells]
        self._last_replan = self.t

    # -------------------------------------------------------------- step
    def step(self) -> Telemetry:
        self.t += self.DT
        kinds = self._fault_kinds()

        self.sense()

        # Replan periodically, and whenever the current path has been
        # invalidated by something newly observed.
        if (self.t - self._last_replan > 0.6) or not self.path:
            self.replan()

        cmd_lin, cmd_ang = pure_pursuit(self.pose, self.path, lookahead=1.2,
                                        v_nominal=1.0)

        # Perception confidence: high when the pipeline is healthy. A frozen
        # camera does NOT lower it -- that is the whole point of D19. It is
        # caught by rgb_static_for, not by pretending the model got unsure.
        confidence = 0.0 if "camera_silence" in kinds else 0.9

        inputs = Inputs(
            now=self.t,
            last_rgb_stamp=self.last_rgb,
            last_depth_stamp=self.last_depth,
            rgb_static_for=self.rgb_static_for,
            pose_valid="slam_loss" not in kinds,
            pose_covariance_max=0.05,
            nearest_obstacle=self.nearest_obstacle(),
            perception_confidence=confidence,
            path_valid=len(self.path) > 1,
            cmd_linear_x=cmd_lin,
            cmd_angular_z=cmd_ang,
        )
        decision = self.supervisor.evaluate(inputs)

        # Only the supervisor's output ever moves the vehicle, exactly as
        # /cmd_vel is the supervisor's alone in the real stack.
        self._drive(decision.linear_x, decision.angular_z)

        x, y, _ = self.pose
        dist = math.hypot(x - self.world.goal[0], y - self.world.goal[1])
        self._classify(dist, decision)

        observed_n = sum(1 for row in self.observed for v in row if v)
        total = self.world.width * self.world.height

        self.telemetry = Telemetry(
            t=self.t, pose=self.pose,
            action=decision.action, reason=decision.reason,
            reason_text=REASON_TEXT[decision.reason],
            v_limit=decision.v_limit,
            cmd_in=(cmd_lin, cmd_ang),
            cmd_out=(decision.linear_x, decision.angular_z),
            rgb_age=decision.rgb_age, depth_age=decision.depth_age,
            rgb_static_for=decision.rgb_static_for,
            confidence=confidence,
            nearest_obstacle=inputs.nearest_obstacle,
            path=list(self.path),
            observed_fraction=observed_n / total,
            lethal_cells=sum(1 for row in self.lethal for v in row if v),
            distance_to_goal=dist,
            outcome=self.outcome,
            active_faults=[f.label or f.kind for f in self.active_faults()],
        )
        return self.telemetry

    def _drive(self, lin: float, ang: float) -> None:
        x, y, th = self.pose
        th += ang * self.DT
        nx = x + lin * math.cos(th) * self.DT
        ny = y + lin * math.sin(th) * self.DT

        # Collision and ditch-fall detection, from the TRUE world rather than
        # the belief: the vehicle can drive somewhere it wrongly believed safe,
        # and that has to be visible.
        cx, cy = self.world.to_cell(nx, ny)
        if self.world.in_bounds(cx, cy):
            if self.world.step_at(cx, cy) >= T.Limits().step_lethal:
                self._collisions += 1
            self.pose = (nx, ny, th)

    def _classify(self, dist: float, decision) -> None:
        if self.outcome is not None:
            return
        if self._collisions > 0:
            self.outcome = "collision"
        elif dist <= self.GOAL_TOLERANCE:
            self.outcome = "success"
        elif self.t > self.timeout_s:
            self.outcome = "timeout"
        elif decision.action == Action.STOP and self.t > 4.0:
            # A halt only counts as a safe abort once it has persisted; the
            # supervisor legitimately stops for a tick while replanning.
            if self._stop_since is None:
                self._stop_since = self.t
            elif self.t - self._stop_since > 6.0:
                self.outcome = "safe_abort"
        else:
            self._stop_since = None

    @property
    def finished(self) -> bool:
        return self.outcome is not None
