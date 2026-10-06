# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
"""A small 2-D world with terrain the vehicle has to reason about.

Not a physics engine. It is a height field plus a semantic label per cell,
which is exactly the two things the real elevation pipeline hands to the
traversability cost function. That is the point: the demo feeds the REAL cost
function the same kind of input Gazebo plus elevation_mapping_cupy would, so
what you watch is the actual decision logic rather than an illustration of it.

Worlds mirror the Gazebo ones in drishti_sim so the demo and the real suite are
talking about the same scenarios.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

# Class ids come from the real taxonomy (drishti_perception). See sim.py for
# how it is imported; world.py keeps the numbers it needs locally so it can be
# used standalone.
CLASS_UNKNOWN = 0
CLASS_DIRT = 10
CLASS_GRASS = 12
CLASS_GRAVEL = 20
CLASS_MUD = 30
CLASS_WATER = 31
CLASS_DITCH = 40
CLASS_ROCK = 42
CLASS_TREE = 43


@dataclass
class World:
    """Height field plus semantics on a regular grid."""

    name: str
    width: int                       # cells
    height: int                      # cells
    resolution: float = 0.25         # m per cell
    heights: List[List[float]] = field(default_factory=list)
    classes: List[List[int]] = field(default_factory=list)
    start: Tuple[float, float] = (1.0, 5.0)
    goal: Tuple[float, float] = (18.0, 5.0)
    description: str = ""

    # ---------------------------------------------------------------- grid
    def in_bounds(self, cx: int, cy: int) -> bool:
        return 0 <= cx < self.width and 0 <= cy < self.height

    def height_at(self, cx: int, cy: int) -> float:
        if not self.in_bounds(cx, cy):
            return 0.0
        return self.heights[cy][cx]

    def class_at(self, cx: int, cy: int) -> int:
        if not self.in_bounds(cx, cy):
            return CLASS_UNKNOWN
        return self.classes[cy][cx]

    def to_cell(self, x: float, y: float) -> Tuple[int, int]:
        return int(x / self.resolution), int(y / self.resolution)

    def to_world(self, cx: int, cy: int) -> Tuple[float, float]:
        return ((cx + 0.5) * self.resolution, (cy + 0.5) * self.resolution)

    @property
    def extent(self) -> Tuple[float, float]:
        return self.width * self.resolution, self.height * self.resolution

    # ------------------------------------------------- terrain properties
    def slope_at(self, cx: int, cy: int) -> float:
        """Surface slope in radians, from the local height gradient."""
        h = self.height_at
        dzdx = (h(cx + 1, cy) - h(cx - 1, cy)) / (2 * self.resolution)
        dzdy = (h(cx, cy + 1) - h(cx, cy - 1)) / (2 * self.resolution)
        return math.atan(math.hypot(dzdx, dzdy))

    def step_at(self, cx: int, cy: int) -> float:
        """Largest height jump to a 4-neighbour.

        This is what makes a ditch lethal on geometry alone: the drop to the
        cell beside it exceeds step_lethal, and no semantic model is involved.
        """
        centre = self.height_at(cx, cy)
        worst = 0.0
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            if self.in_bounds(cx + dx, cy + dy):
                worst = max(worst, abs(self.height_at(cx + dx, cy + dy) - centre))
        return worst

    def variance_at(self, cx: int, cy: int) -> float:
        """Height variance of the 3x3 neighbourhood."""
        vals = [self.height_at(cx + dx, cy + dy)
                for dx in (-1, 0, 1) for dy in (-1, 0, 1)
                if self.in_bounds(cx + dx, cy + dy)]
        if len(vals) < 2:
            return 0.0
        mean = sum(vals) / len(vals)
        return sum((v - mean) ** 2 for v in vals) / len(vals)

    def roughness_at(self, cx: int, cy: int) -> float:
        """Residual of the 3x3 neighbourhood about its mean height."""
        return math.sqrt(self.variance_at(cx, cy))


def _blank(width: int, height: int, fill_class: int = CLASS_DIRT):
    heights = [[0.0 for _ in range(width)] for _ in range(height)]
    classes = [[fill_class for _ in range(width)] for _ in range(height)]
    return heights, classes


def _disc(world: World, x: float, y: float, radius: float,
          height: Optional[float] = None, cls: Optional[int] = None):
    """Stamp a circular feature in world coordinates."""
    r_cells = int(radius / world.resolution) + 1
    cx0, cy0 = world.to_cell(x, y)
    for dy in range(-r_cells, r_cells + 1):
        for dx in range(-r_cells, r_cells + 1):
            cx, cy = cx0 + dx, cy0 + dy
            if not world.in_bounds(cx, cy):
                continue
            wx, wy = world.to_world(cx, cy)
            if math.hypot(wx - x, wy - y) <= radius:
                if height is not None:
                    world.heights[cy][cx] = height
                if cls is not None:
                    world.classes[cy][cx] = cls


def _rect(world: World, x0: float, y0: float, x1: float, y1: float,
          height: Optional[float] = None, cls: Optional[int] = None):
    a = world.to_cell(x0, y0)
    b = world.to_cell(x1, y1)
    for cy in range(min(a[1], b[1]), max(a[1], b[1]) + 1):
        for cx in range(min(a[0], b[0]), max(a[0], b[0]) + 1):
            if not world.in_bounds(cx, cy):
                continue
            if height is not None:
                world.heights[cy][cx] = height
            if cls is not None:
                world.classes[cy][cx] = cls


# ---------------------------------------------------------------- worlds
def easy(seed: int = 0) -> World:
    """Flat dirt, sparse rocks. Mirrors drishti_sim/worlds/easy.sdf."""
    w, h = 88, 44
    heights, classes = _blank(w, h)
    world = World("Easy", w, h, 0.25, heights, classes,
                  start=(1.5, 5.5), goal=(19.5, 5.5),
                  description="Flat dirt, sparse rocks. The baseline: if this "
                              "does not work, nothing else matters.")
    rng = random.Random(seed)
    for _ in range(6):
        x = rng.uniform(4.0, 17.0)
        y = rng.uniform(1.0, 10.0)
        _disc(world, x, y, rng.uniform(0.3, 0.5), height=0.45, cls=CLASS_ROCK)
    # Gentle undulation, well inside every threshold.
    for cy in range(h):
        for cx in range(w):
            world.heights[cy][cx] += 0.012 * math.sin(cx * 0.35) * math.cos(cy * 0.3)
    return world


def medium(seed: int = 0) -> World:
    """Grass, humps, a ramp and tree trunks. Mirrors medium.sdf."""
    w, h = 88, 44
    heights, classes = _blank(w, h, CLASS_GRASS)
    world = World("Medium", w, h, 0.25, heights, classes,
                  start=(1.5, 5.5), goal=(19.5, 5.5),
                  description="Slopes, roughness and tree trunks. Terrain the "
                              "planner must reason about, not just avoid.")
    rng = random.Random(seed + 1)

    # A ramp up and back down across the direct line: ~9 degrees, climbable.
    for cy in range(h):
        for cx in range(w):
            wx, _ = world.to_world(cx, cy)
            if 7.0 <= wx <= 10.0:
                world.heights[cy][cx] += (wx - 7.0) * 0.16
            elif 10.0 < wx <= 13.0:
                world.heights[cy][cx] += (13.0 - wx) * 0.16

    for _ in range(14):                      # roughness
        _disc(world, rng.uniform(2.0, 18.0), rng.uniform(1.0, 10.0),
              rng.uniform(0.2, 0.35), height=rng.uniform(0.03, 0.07),
              cls=CLASS_GRAVEL)
    for _ in range(5):                       # trunks: lethal on geometry
        _disc(world, rng.uniform(4.0, 18.0), rng.uniform(1.0, 10.0),
              0.28, height=1.6, cls=CLASS_TREE)
    return world


def hard(seed: int = 0) -> World:
    """A ditch across the route, with a flank detour. Mirrors hard.sdf / T07.

    THE scenario. A ditch is a negative obstacle: nothing sticks up, so an
    occupancy grid sees free space and drives in. It has to be refused on the
    step height alone, with no semantic model in the loop.
    """
    w, h = 88, 52
    heights, classes = _blank(w, h)
    world = World("Hard (ditch)", w, h, 0.25, heights, classes,
                  start=(1.5, 6.5), goal=(19.5, 6.5),
                  description="A 1.5 m ditch across the direct line, and a "
                              "detour to the north. Geometry alone must "
                              "refuse the ditch (scenario T07).")
    rng = random.Random(seed + 2)

    # The ditch: a trench from x=9.0 to x=10.5, spanning the southern half.
    # The gap at the top is the detour, so avoiding it is a success rather
    # than a planner failure.
    _rect(world, 9.0, 0.0, 10.5, 9.5, height=-0.55, cls=CLASS_DITCH)

    for _ in range(5):
        _disc(world, rng.uniform(3.0, 7.0), rng.uniform(1.0, 11.0),
              rng.uniform(0.3, 0.45), height=0.5, cls=CLASS_ROCK)

    # Mud beside the detour: passable, expensive, and invisible to geometry.
    _disc(world, 10.0, 11.5, 1.1, cls=CLASS_MUD)
    return world


WORLDS = {"easy": easy, "medium": medium, "hard": hard}
