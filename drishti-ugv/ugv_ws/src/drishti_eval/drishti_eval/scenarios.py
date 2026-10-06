# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
"""Seeded, repeatable mission generation.

TASK.md Phase 6 wants a scenario runner that is headless, seeded and
repeatable, plus the domain randomisation in SPEC.md §10.3. This module is the
generator half: it turns a seed into a fully specified mission.

EVALUATION.md §7.2 is blunt about why the seed matters -- "without the seed and
the parameter set, a result is an anecdote". So the seed is not a convenience
for debugging; it is what makes a reported number checkable by someone else.

No ROS. Pure Python `random`, deliberately not numpy: numpy's global generator
state is easy to disturb from elsewhere in a process, and a mission that
regenerates differently on a different machine would break the one property
this module exists to provide.
"""
from __future__ import annotations

import math
import random
from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

#: EVALUATION.md §6 worlds, and the scenarios each one supports.
WORLDS: Dict[str, Tuple[str, ...]] = {
    "easy.sdf": ("T01", "T02"),
    "medium.sdf": ("T04", "T05", "T06"),
    "hard.sdf": ("T03", "T07"),
}

#: Spawn height per world. hard.sdf drives on a raised platform; spawning at
#: the default drops the vehicle into the ditch at t=0, which looks like a
#: catastrophic failure and is only a launch argument.
SPAWN_Z: Dict[str, float] = {
    "easy.sdf": 0.15,
    "medium.sdf": 0.15,
    "hard.sdf": 0.60,
}


@dataclass(frozen=True)
class Randomisation:
    """SPEC.md §10.3 domain randomisation, as sampled for one mission."""

    sun_azimuth_deg: float
    sun_elevation_deg: float
    ambient: float
    camera_noise_stddev: float
    depth_dropout_fraction: float
    ground_friction: float


@dataclass(frozen=True)
class Mission:
    """One fully specified, reproducible run."""

    seed: int
    world: str
    scenario: str
    start_xy: Tuple[float, float]
    start_yaw: float
    spawn_z: float
    goal_xy: Tuple[float, float]
    goal_tolerance_m: float
    mission_timeout_s: float
    randomisation: Randomisation
    #: Named fault schedule from faults.py, or None for a clean run.
    fault_scenario: Optional[str] = None

    @property
    def straight_line_distance_m(self) -> float:
        dx = self.goal_xy[0] - self.start_xy[0]
        dy = self.goal_xy[1] - self.start_xy[1]
        return math.hypot(dx, dy)

    def as_dict(self) -> dict:
        out = asdict(self)
        out["straight_line_distance_m"] = self.straight_line_distance_m
        return out

    def launch_arguments(self) -> Dict[str, str]:
        """Arguments for `ros2 launch drishti_bringup bringup.launch.py`."""
        return {
            "world": self.world,
            "x": "%.3f" % self.start_xy[0],
            "y": "%.3f" % self.start_xy[1],
            "z": "%.3f" % self.spawn_z,
            "yaw": "%.4f" % self.start_yaw,
            "headless": "true",
        }


def generate(seed: int,
             world: Optional[str] = None,
             scenario: Optional[str] = None,
             fault_scenario: Optional[str] = None,
             mission_timeout_s: float = 300.0,
             goal_tolerance_m: float = 0.25) -> Mission:
    """Build one mission from a seed.

    The same seed always produces the same mission, on any machine and in any
    order. That is the whole contract: a suite result is only checkable if
    someone else can regenerate run #417 exactly.
    """
    rng = random.Random(seed)

    if world is None:
        world = rng.choice(sorted(WORLDS))
    if world not in WORLDS:
        raise KeyError("unknown world %r; known: %s"
                       % (world, ", ".join(sorted(WORLDS))))
    if scenario is None:
        scenario = rng.choice(WORLDS[world])

    # Start near the origin, facing roughly along +x. The worlds are laid out
    # with their features along that axis.
    start = (rng.uniform(-1.0, 1.0), rng.uniform(-1.5, 1.5))
    start_yaw = rng.uniform(-0.35, 0.35)

    # Goal far enough to be a real traverse. On hard.sdf the ditch sits across
    # the direct line at x in [11.0, 12.6], so a goal beyond it forces the
    # detour that T07 is actually testing.
    goal_x = rng.uniform(8.0, 12.0) if world != "hard.sdf" else rng.uniform(16.0, 22.0)
    goal = (goal_x, rng.uniform(-2.0, 2.0))

    randomisation = Randomisation(
        sun_azimuth_deg=rng.uniform(0.0, 360.0),
        # Never below the horizon: night is the Adversarial world's job, and
        # letting it leak in here would make Easy and Medium results
        # incomparable between runs.
        sun_elevation_deg=rng.uniform(25.0, 80.0),
        ambient=rng.uniform(0.35, 0.65),
        camera_noise_stddev=rng.uniform(0.002, 0.012),
        depth_dropout_fraction=rng.uniform(0.0, 0.05),
        ground_friction=rng.uniform(0.6, 1.0),
    )

    return Mission(
        seed=seed,
        world=world,
        scenario=scenario,
        start_xy=start,
        start_yaw=start_yaw,
        spawn_z=SPAWN_Z[world],
        goal_xy=goal,
        goal_tolerance_m=goal_tolerance_m,
        mission_timeout_s=mission_timeout_s,
        randomisation=randomisation,
        fault_scenario=fault_scenario,
    )


def suite(count: int, base_seed: int = 0, **kw) -> List[Mission]:
    """`count` missions with consecutive seeds.

    Seeds are consecutive rather than random so a suite is described by two
    numbers and anyone can regenerate any single run from it.
    """
    if count < 0:
        raise ValueError("count must be >= 0, got %d" % count)
    return [generate(base_seed + i, **kw) for i in range(count)]


def coverage(missions: Sequence[Mission]) -> Dict[str, int]:
    """How many missions hit each scenario id.

    TASK.md Phase 6 asks for the whole T01-T20 catalogue to be automated. A
    suite that ran 500 missions and never touched T07 has not tested the ditch,
    however good its headline rate looks.
    """
    counts: Dict[str, int] = {}
    for m in missions:
        counts[m.scenario] = counts.get(m.scenario, 0) + 1
    return counts


def missing_scenarios(missions: Sequence[Mission],
                      required: Sequence[str]) -> List[str]:
    seen = set(coverage(missions))
    return sorted(set(required) - seen)
