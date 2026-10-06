# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
"""The semantic vocabulary, and what each class does to the cost.

SPEC.md §5.2. Deliberately small: every class must change a navigation
decision, and one that does not does not belong here.

No ROS, no model, no numpy beyond arithmetic. This module is the contract
between whatever detector we run this week and the traversability cost
function, so it is testable on its own and does not move when the model does.

Two properties matter more than the numbers:

1. **Class ids are stable.** They travel in a `mono8` mask on
   `/perception/semantic_mask` and end up in recorded bags. Renumbering breaks
   every bag ever recorded, so ids are assigned once and only appended to.

2. **An unrecognised id is expensive, never traversable.** A model that emits a
   class we do not know about, or a corrupted mask byte, must not decay to
   "road". This is the same rule as SPEC.md §6.2 for unobserved terrain, and it
   is the one thing in this file that is a safety property rather than a
   tuning choice.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Dict, Iterable, Optional


class Tier(IntEnum):
    """SPEC.md §5.2 tiers, in increasing order of consequence."""

    TRAVERSABLE = 0
    CAUTION = 1
    HIGH_COST = 2
    LETHAL = 3
    DYNAMIC = 4
    UNKNOWN = 5


class ClassId(IntEnum):
    """Stable semantic class ids.

    Values are frozen. Append new classes with new numbers; never renumber an
    existing one, and never reuse the number of a class that is removed.

    Blocks of ten by tier are for human readability only -- nothing derives a
    tier from an id, because that coupling would silently mis-tier the first
    class that did not fit the pattern.
    """

    UNKNOWN = 0

    # Traversable
    DIRT = 10
    ROAD = 11
    GRASS = 12

    # Caution
    GRAVEL = 20
    UNEVEN_GRASS = 21
    ROUGH_GROUND = 22

    # High cost
    MUD = 30
    WATER = 31
    DEEP_VEGETATION = 32
    STEEP_SLOPE = 33

    # Lethal, static
    DITCH = 40
    CLIFF_EDGE = 41
    ROCK = 42
    TREE_TRUNK = 43
    WALL = 44

    # Dynamic
    PERSON = 50
    VEHICLE = 51
    ANIMAL = 52


@dataclass(frozen=True)
class TierPolicy:
    """What a tier does to the cost.

    `cost` feeds the `semantic` term of SPEC.md §6.1 and is already normalised
    to [0, 1]; the traversability core weights it from there.
    """

    cost: float
    lethal: bool
    #: Extra clearance beyond the footprint, metres. Only dynamic classes get
    #: one: a person may step the way we are about to drive.
    clearance_m: float = 0.0
    #: Seconds for the contribution to halve once the class stops being
    #: observed. Static terrain persists; a pedestrian who has walked out of
    #: frame must not haunt the costmap.
    half_life_s: float = float("inf")


TIER_POLICY: Dict[Tier, TierPolicy] = {
    Tier.TRAVERSABLE: TierPolicy(cost=0.00, lethal=False),
    Tier.CAUTION: TierPolicy(cost=0.35, lethal=False),
    Tier.HIGH_COST: TierPolicy(cost=0.75, lethal=False),
    Tier.LETHAL: TierPolicy(cost=1.00, lethal=True),
    Tier.DYNAMIC: TierPolicy(cost=1.00, lethal=True, clearance_m=0.60,
                             half_life_s=0.5),
    # Matches `unknown_cost` in traversability.yaml. Expensive, not lethal:
    # lethal would forbid the planner from entering anything unclassified, and
    # a vehicle that will not drive past something it cannot name cannot move.
    Tier.UNKNOWN: TierPolicy(cost=0.85, lethal=False),
}


CLASS_TIER: Dict[ClassId, Tier] = {
    ClassId.UNKNOWN: Tier.UNKNOWN,

    ClassId.DIRT: Tier.TRAVERSABLE,
    ClassId.ROAD: Tier.TRAVERSABLE,
    ClassId.GRASS: Tier.TRAVERSABLE,

    ClassId.GRAVEL: Tier.CAUTION,
    ClassId.UNEVEN_GRASS: Tier.CAUTION,
    ClassId.ROUGH_GROUND: Tier.CAUTION,

    ClassId.MUD: Tier.HIGH_COST,
    ClassId.WATER: Tier.HIGH_COST,
    ClassId.DEEP_VEGETATION: Tier.HIGH_COST,
    ClassId.STEEP_SLOPE: Tier.HIGH_COST,

    ClassId.DITCH: Tier.LETHAL,
    ClassId.CLIFF_EDGE: Tier.LETHAL,
    ClassId.ROCK: Tier.LETHAL,
    ClassId.TREE_TRUNK: Tier.LETHAL,
    ClassId.WALL: Tier.LETHAL,

    ClassId.PERSON: Tier.DYNAMIC,
    ClassId.VEHICLE: Tier.DYNAMIC,
    ClassId.ANIMAL: Tier.DYNAMIC,
}


def tier_of(class_id: int) -> Tier:
    """Tier for a class id. Anything unrecognised is UNKNOWN, never traversable.

    Covers a model emitting a class outside our vocabulary, a corrupted mask
    byte, and a negative or out-of-range value. All of them are ignorance, and
    ignorance is expensive.
    """
    try:
        return CLASS_TIER[ClassId(class_id)]
    except (ValueError, KeyError):
        return Tier.UNKNOWN


def policy_of(class_id: int) -> TierPolicy:
    return TIER_POLICY[tier_of(class_id)]


def semantic_cost(class_id: int) -> float:
    """Normalised [0, 1] cost for the SPEC.md §6.1 `semantic` term."""
    return policy_of(class_id).cost


def is_lethal(class_id: int) -> bool:
    return policy_of(class_id).lethal


def clearance_m(class_id: int) -> float:
    return policy_of(class_id).clearance_m


def decayed_cost(class_id: int, age_s: float) -> float:
    """Cost after `age_s` seconds without re-observation.

    Static terrain does not fade: a ditch seen once is still a ditch. Dynamic
    classes halve every `half_life_s`, so a pedestrian who left the frame stops
    blocking the map within about a second rather than becoming a permanent
    phantom obstacle.

    A lethal *static* class never decays below its full cost regardless of age.
    """
    policy = policy_of(class_id)
    if age_s <= 0.0:
        return policy.cost
    if policy.half_life_s == float("inf"):
        return policy.cost
    return policy.cost * (0.5 ** (age_s / policy.half_life_s))


def decayed_lethal(class_id: int, age_s: float,
                   lethal_floor: float = 0.5) -> bool:
    """Whether a class still saturates the cell after `age_s`.

    A dynamic obstacle stops being lethal once its decayed cost falls below
    `lethal_floor`; a static one never does.
    """
    policy = policy_of(class_id)
    if not policy.lethal:
        return False
    if policy.half_life_s == float("inf"):
        return True
    return decayed_cost(class_id, age_s) >= lethal_floor


def all_classes() -> Iterable[ClassId]:
    return tuple(ClassId)


def name_of(class_id: int) -> str:
    try:
        return ClassId(class_id).name
    except ValueError:
        return "UNRECOGNISED_%d" % class_id


def from_detector_label(label: str,
                        mapping: Optional[Dict[str, int]] = None) -> ClassId:
    """Map a detector's own label onto our vocabulary.

    A pretrained COCO detector calls things "person", "car", "truck". Anything
    we have no mapping for becomes UNKNOWN rather than being dropped: an
    unmapped detection is still something the model saw, and discarding it
    would turn a real object into empty space.
    """
    table = mapping if mapping is not None else DEFAULT_LABEL_MAP
    key = (label or "").strip().lower()
    return ClassId(table.get(key, int(ClassId.UNKNOWN)))


#: COCO-ish labels a pretrained YOLO emits, mapped onto SPEC.md §5.2.
#: Everything absent becomes UNKNOWN by way of from_detector_label.
DEFAULT_LABEL_MAP: Dict[str, int] = {
    "person": int(ClassId.PERSON),
    "bicycle": int(ClassId.VEHICLE),
    "car": int(ClassId.VEHICLE),
    "motorcycle": int(ClassId.VEHICLE),
    "bus": int(ClassId.VEHICLE),
    "truck": int(ClassId.VEHICLE),
    "train": int(ClassId.VEHICLE),
    "boat": int(ClassId.VEHICLE),
    "bird": int(ClassId.ANIMAL),
    "cat": int(ClassId.ANIMAL),
    "dog": int(ClassId.ANIMAL),
    "horse": int(ClassId.ANIMAL),
    "sheep": int(ClassId.ANIMAL),
    "cow": int(ClassId.ANIMAL),
    "elephant": int(ClassId.ANIMAL),
    "bear": int(ClassId.ANIMAL),
    "zebra": int(ClassId.ANIMAL),
    "giraffe": int(ClassId.ANIMAL),
}
