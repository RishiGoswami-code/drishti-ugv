# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
"""Fault injection for the Failure scenarios, T16-T19.

EVALUATION.md calls this the "Failure world", but it is not scenery. Every one
of T16-T19 is a fault in the software path, not a rock in the ground:

    T16  camera dropout    stop publishing /camera/rgb/image_raw
    T17  depth dropout     stop publishing /camera/depth/image_rect_raw
    T18  SLAM loss         stop publishing /rtabmap/localization_pose
    T19  no valid path     publish an empty /plan

So the harness sits between the simulator and the stack and suppresses or
corrupts specific topics on a schedule. The world underneath can be any of the
others -- the fault is what makes the scenario.

This module holds the schedule logic only: given a schedule and a time, what
should be suppressed, and what should be corrupted? No ROS, so it is testable.

---------------------------------------------------------------------------
A FROZEN CAMERA IS NOT A SILENT ONE

T16 is "camera dropout", and there are two very different failures hiding
under that name:

  silence   no messages arrive at all. Easy to detect: the age of the last
            frame grows without bound, and the supervisor stops.

  freeze    messages keep arriving at the normal rate, with the same image
            and a FRESH timestamp. Age never grows. A stack that only checks
            liveness sees a perfectly healthy camera and drives on using an
            image of the world as it was ten seconds ago.

The second is the dangerous one and the harder test, so FaultKind has both. The
supervisor as specified catches silence via t_camera_stale; whether it catches
a freeze depends on something noticing the content is not changing, which is
NOT currently in SPEC.md §9. That gap is real and is recorded rather than
quietly designed around.
---------------------------------------------------------------------------
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Sequence, Tuple


class FaultKind(str, Enum):
    """What is done to a topic while the fault is active."""

    #: Drop every message. The stream goes silent.
    SILENCE = "silence"
    #: Keep republishing the last message with a fresh timestamp. The stream
    #: looks alive and is lying.
    FREEZE = "freeze"
    #: Keep the payload but stop advancing the timestamp, so age grows while
    #: messages still arrive.
    STALE_STAMP = "stale_stamp"
    #: Replace the payload with something structurally valid but useless --
    #: an empty path, a NaN command.
    EMPTY = "empty"
    #: Publish non-finite values, to exercise the COMMAND_INVALID branch.
    NAN = "nan"


@dataclass(frozen=True)
class Fault:
    """One scheduled fault."""

    #: Seconds after the run starts.
    start_s: float
    #: Seconds. None means "for the rest of the run".
    duration_s: Optional[float]
    topic: str
    kind: FaultKind
    #: Scenario id from EVALUATION.md §6, for the record.
    scenario: str = ""

    def active_at(self, t: float) -> bool:
        if t < self.start_s:
            return False
        if self.duration_s is None:
            return True
        return t < self.start_s + self.duration_s

    @property
    def end_s(self) -> Optional[float]:
        return None if self.duration_s is None else self.start_s + self.duration_s


@dataclass
class FaultSchedule:
    """The faults for one run, and what they imply at any instant."""

    faults: Sequence[Fault] = field(default_factory=tuple)

    def __post_init__(self) -> None:
        for f in self.faults:
            if f.start_s < 0:
                raise ValueError("fault start %.3f is negative" % f.start_s)
            if f.duration_s is not None and f.duration_s <= 0:
                raise ValueError(
                    "fault on %s has non-positive duration %r; use None for "
                    "'until the end of the run'" % (f.topic, f.duration_s))
            if not f.topic:
                raise ValueError("a fault must name a topic")

    def active(self, t: float) -> List[Fault]:
        return [f for f in self.faults if f.active_at(t)]

    def kind_for(self, topic: str, t: float) -> Optional[FaultKind]:
        """What is being done to `topic` right now, if anything.

        When two faults overlap on one topic the more severe wins, by the
        order in SEVERITY. Overlapping faults are a scenario-authoring mistake
        rather than a design feature, but resolving them deterministically
        beats depending on declaration order.
        """
        kinds = [f.kind for f in self.faults
                 if f.topic == topic and f.active_at(t)]
        if not kinds:
            return None
        return max(kinds, key=lambda k: SEVERITY.index(k))

    def is_suppressed(self, topic: str, t: float) -> bool:
        """Whether the topic should carry nothing at all right now."""
        return self.kind_for(topic, t) is FaultKind.SILENCE

    def injection_times(self) -> List[Tuple[float, str]]:
        """(time, label) for every fault onset, for latency measurement.

        These are the t0 values EVALUATION.md §2.1 measures stop latency from.
        """
        return sorted(
            (f.start_s, "%s:%s%s" % (f.kind.value, f.topic,
                                     " (%s)" % f.scenario if f.scenario else ""))
            for f in self.faults)

    def topics(self) -> List[str]:
        return sorted({f.topic for f in self.faults})


#: Increasing severity, used to resolve overlapping faults on one topic.
#: SILENCE last because a topic that is absent cannot also be corrupted.
SEVERITY: Tuple[FaultKind, ...] = (
    FaultKind.STALE_STAMP,
    FaultKind.EMPTY,
    FaultKind.NAN,
    FaultKind.FREEZE,
    FaultKind.SILENCE,
)


# --------------------------------------------------------------------------
# The EVALUATION.md §6 failure catalogue, as schedules.
#
# Each fires once the vehicle is well under way, so the baseline is genuinely
# moving -- a stop latency measured from rest is not a stop latency
# (latency.py). None of them recover: the acceptance bar for T16-T19 is a safe
# halt, not a completed mission.
# --------------------------------------------------------------------------
CAMERA_TOPIC = "/camera/rgb/image_raw"
DEPTH_TOPIC = "/camera/depth/image_rect_raw"
POSE_TOPIC = "/rtabmap/localization_pose"
PLAN_TOPIC = "/plan"
HEALTH_TOPIC = "/perception/health"

FAILURE_SCENARIOS: Dict[str, FaultSchedule] = {
    # T16, the easy half: the camera goes silent.
    "T16_camera_dropout": FaultSchedule((
        Fault(8.0, None, CAMERA_TOPIC, FaultKind.SILENCE, "T16"),
        Fault(8.0, None, HEALTH_TOPIC, FaultKind.SILENCE, "T16"),
    )),
    # T16, the hard half: the camera keeps talking and stops telling the truth.
    # See the module docstring -- the supervisor as specified may well pass
    # this by accident and fail it in principle.
    "T16_camera_freeze": FaultSchedule((
        Fault(8.0, None, CAMERA_TOPIC, FaultKind.FREEZE, "T16"),
    )),
    "T17_depth_dropout": FaultSchedule((
        Fault(8.0, None, DEPTH_TOPIC, FaultKind.SILENCE, "T17"),
    )),
    # T17 variant: depth arrives but its stamps stop advancing.
    "T17_depth_stale": FaultSchedule((
        Fault(8.0, None, DEPTH_TOPIC, FaultKind.STALE_STAMP, "T17"),
    )),
    "T18_slam_loss": FaultSchedule((
        Fault(10.0, None, POSE_TOPIC, FaultKind.SILENCE, "T18"),
    )),
    "T19_no_valid_path": FaultSchedule((
        Fault(10.0, None, PLAN_TOPIC, FaultKind.EMPTY, "T19"),
    )),
    # Not in the catalogue, but SPEC.md §9.2 lists an invalid planner command
    # as a stop condition, and an untested branch is an unproven one.
    "invalid_command": FaultSchedule((
        Fault(10.0, None, "/cmd_vel_nav", FaultKind.NAN),
    )),
    # Two faults at once: the failures a real vehicle sees are rarely tidy.
    "T16_T18_combined": FaultSchedule((
        Fault(8.0, None, CAMERA_TOPIC, FaultKind.SILENCE, "T16"),
        Fault(8.5, None, POSE_TOPIC, FaultKind.SILENCE, "T18"),
    )),
}


def scenario(name: str) -> FaultSchedule:
    try:
        return FAILURE_SCENARIOS[name]
    except KeyError:
        raise KeyError(
            "unknown failure scenario %r. Known: %s"
            % (name, ", ".join(sorted(FAILURE_SCENARIOS)))) from None
