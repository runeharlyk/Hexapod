"""Reference implementation of the animation evaluator and player.

This module is the port target for firmware/include/animation/ and app/src/lib/animation/.
The golden fixtures in animations/fixtures/expected.json are generated from it, and the other two
ports are tested against those fixtures, so a behaviour change here is a behaviour change on the
robot. It deliberately imports no protobuf code; animation_files.py does the conversion.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import IntEnum

import numpy as np

from src.robot.firmware_gait import DEFAULT_FEET, BodyState, Kinematics

JOINT_LIMIT_DEG = np.array([31.5, 90.0, 149.0])  # mirrors kinematics.h JOINT_LIMIT_DEG
SCHEMA_VERSION = 1
NAME_MAX = 32
DESCRIPTION_MAX = 96
KEYFRAME_MAX = 32
OVERLAY_MAX = 8
PARAM_MAX = 10
NAME_CHARS = frozenset("abcdefghijklmnopqrstuvwxyz0123456789_-")
DEFAULT_ENTRY_S = 0.5
DEFAULT_EXIT_S = 0.5
STEP_ARC_MM = 45.0
STEP_ARC_FULL_TRAVEL_MM = 40.0
STEP_ARC_MIN_TRAVEL_MM = 2.0


class Ease(IntEnum):
    LINEAR = 0
    EASE_IN = 1
    EASE_OUT = 2
    EASE_IN_OUT = 3


class ParamId(IntEnum):
    SPEED = 0
    BODY_X = 1
    BODY_Y = 2
    BODY_Z = 3
    BODY_ROLL = 4
    BODY_PITCH = 5
    BODY_YAW = 6
    FOOT_LIFT = 7
    OVERLAY_AMPLITUDE = 8
    REPEAT = 9


class BodyAxis(IntEnum):
    ROLL = 0
    PITCH = 1
    YAW = 2
    X = 3
    Y = 4
    Z = 5


BODY_PARAM_FOR_AXIS = (ParamId.BODY_ROLL, ParamId.BODY_PITCH, ParamId.BODY_YAW,
                       ParamId.BODY_X, ParamId.BODY_Y, ParamId.BODY_Z)


@dataclass
class LegTarget:
    foot: np.ndarray | None = None    # mm offset from the standing foot [x, y, z]
    joints: np.ndarray | None = None  # deg [coxa, femur, tibia]

    @staticmethod
    def stance() -> "LegTarget":
        return LegTarget(foot=np.zeros(3))

    def is_joints(self) -> bool:
        return self.joints is not None

    def copy(self) -> "LegTarget":
        return LegTarget(None if self.foot is None else self.foot.copy(),
                         None if self.joints is None else self.joints.copy())


@dataclass
class Keyframe:
    time: float
    ease: Ease = Ease.LINEAR
    body: np.ndarray = field(default_factory=lambda: np.zeros(6))
    legs: list[LegTarget] = field(default_factory=list)


@dataclass
class Overlay:
    body_axis: int | None = None
    foot_channel: int | None = None
    amplitude: float = 0.0
    frequency: float = 1.0
    phase: float = 0.0
    start: float = 0.0
    end: float = 0.0


@dataclass
class ParamSpec:
    id: ParamId
    min: float
    default_value: float
    max: float


@dataclass
class Animation:
    name: str
    description: str = ""
    schema: int = SCHEMA_VERSION
    loop: bool = False
    hold_end: bool = False
    entry_time: float = 0.0
    exit_time: float = 0.0
    keyframes: list[Keyframe] = field(default_factory=list)
    overlays: list[Overlay] = field(default_factory=list)
    params: list[ParamSpec] = field(default_factory=list)

    @property
    def duration(self) -> float:
        return self.keyframes[-1].time if self.keyframes else 0.0

    def entry_seconds(self) -> float:
        return self.entry_time if self.entry_time > 0 else DEFAULT_ENTRY_S

    def exit_seconds(self) -> float:
        return self.exit_time if self.exit_time > 0 else DEFAULT_EXIT_S


def leg_target(keyframe: Keyframe, leg: int) -> LegTarget:
    return keyframe.legs[leg] if keyframe.legs else LegTarget.stance()


def validate(anim: Animation) -> str | None:
    """Returns the first structural error, or None. The firmware and app apply the same rules."""
    if anim.schema != SCHEMA_VERSION:
        return f"schema {anim.schema} is not {SCHEMA_VERSION}"
    if not 1 <= len(anim.name) <= NAME_MAX or not set(anim.name) <= NAME_CHARS:
        return f"name must be 1-{NAME_MAX} characters of [a-z0-9_-]"
    if len(anim.description) > DESCRIPTION_MAX:
        return f"description longer than {DESCRIPTION_MAX}"
    if not anim.keyframes:
        return "at least one keyframe is required"
    if len(anim.keyframes) > KEYFRAME_MAX:
        return f"more than {KEYFRAME_MAX} keyframes"
    if anim.keyframes[0].time != 0.0:
        return "first keyframe must be at time 0"
    for i in range(1, len(anim.keyframes)):
        if anim.keyframes[i].time <= anim.keyframes[i - 1].time:
            return f"keyframe {i} time must increase"
    for i, k in enumerate(anim.keyframes):
        if len(k.legs) not in (0, 6):
            return f"keyframe {i} must have 0 or 6 legs"
    if len(anim.overlays) > OVERLAY_MAX:
        return f"more than {OVERLAY_MAX} overlays"
    for i, o in enumerate(anim.overlays):
        if (o.body_axis is None) == (o.foot_channel is None):
            return f"overlay {i} needs exactly one channel"
        if o.body_axis is not None and not 0 <= o.body_axis <= 5:
            return f"overlay {i} body_axis out of range"
        if o.foot_channel is not None and not 0 <= o.foot_channel <= 17:
            return f"overlay {i} foot_channel out of range"
        if o.start < 0 or o.start >= o.end:
            return f"overlay {i} window must have 0 <= start < end"
        if o.end > anim.duration:
            return f"overlay {i} end is after the last keyframe"
    if len(anim.params) > PARAM_MAX:
        return f"more than {PARAM_MAX} params"
    seen = set()
    for p in anim.params:
        if p.id in seen:
            return f"param {p.id.name} is not unique"
        seen.add(p.id)
        if not p.min <= p.default_value <= p.max:
            return f"param {p.id.name} needs min <= default_value <= max"
        if p.id == ParamId.SPEED and p.min <= 0:
            return "param SPEED needs a positive min"
    return None
