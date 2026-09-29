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


def _non_finite(anim: Animation) -> str | None:
    if not (math.isfinite(anim.entry_time) and math.isfinite(anim.exit_time)):
        return "entry_time and exit_time must be finite"
    for i, k in enumerate(anim.keyframes):
        values = [k.time, *k.body]
        for lt in k.legs:
            values.extend(lt.joints if lt.is_joints() else lt.foot)
        if not all(math.isfinite(v) for v in values):
            return f"keyframe {i} has a non-finite value"
    for i, o in enumerate(anim.overlays):
        if not all(math.isfinite(v) for v in (o.amplitude, o.frequency, o.phase, o.start, o.end)):
            return f"overlay {i} has a non-finite value"
    for i, p in enumerate(anim.params):
        if not all(math.isfinite(v) for v in (p.min, p.default_value, p.max)):
            return f"param {i} has a non-finite value"
    return None


def validate(anim: Animation) -> str | None:
    """Returns the first structural error, or None. The firmware and app apply the same rules.

    The description limit is DESCRIPTION_MAX bytes of UTF-8, not characters, because the nanopb
    buffer is sized in bytes. Every float in the file must be finite. Keyframe ease and param id
    are checked as raw integers (0..3 and 0..9) because the proto enums are open and a decoder
    passes an unknown value through.
    """
    if anim.schema != SCHEMA_VERSION:
        return f"schema {anim.schema} is not {SCHEMA_VERSION}"
    if not 1 <= len(anim.name) <= NAME_MAX or not set(anim.name) <= NAME_CHARS:
        return f"name must be 1-{NAME_MAX} characters of [a-z0-9_-]"
    if len(anim.description.encode("utf-8")) > DESCRIPTION_MAX:
        return f"description longer than {DESCRIPTION_MAX} bytes"
    if anim.loop and anim.hold_end:
        return "loop and hold_end cannot both be set"
    if not anim.keyframes:
        return "at least one keyframe is required"
    if len(anim.keyframes) > KEYFRAME_MAX:
        return f"more than {KEYFRAME_MAX} keyframes"
    if len(anim.overlays) > OVERLAY_MAX:
        return f"more than {OVERLAY_MAX} overlays"
    if len(anim.params) > PARAM_MAX:
        return f"more than {PARAM_MAX} params"
    for i, k in enumerate(anim.keyframes):
        if len(k.legs) not in (0, 6):
            return f"keyframe {i} must have 0 or 6 legs"
        if not 0 <= k.ease <= 3:
            return f"keyframe {i} ease out of range"
    err = _non_finite(anim)
    if err:
        return err
    if anim.keyframes[0].time != 0.0:
        return "first keyframe must be at time 0"
    for i in range(1, len(anim.keyframes)):
        if anim.keyframes[i].time <= anim.keyframes[i - 1].time:
            return f"keyframe {i} time must increase"
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
    seen = set()
    for i, p in enumerate(anim.params):
        if not 0 <= p.id <= 9:
            return f"param {i} id out of range"
        name = ParamId(p.id).name
        if p.id in seen:
            return f"param {name} is not unique"
        seen.add(p.id)
        if not p.min <= p.default_value <= p.max:
            return f"param {name} needs min <= default_value <= max"
        if p.id == ParamId.SPEED and p.min <= 0:
            return "param SPEED needs a positive min"
        if p.id == ParamId.REPEAT and p.min < 1:
            return "param REPEAT needs min >= 1"
    return None


@dataclass
class Pose:
    body: np.ndarray                 # [roll, pitch, yaw, x, y, z] offsets
    legs: list[LegTarget]            # exactly 6

    @staticmethod
    def stance() -> "Pose":
        return Pose(np.zeros(6), [LegTarget.stance() for _ in range(6)])

    def copy(self) -> "Pose":
        return Pose(self.body.copy(), [l.copy() for l in self.legs])


def ease_value(kind: Ease, t: float) -> float:
    if kind == Ease.EASE_IN:
        return t * t
    if kind == Ease.EASE_OUT:
        return t * (2.0 - t)
    if kind == Ease.EASE_IN_OUT:
        return 2.0 * t * t if t < 0.5 else -1.0 + (4.0 - 2.0 * t) * t
    return t


def resolve_params(anim: Animation, values: dict[ParamId, float] | None) -> np.ndarray:
    """Every id gets a value: declared ids take the caller's value clamped to the spec, else the
    default; undeclared ids are 1 (the neutral multiplier and a single play)."""
    out = np.ones(len(ParamId))
    for spec in anim.params:
        v = spec.default_value if values is None or spec.id not in values else values[spec.id]
        out[spec.id] = min(max(v, spec.min), spec.max)
    return out


def _body_state(body6: np.ndarray) -> BodyState:
    return BodyState(omega=float(body6[0]), phi=float(body6[1]), psi=float(body6[2]),
                     xm=float(body6[3]), ym=float(body6[4]), zm=float(body6[5]))


def leg_joints_deg(kin: Kinematics, body6: np.ndarray, foot_offset: np.ndarray, leg: int) -> np.ndarray:
    b = _body_state(body6)
    b.feet[leg, :3] += foot_offset
    return kin.inverse_kinematics(b)[leg * 3:leg * 3 + 3]


def _segment(anim: Animation, t: float) -> tuple[Keyframe, Keyframe, float]:
    """The keyframe pair bracketing t and the eased fraction between them, with t clamped."""
    kfs = anim.keyframes
    if t <= 0.0 or len(kfs) == 1:
        return kfs[0], kfs[0], 0.0
    if t >= kfs[-1].time:
        return kfs[-1], kfs[-1], 0.0
    i = 1
    while kfs[i].time < t:
        i += 1
    k0, k1 = kfs[i - 1], kfs[i]
    return k0, k1, ease_value(k1.ease, (t - k0.time) / (k1.time - k0.time))


def _lifted(foot: np.ndarray, overlay: np.ndarray, lift: float) -> np.ndarray:
    out = foot + overlay
    out[2] *= lift
    return out


def _resolve_leg(kin: Kinematics, a: LegTarget, b: LegTarget, overlay: np.ndarray, lift: float,
                 body: np.ndarray, leg: int, u: float) -> LegTarget:
    if not a.is_joints() and not b.is_joints():
        return LegTarget(foot=_lifted(a.foot + (b.foot - a.foot) * u, overlay, lift))
    ja = a.joints if a.is_joints() else leg_joints_deg(kin, body, _lifted(a.foot, overlay, lift), leg)
    jb = b.joints if b.is_joints() else leg_joints_deg(kin, body, _lifted(b.foot, overlay, lift), leg)
    return LegTarget(joints=ja + (jb - ja) * u)


def evaluate(anim: Animation, params: np.ndarray, t: float, kin: Kinematics) -> Pose:
    """The pose at animation time t (clamped to [0, duration]), computed in this order:

    1. Interpolate the body between the bracketing keyframes with the end keyframe's easing.
    2. Add every overlay whose window contains t: a body overlay adds to the body; a foot overlay
       adds to a per-leg foot_overlay[leg][axis] accumulator, not yet to any leg.
    3. Multiply each body channel by its BODY_* param. This is the output body.
    4. Resolve each leg from its two endpoint targets, where lifted(f) = f + foot_overlay[leg]
       with its z then multiplied by FOOT_LIFT:
       - foot to foot: lifted(lerp of the two offsets).
       - joints to joints: lerp of the angles; overlays and multipliers do not apply.
       - mixed: the foot endpoint becomes IK(output body, lifted(foot)) and the leg is lerped in
         joint space with the raw joint endpoint.
    Because the mixed IK uses the output body and the lifted foot, a leg switching between a foot
    and a joint target is continuous across the keyframe under any multiplier or overlay.
    """
    k0, k1, u = _segment(anim, t)
    t = min(max(t, 0.0), anim.duration)
    body = k0.body + (k1.body - k0.body) * u
    foot_overlay = np.zeros((6, 3))
    for o in anim.overlays:
        if not o.start <= t <= o.end:
            continue
        v = o.amplitude * params[ParamId.OVERLAY_AMPLITUDE] * math.sin(2.0 * math.pi * o.frequency * t + o.phase)
        if o.body_axis is not None:
            body[o.body_axis] += v
        else:
            foot_overlay[o.foot_channel // 3, o.foot_channel % 3] += v
    for axis, pid in enumerate(BODY_PARAM_FOR_AXIS):
        body[axis] *= params[pid]
    lift = params[ParamId.FOOT_LIFT]
    legs = [_resolve_leg(kin, leg_target(k0, i), leg_target(k1, i), foot_overlay[i], lift, body, i, u)
            for i in range(6)]
    return Pose(body, legs)


def foot_reachable(kin: Kinematics, body6: np.ndarray, foot_offset: np.ndarray, leg: int) -> bool:
    """Whether the femur-tibia pair can reach the foot. The foot is transformed exactly as
    Kinematics.inverse_kinematics does (body transform, mount offset, mount rotation), then
    radial = hypot(lx - root_j1, ly) - j1_j2 and lr = hypot(radial, lz), and the foot is reachable
    when |j2_j3 - j3_tip| <= lr <= j2_j3 + j3_tip. The IK clamps its acos arguments, so outside
    that range it silently returns a straight or folded leg with in-limit angles."""
    b = _body_state(body6)
    foot = b.feet[leg].copy()
    foot[:3] += foot_offset
    w = kin.transformation_matrix(b) @ foot
    wx = w[0] - kin.mount_pos[leg][0]
    wy = w[1] - kin.mount_pos[leg][1]
    lz = w[2] - kin.mount_pos[leg][2]
    lx = wx * kin.ca[leg] + wy * kin.sa[leg]
    ly = wx * kin.sa[leg] - wy * kin.ca[leg]
    radial = math.hypot(lx - kin.root_j1, ly) - kin.j1_j2
    lr = math.hypot(radial, lz)
    return abs(kin.j2_j3 - kin.j3_tip) <= lr <= kin.j2_j3 + kin.j3_tip


def pose_to_angles(pose: Pose, kin: Kinematics) -> tuple[np.ndarray, int]:
    """18 servo angles (deg, IK order) and an 18-bit mask, bit leg * 3 + joint, of the joints that
    hit a limit. A foot leg that fails foot_reachable also sets its femur and tibia bits, the joints
    the IK saturates. The firmware mirrors this with the same kinematics constants."""
    b = _body_state(pose.body)
    for i, leg in enumerate(pose.legs):
        if not leg.is_joints():
            b.feet[i, :3] += leg.foot
    angles = kin.inverse_kinematics(b)
    for i, leg in enumerate(pose.legs):
        if leg.is_joints():
            angles[i * 3:i * 3 + 3] = leg.joints
    limit = np.tile(JOINT_LIMIT_DEG, 6)
    clamped = np.clip(angles, -limit, limit)
    mask = 0
    for j in np.flatnonzero(clamped != angles):
        mask |= 1 << int(j)
    for i, leg in enumerate(pose.legs):
        if not leg.is_joints() and not foot_reachable(kin, pose.body, leg.foot, i):
            mask |= 0b110 << (i * 3)
    return clamped, mask


class State(IntEnum):
    IDLE = 0
    ENTRY = 1
    PLAYING = 2
    HOLD = 3
    EXIT = 4


def capture_pose(body: BodyState) -> Pose:
    body6 = np.array([body.omega, body.phi, body.psi, body.xm, body.ym, body.zm], dtype=float)
    legs = [LegTarget(foot=(body.feet[i, :3] - DEFAULT_FEET[i, :3]).astype(float)) for i in range(6)]
    return Pose(body6, legs)


def _blend_targets(kin: Kinematics, src: Pose, dst: Pose) -> tuple[Pose, Pose]:
    """Legs where either side is a joint target are converted to joints on both sides once, so the
    blend itself is a plain lerp. Foot legs keep their offsets and get the step arc."""
    a, b = src.copy(), dst.copy()
    for i in range(6):
        if a.legs[i].is_joints() or b.legs[i].is_joints():
            if not a.legs[i].is_joints():
                a.legs[i] = LegTarget(joints=leg_joints_deg(kin, a.body, a.legs[i].foot, i))
            if not b.legs[i].is_joints():
                b.legs[i] = LegTarget(joints=leg_joints_deg(kin, b.body, b.legs[i].foot, i))
    return a, b


def _blend(a: Pose, b: Pose, u: float) -> Pose:
    e = ease_value(Ease.EASE_IN_OUT, u)
    legs = []
    for la, lb in zip(a.legs, b.legs):
        if la.is_joints():
            legs.append(LegTarget(joints=la.joints + (lb.joints - la.joints) * e))
            continue
        foot = la.foot + (lb.foot - la.foot) * e
        travel = math.hypot(*(lb.foot[:2] - la.foot[:2]))
        if travel > STEP_ARC_MIN_TRAVEL_MM:
            foot[2] += STEP_ARC_MM * min(1.0, travel / STEP_ARC_FULL_TRAVEL_MM) * math.sin(math.pi * u)
        legs.append(LegTarget(foot=foot))
    return Pose(a.body + (b.body - a.body) * e, legs)


class Player:
    """Entry -> Playing -> Hold | Exit -> Done, around evaluate(). Mirrors the firmware AnimationPlayer.

    A non-looping animation plays max(1, floor(REPEAT + 0.5)) times: REPEAT is rounded half up and
    clamped to at least 1, so every language agrees (Python's round() would round half to even).
    """

    def __init__(self, kin: Kinematics | None = None):
        self.kin = kin or Kinematics()
        self.state = State.IDLE
        self.animation: Animation | None = None
        self.params = np.ones(len(ParamId))
        self.t = 0.0
        self.last_pose = Pose.stance()
        self._plays_done = 0
        self._blend_t = 0.0
        self._blend_seconds = 1.0
        self._blend_from = Pose.stance()
        self._blend_to = Pose.stance()

    def play(self, anim: Animation, values: dict[ParamId, float] | None = None, live: Pose | None = None) -> None:
        self.animation = anim
        self.params = resolve_params(anim, values)
        self.t = 0.0
        self._plays_done = 0
        start = live if live is not None else self.last_pose
        self._start_blend(start, evaluate(anim, self.params, 0.0, self.kin), anim.entry_seconds(), State.ENTRY)

    def stop(self) -> None:
        if self.state == State.IDLE:
            return
        self._start_blend(self.last_pose, Pose.stance(), self.animation.exit_seconds(), State.EXIT)

    def update(self, dt: float) -> Pose:
        if self.state == State.IDLE:
            return self.last_pose
        if self.state in (State.ENTRY, State.EXIT):
            pose = self._advance_blend(dt)
        elif self.state == State.HOLD:
            pose = evaluate(self.animation, self.params, self.animation.duration, self.kin)
        else:
            pose = self._advance_playing(dt)
        self.last_pose = pose
        return pose

    def _start_blend(self, src: Pose, dst: Pose, seconds: float, state: State) -> None:
        self._blend_from, self._blend_to = _blend_targets(self.kin, src, dst)
        self._blend_seconds = seconds
        self._blend_t = 0.0
        self.state = state

    def _advance_blend(self, dt: float) -> Pose:
        self._blend_t += dt
        u = min(1.0, self._blend_t / self._blend_seconds)
        pose = _blend(self._blend_from, self._blend_to, u)
        if u >= 1.0:
            if self.state == State.ENTRY:
                self.state = State.PLAYING
                self.t = 0.0
            else:
                self.state = State.IDLE
        return pose

    def _advance_playing(self, dt: float) -> Pose:
        anim = self.animation
        duration = anim.duration
        self.t += dt * self.params[ParamId.SPEED]
        if anim.loop:
            self.t = self.t % duration if duration > 0.0 else 0.0
            return evaluate(anim, self.params, self.t, self.kin)
        if self.t < duration:
            return evaluate(anim, self.params, self.t, self.kin)
        self._plays_done += 1
        if self._plays_done < max(1, math.floor(self.params[ParamId.REPEAT] + 0.5)):
            self.t = self.t - duration if duration > 0.0 else 0.0
            return evaluate(anim, self.params, self.t, self.kin)
        final = evaluate(anim, self.params, duration, self.kin)
        if anim.hold_end:
            self.state = State.HOLD
        else:
            self.last_pose = final
            self.stop()
        return final
