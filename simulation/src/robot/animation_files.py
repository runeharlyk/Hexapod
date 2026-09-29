"""Conversion between the animation dataclasses and the generated protobuf classes."""
from __future__ import annotations

from pathlib import Path

import numpy as np
from google.protobuf import json_format

from src.platform_shared import animation_pb2 as pb
from src.robot import animation as an


def _leg_from_proto(lt: pb.LegTarget) -> an.LegTarget:
    if lt.WhichOneof("target") == "joints":
        return an.LegTarget(joints=np.array([lt.joints.coxa, lt.joints.femur, lt.joints.tibia], dtype=float))
    return an.LegTarget(foot=np.array([lt.foot.x, lt.foot.y, lt.foot.z], dtype=float))


def _overlay_from_proto(o: pb.Overlay) -> an.Overlay:
    which = o.WhichOneof("channel")
    return an.Overlay(
        body_axis=o.body_axis if which == "body_axis" else None,
        foot_channel=o.foot_channel if which == "foot_channel" else None,
        amplitude=o.amplitude, frequency=o.frequency, phase=o.phase, start=o.start, end=o.end,
    )


def from_proto(msg: pb.Animation) -> an.Animation:
    keyframes = [
        an.Keyframe(
            time=k.time, ease=k.ease,
            body=np.array([k.body.roll, k.body.pitch, k.body.yaw, k.body.x, k.body.y, k.body.z], dtype=float),
            legs=[_leg_from_proto(lt) for lt in k.legs],
        )
        for k in msg.keyframes
    ]
    return an.Animation(
        name=msg.name, description=msg.description, schema=msg.schema, loop=msg.loop, hold_end=msg.hold_end,
        entry_time=msg.entry_time, exit_time=msg.exit_time, keyframes=keyframes,
        overlays=[_overlay_from_proto(o) for o in msg.overlays],
        params=[an.ParamSpec(p.id, p.min, p.default_value, p.max) for p in msg.params],
    )


def to_proto(anim: an.Animation) -> pb.Animation:
    msg = pb.Animation(name=anim.name, description=anim.description, schema=anim.schema, loop=anim.loop,
                       hold_end=anim.hold_end, entry_time=anim.entry_time, exit_time=anim.exit_time)
    for k in anim.keyframes:
        kf = msg.keyframes.add(time=k.time, ease=int(k.ease))
        kf.body.roll, kf.body.pitch, kf.body.yaw, kf.body.x, kf.body.y, kf.body.z = map(float, k.body)
        for lt in k.legs:
            target = kf.legs.add()
            if lt.is_joints():
                target.joints.coxa, target.joints.femur, target.joints.tibia = map(float, lt.joints)
            else:
                target.foot.x, target.foot.y, target.foot.z = map(float, lt.foot)
    for o in anim.overlays:
        ov = msg.overlays.add(amplitude=o.amplitude, frequency=o.frequency, phase=o.phase, start=o.start, end=o.end)
        if o.body_axis is not None:
            ov.body_axis = o.body_axis
        else:
            ov.foot_channel = o.foot_channel
    for p in anim.params:
        msg.params.add(id=int(p.id), min=p.min, default_value=p.default_value, max=p.max)
    return msg


def json_text(anim: an.Animation) -> str:
    return json_format.MessageToJson(to_proto(anim), indent=2) + "\n"


def load_json(path: str | Path) -> an.Animation:
    return from_proto(json_format.Parse(Path(path).read_text(), pb.Animation()))


def save_json(anim: an.Animation, path: str | Path) -> None:
    Path(path).write_text(json_text(anim))


def load_binary(path: str | Path) -> an.Animation:
    return from_proto(pb.Animation.FromString(Path(path).read_bytes()))


def save_binary(anim: an.Animation, path: str | Path) -> None:
    Path(path).write_bytes(to_proto(anim).SerializeToString())
