"""Unit tests for the animation reference implementation (src/robot/animation.py)."""
import json
import math

import numpy as np
import pytest
from google.protobuf import json_format

from src.platform_shared import animation_pb2 as pb
from src.robot import animation as an
from src.robot.animation_files import from_proto, json_text, load_binary, load_json, save_binary, to_proto
from src.robot.firmware_gait import BodyState, Kinematics

KIN = Kinematics()


def test_proto_json_round_trip_keeps_the_leg_oneof():
    src = {
        "name": "rt",
        "schema": 1,
        "keyframes": [
            {"time": 0},
            {"time": 1.5, "ease": "EASE_IN_OUT", "body": {"roll": 0.1, "z": 20},
             "legs": [{"foot": {"z": 30}}, {"joints": {"femur": 80, "tibia": -110}},
                      {"foot": {}}, {"foot": {}}, {"foot": {}}, {"foot": {}}]},
        ],
        "params": [{"id": "FOOT_LIFT", "min": 0.5, "defaultValue": 1, "max": 1.5}],
    }
    msg = json_format.ParseDict(src, pb.Animation())
    assert msg.keyframes[1].legs[0].WhichOneof("target") == "foot"
    assert msg.keyframes[1].legs[1].WhichOneof("target") == "joints"
    assert msg.keyframes[1].legs[2].WhichOneof("target") == "foot"
    assert msg.params[0].default_value == 1
    back = json.loads(json_format.MessageToJson(msg))
    assert back["keyframes"][1]["legs"][1] == {"joints": {"femur": 80, "tibia": -110}}
    assert pb.Animation.FromString(msg.SerializeToString()) == msg


def stance_legs():
    return [an.LegTarget.stance() for _ in range(6)]


def two_keyframes(**kw):
    return an.Animation(name="t", keyframes=[an.Keyframe(0.0), an.Keyframe(1.0)], **kw)


def test_validate_accepts_a_minimal_animation():
    assert an.validate(an.Animation(name="ok", keyframes=[an.Keyframe(0.0)])) is None


@pytest.mark.parametrize("mutate, message", [
    (lambda a: setattr(a, "schema", 2), "schema"),
    (lambda a: setattr(a, "name", "Bad Name"), "name"),
    (lambda a: setattr(a, "name", "x" * 33), "name"),
    (lambda a: setattr(a, "description", "d" * 97), "description"),
    (lambda a: setattr(a, "keyframes", []), "keyframe"),
    (lambda a: setattr(a.keyframes[0], "time", 0.1), "time 0"),
    (lambda a: setattr(a.keyframes[1], "time", 0.0), "increase"),
    (lambda a: setattr(a.keyframes[1], "legs", stance_legs()[:3]), "0 or 6"),
    (lambda a: a.overlays.append(an.Overlay(amplitude=1, frequency=1, start=0, end=1)), "channel"),
    (lambda a: a.overlays.append(an.Overlay(body_axis=6, amplitude=1, frequency=1, start=0, end=1)), "body_axis"),
    (lambda a: a.overlays.append(an.Overlay(foot_channel=18, amplitude=1, frequency=1, start=0, end=1)), "foot_channel"),
    (lambda a: a.overlays.append(an.Overlay(body_axis=0, amplitude=1, frequency=1, start=0.5, end=0.5)), "start"),
    (lambda a: a.overlays.append(an.Overlay(body_axis=0, amplitude=1, frequency=1, start=0, end=1.5)), "end"),
    (lambda a: a.params.extend([an.ParamSpec(an.ParamId.SPEED, 0.5, 1, 2)] * 2), "unique"),
    (lambda a: a.params.append(an.ParamSpec(an.ParamId.BODY_Z, 0.5, 3, 2)), "min <= default_value <= max"),
    (lambda a: a.params.append(an.ParamSpec(an.ParamId.SPEED, 0.0, 1, 2)), "SPEED"),
])
def test_validate_reports_each_structural_rule(mutate, message):
    a = two_keyframes()
    mutate(a)
    err = an.validate(a)
    assert err is not None and message in err


def test_validate_rejects_too_many_of_everything():
    a = an.Animation(name="t", keyframes=[an.Keyframe(float(i)) for i in range(33)])
    assert "32" in an.validate(a)
    a = two_keyframes(overlays=[an.Overlay(body_axis=0, amplitude=1, frequency=1, start=0, end=1)] * 9)
    assert "8" in an.validate(a)
    a = two_keyframes(params=[an.ParamSpec(an.ParamId(i), 0.5, 1, 2) for i in range(10)])
    assert an.validate(a) is None
    a.params.append(an.ParamSpec(an.ParamId.REPEAT, 1, 1, 1))
    assert "10" in an.validate(a)


def test_entry_and_exit_default_to_half_a_second():
    a = two_keyframes()
    assert a.entry_seconds() == 0.5 and a.exit_seconds() == 0.5
    a.entry_time, a.exit_time = 0.2, 0.9
    assert a.entry_seconds() == 0.2 and a.exit_seconds() == 0.9


def test_leg_target_of_an_empty_legs_array_is_stance():
    k = an.Keyframe(0.0, legs=[])
    t = an.leg_target(k, 4)
    assert not t.is_joints() and np.array_equal(t.foot, np.zeros(3))


def test_from_proto_round_trips_through_to_proto_and_json():
    a = an.Animation(
        name="rt", description="d", loop=True, hold_end=False, entry_time=0.3, exit_time=0.0,
        keyframes=[an.Keyframe(0.0), an.Keyframe(1.0, an.Ease.EASE_OUT, np.array([0.1, 0, 0, 0, 0, 20.0]),
                                                  [an.LegTarget(foot=np.array([0, 0, 30.0]))]
                                                  + [an.LegTarget(joints=np.array([0, 80.0, -110.0]))]
                                                  + stance_legs()[:4])],
        overlays=[an.Overlay(foot_channel=2, amplitude=5, frequency=2, phase=0.5, start=0, end=1)],
        params=[an.ParamSpec(an.ParamId.FOOT_LIFT, 0.5, 1.0, 1.5)],
    )
    b = from_proto(to_proto(a))
    assert an.validate(b) is None
    assert b.name == "rt" and b.loop and b.entry_time == pytest.approx(0.3)
    assert b.keyframes[1].ease == an.Ease.EASE_OUT
    assert np.allclose(b.keyframes[1].body, a.keyframes[1].body)
    assert b.keyframes[1].legs[1].is_joints()
    assert np.allclose(b.keyframes[1].legs[1].joints, [0, 80, -110])
    assert b.overlays[0].foot_channel == 2 and b.overlays[0].body_axis is None
    assert b.params[0].id == an.ParamId.FOOT_LIFT and b.params[0].default_value == 1.0
    text = json_text(b)
    assert '"defaultValue": 1.0' in text and '"holdEnd"' not in text


def test_from_proto_keeps_a_three_leg_keyframe_for_validate_to_reject():
    msg = to_proto(two_keyframes())
    for _ in range(3):
        msg.keyframes[1].legs.add().foot.z = 1.0
    a = from_proto(msg)
    assert len(a.keyframes[1].legs) == 3
    assert "0 or 6" in an.validate(a)


def test_load_json_reads_a_file(tmp_path):
    p = tmp_path / "x.json"
    p.write_text('{"name": "x", "schema": 1, "keyframes": [{"time": 0}, {"time": 0.5, "legs": [{"foot": {}}, {"foot": {}}, {"foot": {}}, {"foot": {}}, {"foot": {}}, {"joints": {"femur": 45}}]}]}')
    a = load_json(p)
    assert an.validate(a) is None
    assert a.keyframes[1].legs[5].joints[1] == 45


def test_save_binary_round_trips_through_load_binary(tmp_path):
    a = an.Animation(
        name="rt", keyframes=[an.Keyframe(0.0), an.Keyframe(1.0, an.Ease.EASE_OUT, np.zeros(6),
                                                             [an.LegTarget(foot=np.array([0, 0, 30.0]))]
                                                             + [an.LegTarget(joints=np.array([0, 80.0, -110.0]))]
                                                             + stance_legs()[:4])],
        overlays=[an.Overlay(foot_channel=2, amplitude=5, frequency=2, phase=0.5, start=0, end=1)],
    )
    p = tmp_path / "rt.pb"
    save_binary(a, p)
    b = load_binary(p)
    assert an.validate(b) is None
    assert b.keyframes[1].legs[1].is_joints()
    assert np.allclose(b.keyframes[1].legs[1].joints, [0, 80, -110])
    assert b.overlays[0].foot_channel == 2 and b.overlays[0].body_axis is None


def params_of(a, **values):
    return an.resolve_params(a, {an.ParamId[k]: v for k, v in values.items()})


def test_ease_curves_match_the_stash():
    assert an.ease_value(an.Ease.LINEAR, 0.3) == pytest.approx(0.3)
    assert an.ease_value(an.Ease.EASE_IN, 0.5) == pytest.approx(0.25)
    assert an.ease_value(an.Ease.EASE_OUT, 0.5) == pytest.approx(0.75)
    assert an.ease_value(an.Ease.EASE_IN_OUT, 0.25) == pytest.approx(0.125)
    assert an.ease_value(an.Ease.EASE_IN_OUT, 0.75) == pytest.approx(0.875)


def test_resolve_params_defaults_clamps_and_ignores_undeclared():
    a = two_keyframes(params=[an.ParamSpec(an.ParamId.SPEED, 0.5, 1.0, 2.0),
                              an.ParamSpec(an.ParamId.FOOT_LIFT, 0.0, 0.8, 1.0)])
    p = an.resolve_params(a, None)
    assert p[an.ParamId.SPEED] == 1.0 and p[an.ParamId.FOOT_LIFT] == 0.8 and p[an.ParamId.BODY_Z] == 1.0
    p = an.resolve_params(a, {an.ParamId.SPEED: 9.0, an.ParamId.BODY_Z: 0.1})
    assert p[an.ParamId.SPEED] == 2.0 and p[an.ParamId.BODY_Z] == 1.0


def test_evaluate_interpolates_body_and_feet_with_the_end_keyframe_ease():
    a = an.Animation(name="t", keyframes=[
        an.Keyframe(0.0),
        an.Keyframe(2.0, an.Ease.EASE_IN, np.array([0, 0, 0, 0, 0, 40.0]),
                    [an.LegTarget(foot=np.array([0, 0, 20.0]))] + stance_legs()[:5]),
    ])
    pose = an.evaluate(a, an.resolve_params(a, None), 1.0, KIN)
    assert pose.body[an.BodyAxis.Z] == pytest.approx(10.0)  # ease_in(0.5) = 0.25
    assert pose.legs[0].foot[2] == pytest.approx(5.0)
    assert np.array_equal(pose.legs[3].foot, np.zeros(3))


def test_evaluate_clamps_time_to_the_ends():
    a = two_keyframes()
    a.keyframes[1].body[an.BodyAxis.Z] = 30.0
    a.overlays.append(an.Overlay(body_axis=an.BodyAxis.ROLL, amplitude=1.0, frequency=0.25, phase=math.pi / 2,
                                 start=0.2, end=0.8))
    p = an.resolve_params(a, None)
    assert an.evaluate(a, p, -0.1, KIN).body[an.BodyAxis.Z] == 0.0
    assert an.evaluate(a, p, 5.0, KIN).body[an.BodyAxis.Z] == pytest.approx(30.0)
    assert an.evaluate(a, p, 5.0, KIN).body[an.BodyAxis.ROLL] == 0.0  # overlay window is closed


def test_overlay_adds_a_sine_on_a_body_or_foot_channel_and_scales_with_the_param():
    a = two_keyframes(overlays=[
        an.Overlay(body_axis=an.BodyAxis.YAW, amplitude=0.2, frequency=1.0, phase=0.0, start=0.0, end=1.0),
        an.Overlay(foot_channel=3 * 2 + 2, amplitude=10.0, frequency=1.0, phase=0.0, start=0.0, end=1.0),
    ], params=[an.ParamSpec(an.ParamId.OVERLAY_AMPLITUDE, 0.0, 1.0, 2.0)])
    pose = an.evaluate(a, params_of(a, OVERLAY_AMPLITUDE=0.5), 0.25, KIN)  # sin(pi/2) = 1
    assert pose.body[an.BodyAxis.YAW] == pytest.approx(0.1)
    assert pose.legs[2].foot[2] == pytest.approx(5.0)


def test_overlay_on_a_joint_leg_is_ignored():
    a = two_keyframes(overlays=[an.Overlay(foot_channel=0, amplitude=10.0, frequency=1.0, start=0.0, end=1.0)])
    a.keyframes[0].legs = [an.LegTarget(joints=np.array([0, 60.0, -100.0]))] + stance_legs()[:5]
    a.keyframes[1].legs = [an.LegTarget(joints=np.array([0, 60.0, -100.0]))] + stance_legs()[:5]
    pose = an.evaluate(a, an.resolve_params(a, None), 0.25, KIN)
    assert pose.legs[0].is_joints() and np.allclose(pose.legs[0].joints, [0, 60, -100])


def test_body_and_foot_lift_multipliers_apply_after_overlays():
    a = two_keyframes(overlays=[an.Overlay(body_axis=an.BodyAxis.ROLL, amplitude=0.2, frequency=1.0, start=0.0, end=1.0)],
                      params=[an.ParamSpec(an.ParamId.BODY_ROLL, 0, 1, 2), an.ParamSpec(an.ParamId.FOOT_LIFT, 0, 1, 2)])
    a.keyframes[1].body[an.BodyAxis.ROLL] = 0.4
    a.keyframes[1].legs = [an.LegTarget(foot=np.array([5.0, 0, 20.0]))] + stance_legs()[:5]
    pose = an.evaluate(a, params_of(a, BODY_ROLL=0.5, FOOT_LIFT=2.0), 0.25, KIN)
    assert pose.body[an.BodyAxis.ROLL] == pytest.approx((0.1 + 0.2) * 0.5)
    assert np.allclose(pose.legs[0].foot, [1.25, 0, 10.0])  # x is not lifted, z is


def test_mixed_foot_and_joint_keyframes_interpolate_in_joint_space():
    a = two_keyframes()
    a.keyframes[0].legs = [an.LegTarget(foot=np.zeros(3))] + stance_legs()[:5]
    a.keyframes[1].legs = [an.LegTarget(joints=np.array([0, 80.0, -110.0]))] + stance_legs()[:5]
    start = an.leg_joints_deg(KIN, np.zeros(6), np.zeros(3), 0)
    pose = an.evaluate(a, an.resolve_params(a, None), 0.5, KIN)
    assert pose.legs[0].is_joints()
    assert np.allclose(pose.legs[0].joints, (start + np.array([0, 80.0, -110.0])) / 2)


def test_leg_joints_deg_matches_the_full_body_ik():
    body = np.array([0.05, -0.02, 0.1, 5.0, -3.0, 12.0])
    b = BodyState(omega=0.05, phi=-0.02, psi=0.1, xm=5.0, ym=-3.0, zm=12.0)
    b.feet[4, :3] += [3.0, 4.0, 15.0]
    assert np.allclose(an.leg_joints_deg(KIN, body, np.array([3.0, 4.0, 15.0]), 4), KIN.inverse_kinematics(b)[12:15])


def test_pose_to_angles_overrides_joint_legs_and_clamps_with_a_mask():
    pose = an.Pose.stance()
    pose.legs[1] = an.LegTarget(joints=np.array([40.0, 95.0, -100.0]))  # coxa and femur over the limit
    pose.legs[5] = an.LegTarget(foot=np.array([0, 0, 120.0]))           # unreachable lift, IK clamps
    angles, mask = an.pose_to_angles(pose, KIN)
    assert angles.shape == (18,)
    assert angles[3] == 31.5 and angles[4] == 90.0 and angles[5] == -100.0
    assert mask & (1 << 3) and mask & (1 << 4) and not mask & (1 << 5)
    assert mask & (1 << 16)  # femur of leg 5 pinned
    assert not mask & 0b111  # leg 0 at stance is clean


def test_stance_pose_reproduces_the_standing_angles():
    angles, mask = an.pose_to_angles(an.Pose.stance(), KIN)
    assert mask == 0
    assert np.allclose(angles, KIN.inverse_kinematics(BodyState()))
