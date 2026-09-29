"""check_animation.py runs an animation through the servo model and reports what the preview cannot."""
from pathlib import Path

import numpy as np

import check_animation as ca
from src.robot import animation as an
from src.robot.animation_files import load_json, save_json
from src.robot.firmware_gait import Kinematics
from src.sim.mj_runtime import HexapodSim

LIBRARY = Path(__file__).resolve().parents[1] / "animations"


def hard_roll():
    """A 1.3 rad body roll: the legs clamp and the sim robot tilts about 38 deg (measured 2026-09-29)
    without tipping over. A statically stable hexapod rarely falls, so the verdict plumbing is tested
    by lowering the threshold below that measured tilt."""
    return an.Animation(name="tip", keyframes=[
        an.Keyframe(0.0), an.Keyframe(0.15, body=np.array([1.3, 0, 0, 0, 0, 0])),
        an.Keyframe(2.0, body=np.array([1.3, 0, 0, 0, 0, 0]))])


def test_crouch_is_feasible_with_no_clamp_knock_or_fall():
    r = ca.check(load_json(LIBRARY / "crouch.json"), HexapodSim(), Kinematics())
    assert r.error is None and not r.fell and r.clamped_mask == 0
    assert r.knock_fraction == 0.0
    assert 0.0 < r.peak_joint_speed < ca.SERVO_NOLOAD
    assert r.tilt_max_deg < 5.0


def test_play_dead_touches_the_ground_on_purpose_and_is_not_a_fall():
    r = ca.check(load_json(LIBRARY / "play_dead.json"), HexapodSim(), Kinematics())
    assert r.error is None and not r.fell
    assert r.knock_fraction > 0.2  # the belly rests on the ground during the hold


def test_hard_roll_reports_a_large_tilt_and_clamped_joints_but_no_fall_at_the_default_threshold():
    r = ca.check(hard_roll(), HexapodSim(), Kinematics())
    assert r.tilt_max_deg > 30.0 and r.clamped_mask != 0
    assert not r.fell


def test_fall_verdict_follows_the_tilt_threshold(monkeypatch):
    monkeypatch.setattr(ca, "FALL_TILT_DEG", 30.0)
    r = ca.check(hard_roll(), HexapodSim(), Kinematics())
    assert r.fell and not r.ok()


def test_an_impossible_snap_exceeds_the_servo_speed():
    a = an.Animation(name="snap", entry_time=0.05, keyframes=[
        an.Keyframe(0.0), an.Keyframe(0.02, body=np.array([0, 0, 0, 0, 0, 30.0])), an.Keyframe(0.5)])
    r = ca.check(a, HexapodSim(), Kinematics())
    assert r.peak_joint_speed > ca.SERVO_NOLOAD


def test_an_invalid_animation_reports_the_error_without_simulating():
    a = an.Animation(name="Bad", keyframes=[an.Keyframe(0.0)])
    r = ca.check(a, HexapodSim(), Kinematics())
    assert r.error is not None and r.steps == 0


def test_main_exits_nonzero_on_a_fall_and_prints_the_row(tmp_path, capsys, monkeypatch):
    monkeypatch.setattr(ca, "FALL_TILT_DEG", 30.0)
    save_json(hard_roll(), tmp_path / "tip.json")
    assert ca.main([str(tmp_path / "tip.json")]) == 1
    out = capsys.readouterr().out
    assert "tip" in out and "FELL" in out and "0/1 passed" in out
