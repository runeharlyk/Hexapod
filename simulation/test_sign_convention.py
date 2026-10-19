"""The body-frame sign convention against platform_shared/sign_convention.json, which the firmware's native
test (firmware/test/test_sign_convention) and the app's sign-convention.spec.ts pass too. docs/conventions.md derives
the signs."""
import json
import math
from pathlib import Path

import numpy as np
import pytest

from src.robot import firmware_gait as fg
from src.sim.firmware_loop import FirmwareLoop
from src.sim.mj_runtime import HexapodSim

FIXTURE = json.loads(
    (Path(__file__).resolve().parents[1] / "platform_shared" / "sign_convention.json").read_text()
)
SAMPLING = FIXTURE["sampling"]
LEGS = FIXTURE["legs"]
STANCE = np.array(FIXTURE["stance_mm"], dtype=float)
COMMANDS = {c["name"]: c for c in FIXTURE["commands"]}
STAND = FIXTURE["stand"]
STAND_STICKS = {s["name"]: s for s in STAND["sticks"]}


def _sign(value: float, zero: float) -> int:
    return 1 if value > zero else (-1 if value < -zero else 0)


def _walk(command: dict):
    """Mean stance-foot displacement per tick per leg, and the rigid body motion it implies.

    A planted foot moves against the body, so v_body = -mean(v_foot), and the yaw rate is the
    least-squares rotation of the feet about the body centre, negated."""
    gait = fg.GaitState(gait_type=fg.TRI_GATE, step_x=command["gait"]["step_x"],
                        step_y=command["gait"]["step_y"], step_angle=command["gait"]["step_angle"],
                        step_height=40.0)
    fg.set_gait(gait)
    controller = fg.GaitController()
    body = fg.BodyState()
    dt = SAMPLING["dt_s"]
    controller.step(gait, body, dt)
    # A cycle started from rest carries the feet's offset off over FEET_EASE_S; the convention is a
    # steady walk's, so sampling starts once that has gone.
    for _ in range(math.ceil(fg.FEET_EASE_S / dt)):
        controller.step(gait, body, dt)

    def in_stance(phase: float, leg: int) -> bool:
        return math.fmod(phase + gait.offset[leg], 1.0) < gait.stand_frac

    total = np.zeros((6, 2))
    ticks = np.zeros(6, dtype=int)
    for _ in range(SAMPLING["ticks"]):
        phase_before = controller.phase
        before = body.feet[:, :2].copy()
        controller.step(gait, body, dt)
        for i in range(6):
            if in_stance(phase_before, i) and in_stance(controller.phase, i):
                total[i] += body.feet[i, :2] - before[i]
                ticks[i] += 1

    foot = total / np.maximum(ticks, 1)[:, None]
    body_velocity = -foot.mean(axis=0)
    cross = STANCE[:, 0] * foot[:, 1] - STANCE[:, 1] * foot[:, 0]
    yaw = -cross.sum() / (STANCE[:, :2] ** 2).sum()
    return foot, ticks, body_velocity, yaw


def test_fixture_stance_is_the_gait_default_stance():
    np.testing.assert_array_equal(STANCE, fg.DEFAULT_FEET[:, :3])


def test_leg_numbering_matches_the_kinematic_mounts():
    for i, name in enumerate(LEGS):
        assert np.sign(STANCE[i, 0]) == np.sign(fg.MOUNT_X[i]), name
        assert np.sign(STANCE[i, 1]) == np.sign(fg.MOUNT_Y[i]), name


@pytest.mark.parametrize("name", list(COMMANDS))
def test_stick_maps_to_the_fixture_gait_command(name):
    command = COMMANDS[name]
    gait = fg.GaitState(gait_type=fg.TRI_GATE)
    stick = command["stick"]
    fg.command_to_walk_gait(stick["lx"], stick["ly"], stick["rx"], 0.0, 0.0, gait)
    assert (gait.step_x, gait.step_y, gait.step_angle) == pytest.approx(
        (command["gait"]["step_x"], command["gait"]["step_y"], command["gait"]["step_angle"]))


@pytest.mark.parametrize("name", list(COMMANDS))
def test_stance_feet_move_with_the_fixture_signs(name):
    command = COMMANDS[name]
    foot, ticks, _, _ = _walk(command)
    zero = SAMPLING["zero_mm_per_tick"]
    for i, leg in enumerate(LEGS):
        assert ticks[i] > 0, leg
        assert [_sign(foot[i, 0], zero), _sign(foot[i, 1], zero)] == \
            command["stance_foot_displacement_sign"][i], leg


@pytest.mark.parametrize("name", list(COMMANDS))
def test_body_motion_has_the_fixture_signs(name):
    command = COMMANDS[name]
    _, _, body_velocity, yaw = _walk(command)
    zero = SAMPLING["zero_mm_per_tick"]
    assert [_sign(body_velocity[0], zero), _sign(body_velocity[1], zero)] == command["body_velocity_sign"]
    assert _sign(yaw, SAMPLING["zero_rad_per_tick"]) == command["yaw_sign"]


def _stand(stick: dict):
    """The firmware's STAND in MuJoCo, its self-levelling fed the simulated IMU: how far the base moved over the
    floor (mm) and world up in the body frame, once the stick has settled. The robot starts facing world +Y with its
    right side toward world +X, so the world's x and y are the body's."""
    sim = HexapodSim()
    sim.reset_to_stand()
    loop = FirmwareLoop(sim, fg.TRI_GATE)
    loop.hold()
    for _ in range(STAND["settle_ticks"]):
        loop.tick()
    start = sim.data.qpos[:2].copy()
    loop.stand(**stick)
    for _ in range(STAND["settle_ticks"]):
        loop.tick()
    w, x, y, z = sim.base_quat()
    up = (2 * (x * z - w * y), 2 * (y * z + w * x))
    return (sim.data.qpos[:2] - start) * 1000, up


@pytest.mark.parametrize("name", list(STAND_STICKS))
def test_a_standing_body_shifts_and_tilts_with_the_fixture_signs(name):
    row = STAND_STICKS[name]
    shift, up = _stand(row["stick"])
    assert [_sign(v, STAND["zero_mm"]) for v in shift] == row["body_shift_sign"]
    # Nose up tips world up toward body +y; right side down tips it toward body -x.
    tilt = (up[1], -up[0])
    assert [_sign(v, STAND["zero_rad"]) for v in tilt] == row["body_tilt_sign"]
