"""Check the gait speed envelope generator against a timed gait rollout and the reference sweep.

The sampled joint-rate factors replace a rollout per posture, so the first test pins them to one.
The remaining tests pin the table's reference values, its walkable region, the servo scaling of the
speed, and the agreement of the generated header with the generated JSON, and walk the port at full
stick in random walkable postures, between the grid points the table was measured at.
"""

import json
import os
import re

import numpy as np
import pytest

from src.robot import firmware_gait as fg
from src.robot import gait_envelope as ge
from src.robot.animation import JOINT_LIMIT_DEG, foot_reachable

FIRMWARE_INCLUDE = os.path.join(os.path.dirname(__file__), "..", "firmware", "include")
HEADER = os.path.join(FIRMWARE_INCLUDE, "gait_envelope.h")
TABLE_JSON = os.path.join(os.path.dirname(__file__), "src", "resources", "gait_envelope.json")
TOL = 1e-4
GRID_N = 9
MID = GRID_N // 2


def _rollout_peak_joint_rate(stroke_mm, lift_mm, dt=0.0025, seconds=3.0, settle_s=1.5):
    """Peak joint rate in rad/s of the default-posture tripod at 1 cycle/s, measured on GaitController.step."""
    gait = fg.GaitState(gait_type=fg.TRI_GATE)
    fg.set_gait(gait)
    gait.step_height, gait.step_y, gait.phase_rate = lift_mm, stroke_mm, 1.0
    controller, body, kin = fg.GaitController(), fg.BodyState(), fg.Kinematics()
    angles, swinging = [], []
    for _ in range(int(seconds / dt)):
        controller.step(gait, body, dt)
        angles.append(kin.inverse_kinematics(body, degrees=False).reshape(6, 3))
        swinging.append((controller.phase + gait.offset) % 1.0 >= gait.stand_frac)
    skip = int(settle_s / dt)
    angles, swinging = np.array(angles)[skip:], np.array(swinging)[skip:]
    rate = np.abs(np.diff(angles, axis=0)).max(axis=2) / dt
    within_one_phase = swinging[1:] == swinging[:-1]
    return rate[within_one_phase].max()


def test_sampled_kappa_matches_a_timed_rollout():
    stroke, lift = 64.0, 20.0
    kappa_swing, kappa_stance, feasible = ge.geometry_factors("forward", stroke, lift, 0.0, 1.0)
    assert feasible
    d = fg.TRI_STAND_FRAC
    sampled = max(kappa_swing / (1 - d), kappa_stance / d)
    rolled = _rollout_peak_joint_rate(stroke, lift)
    assert sampled == pytest.approx(rolled, rel=0.05)


def test_default_posture_matches_the_reference_sweep():
    forward = lambda gait: ge.top_speed(gait, "forward", 0, 0, 0, 13.0)[0]
    assert forward(fg.TRI_GATE) == pytest.approx(334, rel=0.05)
    assert forward(fg.BI_GATE) == pytest.approx(643, rel=0.05)
    assert forward(fg.WAVE) == pytest.approx(71, rel=0.10)


def test_neutral_posture_is_walkable():
    assert ge.is_walkable(0, 0, 0)


def test_known_unwalkable_postures():
    assert not ge.is_walkable(0, 1, -1), "tall body on close feet needs 111% tibia travel standing"
    assert not ge.is_walkable(0, -1, 1), "low body on wide feet is out of reach"


def test_nothing_feasible_returns_zero():
    assert ge.top_speed(fg.TRI_GATE, "forward", 0, -1, 1, 13.0) == (0.0, 0.0)


def test_top_speed_scales_with_servo_speed_when_servo_bound():
    fast = ge.top_speed(fg.WAVE, "forward", 0, 0, 0, 13.0)[0]
    slow = ge.top_speed(fg.WAVE, "forward", 0, 0, 0, 6.5)[0]
    assert slow == pytest.approx(fast / 2, rel=0.01)


def _stand_frac(gait_type):
    gait = fg.GaitState(gait_type=gait_type)
    fg.set_gait(gait)
    return gait.stand_frac


def test_cadence_cap_binds_when_servos_are_fast():
    v_max, stroke = ge.top_speed(fg.WAVE, "forward", 0, 0, 0, 1000.0)
    assert v_max == pytest.approx(stroke * ge.CADENCE_MAX / _stand_frac(fg.WAVE), rel=1e-9)


def test_stroke_grows_the_joint_rate_and_the_travel():
    short = ge.geometry_factors("forward", 20.0, 20.0, 0.0, 1.0)
    long = ge.geometry_factors("forward", 60.0, 20.0, 0.0, 1.0)
    assert long[0] > short[0] and long[1] > short[1]
    assert not ge.geometry_factors("forward", 400.0, 20.0, 0.0, 1.0)[2]


FIRMWARE_DT = 0.005
FULL_STICKS = {"forward": (0.0, 1.0, 0.0), "lateral": (1.0, 0.0, 0.0), "yaw": (0.0, 0.0, 1.0)}


def _walkable_postures(count, seed):
    rng = np.random.default_rng(seed)
    postures = []
    while len(postures) < count:
        posture = tuple(rng.uniform(-1.0, 1.0, 3))
        if fg.posture_walkable(*posture):
            postures.append(posture)
    return postures


def _worst_excursion(gait_type, stick, s1, h, fd):
    """The port's gait at the firmware's tick from a stand in the posture, the command eased in as MotionService does
    and then held at the top speed for two cycles: the largest |joint| / JOINT_LIMIT_DEG and whether every foot
    stayed in reach. None when the posture cannot walk the axis."""
    target = fg.GaitState(gait_type=gait_type)
    fg.set_gait(target)
    zm = fg.command_to_walk_gait(*stick, 1.0, s1, target, h, fd)
    if target.phase_rate <= 0.0:
        return None
    gait = fg.GaitState(**{**target.__dict__, "offset": target.offset.copy()})
    gait.step_x = gait.step_y = gait.step_angle = gait.phase_rate = 0.0
    stance = fg.DEFAULT_FEET.copy()
    stance[:, :2] *= ge.posture(s1, h, fd)[2]
    controller = fg.GaitController()
    controller.default_position = stance.copy()
    controller.target_default_position = stance.copy()
    body, kin = fg.BodyState(zm=zm, feet=stance.copy()), fg.Kinematics()
    body6 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, zm])
    worst, reachable = 0.0, True
    ramp_s = 1.0
    for _ in range(int(np.ceil((ramp_s + 2.0 / target.phase_rate) / FIRMWARE_DT))):
        fg.approach_gait_command(gait, target, FIRMWARE_DT)
        controller.step(gait, body, FIRMWARE_DT)
        angles = kin.inverse_kinematics(body).reshape(6, 3)
        worst = max(worst, float((np.abs(angles) / JOINT_LIMIT_DEG).max()))
        reachable = reachable and all(foot_reachable(kin, body6, np.zeros(3), leg, body.feet) for leg in range(6))
    return worst, reachable


@pytest.fixture(scope="module")
def full_stick_walks():
    """{(gait, axis, posture): (worst, reachable)} over 40 walkable postures. Between the grid points the envelope is
    interpolated, so a stroke read inside a walkable cell has to hold for the posture actually walked, not only at the
    corners it was measured at."""
    walks = {}
    for posture in _walkable_postures(40, seed=4):
        for gait_type, name in enumerate(ge.GAIT_NAMES):
            for axis, stick in FULL_STICKS.items():
                excursion = _worst_excursion(gait_type, stick, *posture)
                assert excursion is not None or axis != "forward", f"walkable posture {posture} cannot walk forward"
                if excursion is not None:
                    walks[name, axis, tuple(round(v, 3) for v in posture)] = excursion
    walked = {axis: sum(key[1] == axis for key in walks) for axis in FULL_STICKS}
    assert walked["lateral"] >= 40 and walked["yaw"] >= 40, walked
    return walks


def test_walkable_postures_walk_full_stick_in_reach(full_stick_walks):
    assert [where for where, (_, reachable) in full_stick_walks.items() if not reachable] == []


def test_walkable_postures_walk_full_stick_in_travel(full_stick_walks):
    beyond = {where: round(worst, 4) for where, (worst, _) in full_stick_walks.items() if worst > 1.0}
    assert beyond == {}


def _cpp_array(text, name):
    declaration = re.search(rf"constexpr \w+ {name}\s*((?:\[\d+\])*)\s*=\s*([^;]*);", text)
    assert declaration, f"{name} missing from {HEADER}"
    dims = [int(n) for n in re.findall(r"\[(\d+)\]", declaration.group(1))]
    values = np.array([float(v) for v in re.findall(r"-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?", declaration.group(2))])
    return values.reshape(dims) if dims else values[0]


def test_header_and_json_carry_the_same_table():
    with open(HEADER) as f:
        text = f.read()
    with open(TABLE_JSON) as f:
        table = json.load(f)

    assert _cpp_array(text, "GRID_N") == GRID_N == len(table["grid"])
    assert _cpp_array(text, "SERVO_RAD_S") == pytest.approx(table["servo_rad_s"], abs=TOL)
    assert _cpp_array(text, "CADENCE_MAX") == pytest.approx(table["cadence_max"], abs=TOL)
    assert _cpp_array(text, "TRAVEL_MARGIN_DEG") == pytest.approx(table["travel_margin_deg"], abs=TOL)
    assert _cpp_array(text, "DEFAULT_SPEED_FRACTION") == pytest.approx(table["default_speed_fraction"], abs=TOL)
    assert np.array_equal(_cpp_array(text, "WALKABLE"), np.array(table["walkable"], dtype=float))

    gaits = [table["gaits"][name] for name in ge.GAIT_NAMES]
    assert _cpp_array(text, "STAND_FRAC") == pytest.approx([g["stand_frac"] for g in gaits], abs=TOL)
    for header_name, key in (("V_MAX", "v_max"), ("STROKE", "stroke")):
        expected = np.array([[g[axis][key] for axis in ge.AXES] for g in gaits])
        assert expected.shape == (4, 3, GRID_N, GRID_N, GRID_N)
        assert np.abs(_cpp_array(text, header_name) - expected).max() <= TOL


@pytest.mark.parametrize("cell", [(MID, MID, MID), (0, 0, 0), (GRID_N - 1, 2, 6), (2, GRID_N - 1, 1), (5, 1, GRID_N - 1)])
def test_committed_table_agrees_with_the_generator(cell):
    with open(TABLE_JSON) as f:
        table = json.load(f)
    s1, h, fd = (float(ge.GRID[i]) for i in cell)
    i, j, k = cell
    for gait, name in enumerate(ge.GAIT_NAMES):
        for axis in ge.AXES:
            v_max, stroke = ge.top_speed(gait, axis, s1, h, fd, table["servo_rad_s"])
            entry = table["gaits"][name][axis]
            assert entry["v_max"][i][j][k] == pytest.approx(v_max, abs=TOL)
            assert entry["stroke"][i][j][k] == pytest.approx(stroke, abs=TOL)
    assert table["walkable"][i][j][k] == ge.is_walkable(s1, h, fd)
