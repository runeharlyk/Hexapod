"""Check the gait speed envelope generator against a timed gait rollout and the reference sweep.

The sampled joint-rate factors replace a rollout per posture, so the first test pins them to one.
The remaining tests pin the table's reference values, its walkable region in each stance, the legs' clearance and the
static margin that bound a stroke, the servo scaling of the speed, the per-stroke geometry samples the firmware
interpolates, and the agreement of the generated header with the generated JSON, and walk the port at full stick and at the shortest, the
speed-optimal and the longest stride in random walkable postures of each stance, between the grid points the table was
measured at.
"""

import json
import math
import os
import re

import numpy as np
import pytest

import compare_stances
from src.robot import clearance
from src.robot import firmware_gait as fg
from src.robot import gait_envelope as ge
from src.robot.animation import JOINT_LIMIT_DEG, foot_reachable

FIRMWARE_INCLUDE = os.path.join(os.path.dirname(__file__), "..", "firmware", "include")
HEADER = os.path.join(FIRMWARE_INCLUDE, "gait_envelope.h")
TABLE_JSON = os.path.join(os.path.dirname(__file__), "src", "resources", "gait_envelope.json")
TOL = 1e-4
GRID_N = 9
MID = GRID_N // 2
STANCES = (fg.STANCE_DEFAULT, fg.STANCE_CRAB)
# The largest fall of a swing kappa sample below the highest before it, measured over each stance's walkable cells,
# with under a point of slack: default 1.86% forward, 1.87% lateral, 18.1% in yaw; crab 5.14% forward, 5.18% lateral,
# 1.27% in yaw. A dip lets the reference sweep, which reads kappa as it is, run a stroke faster than the firmware, which
# reads the running maximum.
KAPPA_DIP = {
    fg.STANCE_DEFAULT: {"forward": 0.025, "lateral": 0.025, "yaw": 0.19},
    fg.STANCE_CRAB: {"forward": 0.06, "lateral": 0.055, "yaw": 0.015},
}
SPEED_ROUNDING = 1e-3  # the table's kappas are rounded to 4 decimals
MONOTONE_SHARE = 0.95
KAPPA_FALL_TOL = 0.01  # a fall below the running peak of this share of it is the curve sampling's noise
BELOW_SAMPLE_TOL = 0.02


@pytest.fixture(scope="module")
def table():
    with open(TABLE_JSON) as f:
        return json.load(f)


def _tables(table, stance):
    """One stance's tables of the JSON."""
    return table["stances"][fg.STANCE_NAMES[stance]]


def _sample_strokes(stroke_max):
    return stroke_max * np.arange(1, ge.STROKE_SAMPLES + 1) / ge.STROKE_SAMPLES


def _best_sample(table, stance, gait_type, axis, cell):
    """The fastest sampled stroke of the gait on the axis at a grid cell, (speed, stroke), from the table's kappas
    alone by the cadence law the firmware applies, which reads the running maximum of the samples."""
    d = ge.STAND_FRAC[gait_type]
    tables = _tables(table, stance)
    stroke_max = tables["stroke_max"][axis][cell[0]][cell[1]][cell[2]]
    kappa_swing = np.maximum.accumulate(tables["kappa_swing"][axis][cell[0]][cell[1]][cell[2]])
    kappa_stance = np.maximum.accumulate(tables["kappa_stance"][axis][cell[0]][cell[1]][cell[2]])
    cadence = np.minimum(table["servo_rad_s"] / np.maximum(kappa_swing / (1 - d), kappa_stance / d), ge.CADENCE_MAX)
    strokes = _sample_strokes(stroke_max)
    speed = strokes * cadence / d
    best = int(np.argmax(speed))
    return float(speed[best]), float(strokes[best])


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


@pytest.mark.parametrize("axis, stroke", [("forward", 119.5), ("lateral", 65.5), ("yaw", 0.775)])
def test_sampled_kappa_does_not_understate_the_peak_joint_rate(monkeypatch, axis, stroke):
    """Kappa is the largest joint step between curve samples, which understates the peak rate between them, and the
    servos are then asked for that much more than their speed. At the neutral posture's longest strokes it stays within
    1% of a ten times finer sampling, 0.04 deg of a servo's 3.75 deg per 5 ms tick."""
    lift, zm, feet_scale = ge.posture(0.0, 0.0, 0.0)
    feet = fg.stance_feet(fg.STANCE_DEFAULT, feet_scale)
    sampled = ge.kappas(axis, stroke, lift, zm, feet)[:2]
    monkeypatch.setattr(ge, "SWING_SAMPLES", 10 * ge.SWING_SAMPLES)
    monkeypatch.setattr(ge, "STANCE_SAMPLES", 10 * ge.STANCE_SAMPLES)
    fine = ge.kappas(axis, stroke, lift, zm, feet)[:2]
    assert sampled[0] >= 0.99 * fine[0] and sampled[1] >= 0.99 * fine[1], (sampled, fine)


def _walk_command(axis, stroke):
    """step_x, step_y and step_angle of a walk at the stroke along the envelope axis."""
    return {"forward": (0.0, stroke, 0.0), "lateral": (stroke, 0.0, 0.0), "yaw": (0.0, 0.0, stroke)}[axis]


@pytest.mark.parametrize("stance", STANCES)
@pytest.mark.parametrize("axis, stroke", [("forward", 40.0), ("lateral", 30.0), ("yaw", 0.2)])
def test_the_sampled_cycle_is_where_the_port_walks_the_feet(stance, axis, stroke):
    """The clearance is judged on every classic timing's cycle, sampled at CYCLE_PHASES; each sample is the feet the
    port's own generate_feet places at that phase, in the stance, at the posture's lift."""
    lift, zm, feet_scale = ge.posture(0.25, 0.0, -0.5)
    feet = fg.stance_feet(stance, feet_scale)
    cycle = ge.cycle_feet(axis, stroke, lift, feet)
    assert cycle.shape == (len(ge.GAIT_NAMES), ge.CYCLE_PHASES, 6, 3)
    for gait_type in range(len(ge.GAIT_NAMES)):
        gait = fg.GaitState(gait_type=gait_type)
        fg.set_gait(gait)
        gait.step_height = lift
        gait.step_x, gait.step_y, gait.step_angle = _walk_command(axis, stroke)
        controller = fg.GaitController()
        controller.default_position = feet.copy()
        controller.target_default_position = feet.copy()
        for k in range(ge.CYCLE_PHASES):
            body = fg.BodyState(zm=zm)
            controller.set_phase(k / ge.CYCLE_PHASES)
            controller.generate_feet(gait, body)
            np.testing.assert_allclose(cycle[gait_type, k], body.feet[:, :3], rtol=0, atol=1e-9,
                                       err_msg=f"{ge.GAIT_NAMES[gait_type]} phase {k}")


def test_the_level_ik_is_the_port_s():
    lift, zm, feet_scale = ge.posture(0.5, -0.25, 0.25)
    cycle = ge.cycle_feet("yaw", 0.3, lift, fg.stance_feet(fg.STANCE_CRAB, feet_scale))
    angles = ge.level_inverse_kinematics(cycle, zm)
    kin = fg.Kinematics()
    for gait_type, phase in [(0, 0), (1, 17), (2, 64), (3, 119)]:
        feet = np.column_stack([cycle[gait_type, phase], np.ones(6)])
        expected = kin.inverse_kinematics(fg.BodyState(zm=zm, feet=feet), degrees=False).reshape(6, 3)
        np.testing.assert_allclose(angles[gait_type, phase], expected, rtol=0, atol=1e-12)
    body6 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, zm])
    feet = np.column_stack([cycle[0, 0], np.ones(6)])
    for reach in (0.5, 1.0, 1.6):
        stretched = feet.copy()
        stretched[:, :2] = fg.MOUNT_X[:, None] * [1, 0] + fg.MOUNT_Y[:, None] * [0, 1] + reach * (
            feet[:, :2] - np.column_stack([fg.MOUNT_X, fg.MOUNT_Y]))
        expected = [foot_reachable(kin, body6, np.zeros(3), leg, stretched) for leg in range(6)]
        np.testing.assert_array_equal(ge.level_reachable(stretched[:, :3], zm), expected)


def test_a_stride_that_brings_two_legs_within_the_clearance_is_not_walkable(monkeypatch):
    """A stride is feasible only if every classic timing's cycle keeps every leg pair LEG_CLEARANCE_MM apart, with
    CLEARANCE_RESERVE_MM to spare. With the two set just above the clearance a stroke keeps, within travel and reach,
    that stroke is no longer feasible, while a shorter one that keeps more is."""
    lift, zm, feet_scale = ge.posture(0.0, 0.0, 0.0)
    feet = fg.stance_feet(fg.STANCE_DEFAULT, feet_scale)
    long_stroke, short_stroke = 80.0, 8.0
    kept = ge.cycle_clearance("forward", long_stroke, lift, zm, feet)
    assert ge.geometry_factors("forward", long_stroke, lift, zm, feet_scale)[2]
    assert ge.cycle_clearance("forward", short_stroke, lift, zm, feet) > kept + 0.5
    monkeypatch.setattr(ge, "LEG_CLEARANCE_MM", kept - ge.CLEARANCE_RESERVE_MM + 0.1)
    assert not ge.geometry_factors("forward", long_stroke, lift, zm, feet_scale)[2]
    assert ge.geometry_factors("forward", short_stroke, lift, zm, feet_scale)[2]


def test_the_cycle_clearance_is_the_check_the_firmware_ports():
    """cycle_clearance is the least of clearance.leg_clearance, the firmware's legClearance, over the sampled cycle, on
    the angles the servos reach."""
    lift, zm, feet_scale = ge.posture(0.0, 0.0, 0.0)
    feet = fg.stance_feet(fg.STANCE_DEFAULT, feet_scale)
    angles = ge.level_inverse_kinematics(ge.cycle_feet("forward", 80.0, lift, feet), zm)
    limit = np.radians(JOINT_LIMIT_DEG)
    least = min(clearance.leg_clearance(np.degrees(np.clip(pose, -limit, limit))) for pose in angles.reshape(-1, 6, 3))
    assert ge.cycle_clearance("forward", 80.0, lift, zm, feet) == pytest.approx(least, abs=1e-9)


def test_a_stride_that_brings_the_body_centre_within_the_static_margin_is_not_feasible():
    """Every statically stable timing's cycle keeps the body centre STATIC_MARGIN_MM inside the support of its feet in
    stance. The crab stands its feet out to the sides, so a forward stride soon brings the support's front or rear edge
    near the centre: at the neutral posture 40 mm, within travel, reach and the legs' clearance, leaves 20 mm and is not
    feasible, while 20 mm leaves 29 mm and is."""
    lift, zm, feet_scale = ge.posture(0.0, 0.0, 0.0)
    feet = fg.stance_feet(fg.STANCE_CRAB, feet_scale)
    long_stroke, short_stroke = 40.0, 20.0
    assert ge.kappas("forward", long_stroke, lift, zm, feet)[2]
    assert ge.cycle_clearance("forward", long_stroke, lift, zm, feet) >= ge.LEG_CLEARANCE_MM + ge.CLEARANCE_RESERVE_MM
    assert ge.cycle_static_margin("forward", long_stroke, lift, feet) == pytest.approx(20.1, abs=0.1)
    assert ge.cycle_static_margin("forward", short_stroke, lift, feet) == pytest.approx(29.3, abs=0.1)
    assert not ge.geometry_factors("forward", long_stroke, lift, zm, feet_scale, fg.STANCE_CRAB)[2]
    assert ge.geometry_factors("forward", short_stroke, lift, zm, feet_scale, fg.STANCE_CRAB)[2]


@pytest.mark.parametrize("stance", STANCES)
@pytest.mark.parametrize("axis, stroke", [("forward", 30.0), ("lateral", 60.0), ("yaw", 0.3)])
def test_the_static_margin_is_the_one_compare_stances_reports(stance, axis, stroke):
    """cycle_static_margin is the least of compare_stances.support_margin, with the centre of mass at the body centre,
    over the feet in stance of each statically stable timing's sampled cycle, the timings compare_stances judges."""
    assert tuple(getattr(fg, ge.GAIT_NAMES[t]) for t in ge.STATICALLY_STABLE) == compare_stances.STATICALLY_STABLE
    lift, _, feet_scale = ge.posture(0.0, 0.0, 0.0)
    feet = fg.stance_feet(stance, feet_scale)
    placed = ge.cycle_feet(axis, stroke, lift, feet)
    least = math.inf
    for t in ge.STATICALLY_STABLE:
        standing = ge.cycle_phases(ge.TIMINGS[t][0]) < ge.TIMINGS[t][1]
        for phase in range(ge.CYCLE_PHASES):
            on_floor = placed[t, phase][standing[phase]]
            on_floor[:, 2] = 0.0  # compare_stances takes the feet within TOUCH_MM of the lowest as the support
            least = min(least, compare_stances.support_margin(np.zeros(2), on_floor))
    assert ge.cycle_static_margin(axis, stroke, lift, feet) == pytest.approx(least, abs=1e-9)


def test_feet_on_one_line_hold_no_static_margin():
    """Two feet, or three in a row, enclose no area, so no centre is inside their support."""
    feet = np.array([[-100.0, -10.0], [100.0, -10.0], [0.0, -10.0], [0.0, 90.0], [50.0, 40.0], [-50.0, 40.0]])
    two = np.array([True, True, False, False, False, False])
    in_a_row = np.array([True, True, True, False, False, False])
    triangle = np.array([True, True, False, True, False, False])
    assert ge.support_margins(feet, two) == pytest.approx(-10.0)
    assert ge.support_margins(feet, in_a_row) == pytest.approx(-10.0)
    assert ge.support_margins(feet, triangle) == pytest.approx(10.0)


def test_the_crab_is_walkable_by_its_sideways_stroke():
    """The crab walks sideways: a posture is walkable in it when it can stride WALKABLE_STROKE_MM laterally, whatever
    it strides forward."""
    assert ge.WALK_AXIS == {fg.STANCE_DEFAULT: "forward", fg.STANCE_CRAB: "lateral"}
    assert ge.is_walkable(0.0, 0.0, 0.0, fg.STANCE_CRAB)
    assert ge.longest_stroke("lateral", 0.0, 0.0, 0.0, fg.STANCE_CRAB) >= ge.WALKABLE_STROKE_MM


def top_speed(gait_type, axis, s1, h, fd, servo_rad_s, stance=fg.STANCE_DEFAULT):
    """The reference sweep: (v_max, stroke), the fastest body speed over the swept feasible strokes (mm/s, or rad/s
    for yaw) and the stroke that reaches it; (0, 0) when no stroke is feasible."""
    d = ge.STAND_FRAC[gait_type]
    best = (0.0, 0.0)
    for stroke, kappa_swing, kappa_stance in ge._feasible_strokes(axis, float(s1), float(h), float(fd), stance):
        cadence = min(servo_rad_s / max(kappa_swing / (1.0 - d), kappa_stance / d), ge.CADENCE_MAX)
        speed = stroke * cadence / d
        if speed > best[0]:
            best = (speed, stroke)
    return best


def test_default_posture_matches_the_reference_sweep():
    forward = lambda gait: top_speed(gait, "forward", 0, 0, 0, 13.0)[0]
    assert forward(fg.TRI_GATE) == pytest.approx(334, rel=0.05)
    assert forward(fg.BI_GATE) == pytest.approx(643, rel=0.05)
    assert forward(fg.WAVE) == pytest.approx(71, rel=0.10)


@pytest.mark.parametrize("stance", STANCES)
def test_neutral_posture_is_walkable(stance):
    assert ge.is_walkable(0, 0, 0, stance)


def test_known_unwalkable_postures():
    assert not ge.is_walkable(0, 1, -1), "tall body on close feet needs 111% tibia travel standing"
    assert not ge.is_walkable(0, -1, 1), "low body on wide feet is out of reach"


def test_nothing_feasible_returns_zero():
    assert top_speed(fg.TRI_GATE, "forward", 0, -1, 1, 13.0) == (0.0, 0.0)


def test_top_speed_scales_with_servo_speed_when_servo_bound():
    fast = top_speed(fg.WAVE, "forward", 0, 0, 0, 13.0)[0]
    slow = top_speed(fg.WAVE, "forward", 0, 0, 0, 6.5)[0]
    assert slow == pytest.approx(fast / 2, rel=0.01)


def _stand_frac(gait_type):
    gait = fg.GaitState(gait_type=gait_type)
    fg.set_gait(gait)
    return gait.stand_frac


def test_cadence_cap_binds_when_servos_are_fast():
    v_max, stroke = top_speed(fg.WAVE, "forward", 0, 0, 0, 1000.0)
    assert v_max == pytest.approx(stroke * ge.CADENCE_MAX / _stand_frac(fg.WAVE), rel=1e-9)


def test_stroke_grows_the_joint_rate_and_the_travel():
    short = ge.geometry_factors("forward", 20.0, 20.0, 0.0, 1.0)
    long = ge.geometry_factors("forward", 60.0, 20.0, 0.0, 1.0)
    assert long[0] > short[0] and long[1] > short[1]
    assert not ge.geometry_factors("forward", 400.0, 20.0, 0.0, 1.0)[2]


FIRMWARE_DT = 0.005
FULL_STICKS = {"forward": (0.0, 1.0, 0.0), "lateral": (1.0, 0.0, 0.0), "yaw": (0.0, 0.0, 1.0)}


def _walkable_postures(count, seed, stance):
    rng = np.random.default_rng(seed)
    postures = []
    while len(postures) < count:
        posture = tuple(rng.uniform(-1.0, 1.0, 3))
        if fg.posture_walkable(*posture, stance):
            postures.append(posture)
    return postures


def _worst_excursion(gait_type, stick, s1, h, fd, stride, stance):
    """The port's gait at the firmware's tick from a stand in the posture and stance, the command eased in as
    MotionService does and then held at the top speed for two cycles: the largest |joint| / JOINT_LIMIT_DEG, whether
    every foot stayed in reach, the least leg-pair clearance on the angles the servos reach, and the least static margin
    of the body centre over the feet in stance (inf for BI_GATE, which has none). None when the posture cannot walk the
    axis."""
    target = fg.GaitState(gait_type=gait_type)
    fg.set_gait(target)
    zm = fg.command_to_walk_gait(*stick, 1.0, s1, target, h, fd, stride, stance)
    if target.phase_rate <= 0.0:
        return None
    gait = fg.GaitState(**{**target.__dict__, "offset": target.offset.copy()})
    gait.step_x = gait.step_y = gait.step_angle = gait.phase_rate = 0.0
    feet = fg.stance_feet(stance, ge.posture(s1, h, fd)[2])
    controller = fg.GaitController()
    controller.default_position = feet.copy()
    controller.target_default_position = feet.copy()
    body, kin = fg.BodyState(zm=zm, feet=feet.copy()), fg.Kinematics()
    body6 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, zm])
    reachable, walked, margin = True, [], math.inf
    ramp_s = 1.0
    for _ in range(int(np.ceil((ramp_s + 2.0 / target.phase_rate) / FIRMWARE_DT))):
        fg.approach_gait_command(gait, target, FIRMWARE_DT)
        controller.step(gait, body, FIRMWARE_DT)
        walked.append(kin.inverse_kinematics(body).reshape(6, 3))
        reachable = reachable and all(foot_reachable(kin, body6, np.zeros(3), leg, body.feet) for leg in range(6))
        if gait_type in ge.STATICALLY_STABLE:
            standing = (controller.phase + gait.offset) % 1.0 < gait.stand_frac
            margin = min(margin, float(ge.support_margins(body.feet[:, :2], standing)))
    walked = np.array(walked)
    worst = float((np.abs(walked) / JOINT_LIMIT_DEG).max())
    chains = clearance.leg_chains(clearance.KIN, np.radians(np.clip(walked, -JOINT_LIMIT_DEG, JOINT_LIMIT_DEG)))
    pairs = np.array(clearance.NEIGHBOUR_PAIRS)
    least = float(clearance.surface_clearance(
        clearance.segment_pair_distances(chains[:, pairs[:, 0]], chains[:, pairs[:, 1]])).min())
    return worst, reachable, least, margin


@pytest.fixture(scope="module")
def full_stick_walks():
    """{(stance, gait, axis, stride, posture): (worst, reachable, least clearance, least static margin)} over 40
    walkable postures of each
    stance, at the shortest, the speed-optimal and the longest stride. Between the grid points the envelope is
    interpolated, so a stroke read inside a walkable cell has to hold for the posture actually walked, not only at the
    corners it was measured at."""
    walks = {}
    for stance in STANCES:
        for posture in _walkable_postures(40, 4, stance):
            for gait_type, name in enumerate(ge.GAIT_NAMES):
                for axis, stick in FULL_STICKS.items():
                    for stride in (-1.0, 0.0, 1.0):
                        excursion = _worst_excursion(gait_type, stick, *posture, stride, stance)
                        assert excursion is not None or axis != ge.WALK_AXIS[stance], (
                            f"walkable posture {posture} cannot walk {axis} in stance {stance}")
                        if excursion is not None:
                            walks[stance, name, axis, stride, tuple(round(v, 3) for v in posture)] = excursion
    walked = {axis: sum(key[0] == fg.STANCE_DEFAULT and key[2] == axis for key in walks) for axis in FULL_STICKS}
    assert walked["lateral"] >= 120 and walked["yaw"] >= 120, walked
    return walks


def test_walkable_postures_walk_full_stick_in_reach(full_stick_walks):
    assert [where for where, (_, reachable, _, _) in full_stick_walks.items() if not reachable] == []


def test_walkable_postures_walk_full_stick_in_travel(full_stick_walks):
    beyond = {where: round(worst, 4) for where, (worst, _, _, _) in full_stick_walks.items() if worst > 1.0}
    assert beyond == {}


def test_walkable_postures_walk_full_stick_clear_of_the_other_legs(full_stick_walks):
    touching = {where: round(least, 2) for where, (_, _, least, _) in full_stick_walks.items()
                if least < clearance.LEG_CLEARANCE_MM}
    assert touching == {}


def test_walkable_postures_walk_full_stick_with_the_static_margin(full_stick_walks):
    near_an_edge = {where: round(margin, 2) for where, (_, _, _, margin) in full_stick_walks.items()
                    if margin < ge.STATIC_MARGIN_MM}
    assert near_an_edge == {}


@pytest.mark.parametrize("stance", STANCES)
def test_table_holds_the_geometry_per_axis(table, stance):
    tables = _tables(table, stance)
    walkable = np.array(tables["walkable"], dtype=bool)
    assert "gaits" not in table, "the per-gait speeds are derived from the geometry, not tabulated"
    assert table["stand_frac"] == pytest.approx(ge.STAND_FRAC)
    assert table["leg_clearance_mm"] == clearance.LEG_CLEARANCE_MM
    assert tables["walk_axis"] == ge.WALK_AXIS[stance]
    for key in ("stroke_max", "kappa_swing", "kappa_stance"):
        assert set(tables[key]) == set(ge.AXES)
    for axis in ge.AXES:
        stroke_max = np.array(tables["stroke_max"][axis])
        assert stroke_max.shape == (GRID_N,) * 3
        for key in ("kappa_swing", "kappa_stance"):
            assert np.array(tables[key][axis]).shape == (GRID_N,) * 3 + (ge.STROKE_SAMPLES,)
        if stance == fg.STANCE_DEFAULT:
            assert (stroke_max[walkable] > 0).all(), "a default-stance posture that walks strides every axis"
    walk_stroke = np.array(tables["stroke_max"][ge.WALK_AXIS[stance]])
    np.testing.assert_array_equal(walkable, walk_stroke >= ge.WALKABLE_STROKE_MM)


def test_the_crab_table_judges_a_posture_by_its_sideways_stroke_not_its_forward_one(table):
    """Postures where the crab could stride forward but not sideways do not walk in it."""
    crab = _tables(table, fg.STANCE_CRAB)
    walkable = np.array(crab["walkable"], dtype=bool)
    forward = np.array(crab["stroke_max"]["forward"])
    lateral = np.array(crab["stroke_max"]["lateral"])
    assert walkable.any()
    strides_forward_alone = (forward >= ge.WALKABLE_STROKE_MM) & (lateral < ge.WALKABLE_STROKE_MM)
    assert strides_forward_alone.any() and not walkable[strides_forward_alone].any()


@pytest.mark.parametrize("stance", STANCES)
def test_the_geometry_and_not_the_sweep_ends_every_stroke(table, stance):
    for axis in ge.AXES:
        assert np.max(_tables(table, stance)["stroke_max"][axis]) < ge.STROKES[axis][-1], axis


@pytest.mark.parametrize("stance", STANCES)
@pytest.mark.parametrize("axis", ge.AXES)
def test_a_cell_with_no_feasible_stroke_holds_no_kappa(table, axis, stance):
    tables = _tables(table, stance)
    stroke_max = np.array(tables["stroke_max"][axis])
    infeasible = stroke_max == 0
    assert infeasible.any(), "the grid must reach postures that cannot stride the axis"
    for key in ("kappa_swing", "kappa_stance"):
        kappa = np.array(tables[key][axis])
        assert (kappa[infeasible] == 0).all()
        assert (kappa[~infeasible] > 0).all()


def _speed_optimal_cells(table, stance, count, seed):
    """The default posture and `count` walkable grid cells drawn at random."""
    cells = np.argwhere(np.array(_tables(table, stance)["walkable"], dtype=bool))
    picked = cells[np.random.default_rng(seed).choice(len(cells), count, replace=False)]
    return [(MID, MID, MID)] + [tuple(int(i) for i in cell) for cell in picked]


@pytest.mark.parametrize("stance", STANCES)
@pytest.mark.parametrize("axis", ge.AXES)
def test_speed_optimal_stroke_matches_the_reference_sweep(table, axis, stance):
    """The table samples the refined longest stroke at tenths and the reference sweeps fixed steps, so neither finds
    the other's optimum exactly. Kappa mostly rises with the stroke, so a shorter stroke steps at least as fast: the best
    sample is at least the reference's speed scaled down to the sample just short of the reference's stroke, less what
    a dip in kappa costs, and at most the reference's speed scaled up from the swept stroke just short of its own.
    A cell whose longest stroke is shorter than the sweep's first step has no reference to compare."""
    sweep_step = ge.STROKES[axis][0]
    compared = 0
    for cell in _speed_optimal_cells(table, stance, 30, seed=11):
        posture = tuple(float(ge.GRID[i]) for i in cell)
        stroke_max = _tables(table, stance)["stroke_max"][axis][cell[0]][cell[1]][cell[2]]
        if stroke_max < sweep_step:
            continue
        strokes = _sample_strokes(stroke_max)
        for gait_type in range(len(ge.GAIT_NAMES)):
            reference, reference_stroke = top_speed(gait_type, axis, *posture, table["servo_rad_s"], stance)
            sampled, stroke = _best_sample(table, stance, gait_type, axis, cell)
            shorter = strokes[strokes <= reference_stroke]
            swept = np.floor(stroke / sweep_step) * sweep_step
            where = (axis, gait_type, cell, sampled, reference, stroke, reference_stroke)
            assert len(shorter) > 0 and swept > 0, where
            assert sampled >= reference * shorter[-1] / reference_stroke * (1 - KAPPA_DIP[stance][axis]), where
            assert sampled <= reference * stroke / swept * (1 + SPEED_ROUNDING), where
        compared += 1
    assert compared >= 10, f"{compared} cells stride the axis far enough to compare"


@pytest.mark.parametrize("stance, axis", [(fg.STANCE_DEFAULT, "forward"), (fg.STANCE_DEFAULT, "lateral"),
                                          (fg.STANCE_CRAB, "lateral"), (fg.STANCE_CRAB, "yaw")])
def test_kappa_rises_with_stroke(table, axis, stance):
    """Between the samples kappa is interpolated, so it is the rise of kappa that keeps a lookup below a sample safe.
    The axes that turn the coxas furthest do not rise: the default stance's yaw, whose swing kappa falls up to 18% from
    its peak in 59% of its cells, and the crab's forward, up to 5.1% in 62%; the test below bounds that where it matters,
    and the firmware reads the running maximum."""
    tables = _tables(table, stance)
    strides = np.array(tables["walkable"], dtype=bool) & (np.array(tables["stroke_max"][axis]) > 0)
    rises = np.ones(strides.shape, dtype=bool)
    for key in ("kappa_swing", "kappa_stance"):
        kappa = np.array(tables[key][axis])
        fall = (np.maximum.accumulate(kappa, axis=-1) - kappa) / np.maximum(kappa, 1e-9)
        rises &= (fall <= KAPPA_FALL_TOL).all(axis=-1)
    exceptions = [tuple(int(i) for i in cell) for cell in np.argwhere(strides & ~rises)]
    assert rises[strides].mean() >= MONOTONE_SHARE, f"{axis}: {len(exceptions)} cells fall, e.g. {exceptions[:8]}"


@pytest.mark.parametrize("stance", STANCES)
@pytest.mark.parametrize("axis", ge.AXES)
def test_first_sample_bounds_the_kappa_of_shorter_strokes(table, axis, stance):
    """Below its first sample the firmware holds that sample's kappa, so a shorter stroke must not exceed it."""
    rng = np.random.default_rng(7)
    tables = _tables(table, stance)
    cells = np.argwhere(np.array(tables["walkable"], dtype=bool) & (np.array(tables["stroke_max"][axis]) > 0))
    for i, j, k in cells[rng.choice(len(cells), min(30, len(cells)), replace=False)]:
        lift, zm, feet_scale = ge.posture(*(float(ge.GRID[c]) for c in (i, j, k)))
        feet = fg.stance_feet(stance, feet_scale)
        first = _sample_strokes(tables["stroke_max"][axis][i][j][k])[0]
        bounds = (tables["kappa_swing"][axis][i][j][k][0], tables["kappa_stance"][axis][i][j][k][0])
        for share in (0.05, 0.25, 0.5, 0.75):
            kappa = ge.kappas(axis, float(first * share), lift, zm, feet)[:2]
            assert kappa[0] <= bounds[0] * (1 + BELOW_SAMPLE_TOL) and kappa[1] <= bounds[1] * (1 + BELOW_SAMPLE_TOL), (
                axis, (i, j, k), share, kappa, bounds)


@pytest.mark.parametrize("stance, axis, cell", [
    *((fg.STANCE_DEFAULT, axis, cell) for axis in ge.AXES for cell in ((MID, MID, MID), (1, 5, 3), (6, 2, 5))),
    (fg.STANCE_CRAB, "lateral", (MID, MID, MID)),
    (fg.STANCE_CRAB, "yaw", (MID, MID, MID)),
])
def test_longest_stroke_is_feasible_and_one_step_more_is_not(table, stance, axis, cell):
    stroke_max = _tables(table, stance)["stroke_max"][axis][cell[0]][cell[1]][cell[2]]
    lift, zm, feet_scale = ge.posture(*(float(ge.GRID[i]) for i in cell))
    assert 0 < stroke_max < ge.STROKES[axis][-1], "pick a posture whose reach ends inside the sweep"
    assert ge.geometry_factors(axis, stroke_max, lift, zm, feet_scale, stance)[2]
    assert not ge.geometry_factors(axis, stroke_max + ge.REFINED_STEP[axis], lift, zm, feet_scale, stance)[2]


def _cpp_array(text, name):
    declaration = re.search(rf"constexpr \w+ {name}\s*((?:\[\w+\])*)\s*=\s*([^;]*);", text)
    assert declaration, f"{name} missing from {HEADER}"
    sizes = re.findall(r"\[(\w+)\]", declaration.group(1))
    dims = [int(n) if n.isdigit() else int(_cpp_array(text, n)) for n in sizes]
    values = np.array([float(v) for v in re.findall(r"-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?", declaration.group(2))])
    return values.reshape(dims) if dims else values[0]


def test_header_and_json_carry_the_same_table(table):
    with open(HEADER) as f:
        text = f.read()

    assert _cpp_array(text, "GRID_N") == GRID_N == len(table["grid"])
    assert _cpp_array(text, "STROKE_SAMPLES") == ge.STROKE_SAMPLES
    assert _cpp_array(text, "STANCES") == len(fg.STANCE_NAMES) == len(table["stances"])
    assert _cpp_array(text, "SERVO_RAD_S") == pytest.approx(table["servo_rad_s"], abs=TOL)
    assert _cpp_array(text, "CADENCE_MAX") == pytest.approx(table["cadence_max"], abs=TOL)
    assert _cpp_array(text, "TRAVEL_MARGIN_DEG") == pytest.approx(table["travel_margin_deg"], abs=TOL)
    assert _cpp_array(text, "CLEARANCE_MM") == pytest.approx(table["leg_clearance_mm"], abs=TOL)
    assert _cpp_array(text, "STATIC_MARGIN_MM") == pytest.approx(table["static_margin_mm"], abs=TOL)
    assert table["static_margin_mm"] == ge.STATIC_MARGIN_MM
    # The capsules the clearance was judged with, which gait_speed.h holds to leg_clearance.h's.
    assert table["link_radius_mm"] == clearance.LINK_RADIUS_MM
    for segment, radius in clearance.LINK_RADIUS_MM.items():
        assert _cpp_array(text, f"{segment.upper()}_RADIUS_MM") == pytest.approx(radius, abs=TOL)
    assert _cpp_array(text, "DEFAULT_SPEED_FRACTION") == pytest.approx(table["default_speed_fraction"], abs=TOL)
    stances = [_tables(table, stance) for stance in range(len(fg.STANCE_NAMES))]
    assert np.array_equal(_cpp_array(text, "WALKABLE"), np.array([t["walkable"] for t in stances], dtype=float))

    assert _cpp_array(text, "STAND_FRAC") == pytest.approx(table["stand_frac"], abs=TOL)
    for retired in ("V_MAX", "STROKE"):
        assert not re.search(rf"\b{retired}\[", text), f"{retired} is no longer part of the table"
    for header_name, key in (("STROKE_MAX", "stroke_max"), ("KAPPA_SWING", "kappa_swing"),
                             ("KAPPA_STANCE", "kappa_stance")):
        expected = np.array([[t[key][axis] for axis in ge.AXES] for t in stances])
        assert expected.shape[:5] == (len(stances), 3) + (GRID_N,) * 3
        assert np.abs(_cpp_array(text, header_name) - expected).max() <= TOL


@pytest.mark.parametrize("stance", STANCES)
@pytest.mark.parametrize("cell", [(MID, MID, MID), (0, 0, 0), (GRID_N - 1, 2, 6), (2, GRID_N - 1, 1), (5, 1, GRID_N - 1)])
def test_committed_table_agrees_with_the_generator(table, cell, stance):
    s1, h, fd = (float(ge.GRID[i]) for i in cell)
    i, j, k = cell
    tables = _tables(table, stance)
    assert tables["walkable"][i][j][k] == ge.is_walkable(s1, h, fd, stance)
    lift, zm, feet_scale = ge.posture(s1, h, fd)
    feet = fg.stance_feet(stance, feet_scale)
    for axis in ge.AXES:
        stroke_max = ge.longest_stroke(axis, s1, h, fd, stance)
        assert tables["stroke_max"][axis][i][j][k] == pytest.approx(stroke_max, abs=TOL)
        for sample, stroke in enumerate(_sample_strokes(stroke_max)):
            kappa = ge.kappas(axis, float(stroke), lift, zm, feet)[:2] if stroke_max > 0 else (0.0, 0.0)
            assert tables["kappa_swing"][axis][i][j][k][sample] == pytest.approx(kappa[0], abs=TOL)
            assert tables["kappa_stance"][axis][i][j][k][sample] == pytest.approx(kappa[1], abs=TOL)
