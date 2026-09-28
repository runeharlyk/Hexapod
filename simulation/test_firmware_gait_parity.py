"""Check that the firmware gait constants and the simulation's port of them still agree.

`gait_tuned.h` is generated, but nothing stops someone editing it, or re-searching the library
without re-exporting. Either way the robot would walk a gait the sim never measured, which is
exactly the failure mode the repo's "keep firmware and sim in sync" rule exists to prevent.
The same holds for the hand-ported constants of `firmware_gait.py`, which mirror `gait.h`,
`motion.h` and `kinematics.h`.

As a pytest module it checks:
  * gait_tuned.h against its gait_library.json entry, including the C++ velocity_to_gait port;
  * firmware_gait.py against constants parsed out of the firmware headers;
  * firmware_gait.py's TUNED command mapping against the header's own formula;
  * the nominal-stance IK against the properties the firmware host test pins.

As a script it runs the header check only:
  python test_firmware_gait_parity.py            # checks the key recorded in the header
  python test_firmware_gait_parity.py --key flat_tripod
"""

import argparse
import json
import os
import re
import sys

import numpy as np
import pytest

from src.envs.hexapod_mj_env import PG_HEIGHT, PG_PHASE_RATE, PG_STEP_ANGLE, PG_STEP_XY
from src.robot import firmware_gait as fg
from src.robot.gait_schedule import GaitSchedule

FIRMWARE_INCLUDE = os.path.join(os.path.dirname(__file__), "..", "firmware", "include")
HDR = os.path.join(FIRMWARE_INCLUDE, "gait_tuned.h")
GAIT_H = os.path.join(FIRMWARE_INCLUDE, "gait.h")
MOTION_H = os.path.join(FIRMWARE_INCLUDE, "motion.h")
KINEMATICS_H = os.path.join(FIRMWARE_INCLUDE, "kinematics.h")
LIB = os.path.join(os.path.dirname(__file__), "src", "resources", "gait_library.json")
TOL = 1e-4

PARITY_COMMANDS = [(0.15, 0, 0), (0.30, 0, 0), (0.42, 0, 0), (-0.20, 0, 0),
                   (0.10, 0.10, 0), (0.05, 0, -1.0), (0.0, 0, 0.5), (0.12, 0, 0.85)]


# ------------------------------------------------------------------ C++ source parsing
def _read(path):
    with open(path) as f:
        return f.read()


def _cpp_value(expr):
    """Evaluates a constant C++ arithmetic expression such as `2.0f / 3.0f` or `3.1 / 6`."""
    expr = re.sub(r"([\d.])[fF]\b", r"\1", expr.strip())
    if not re.fullmatch(r"[\d.eE+\-*/() ]+", expr):
        raise ValueError(f"not a constant expression: {expr!r}")
    return float(eval(expr, {"__builtins__": {}}))


def _cpp_list(body):
    return [_cpp_value(v) for v in body.split(",") if v.strip()]


def _constexpr(text, name):
    m = re.search(rf"\b{name}\s*=\s*([^;,]+)[;,]", text)
    if not m:
        raise KeyError(f"{name} not found")
    return _cpp_value(m.group(1))


def _tuned_stick_constants(gait_h):
    """The TUNED stick ranges are defined in motion.h and are moving to gait.h; accept either home."""
    text = gait_h + _read(MOTION_H)
    return {name: _constexpr(text, name) for name in
            ("TUNED_CMD_VX_MAX", "TUNED_CMD_VY_MAX", "TUNED_CMD_YAW_MAX", "TUNED_LIFT_TRIM_MM")}


def _array(text, name):
    m = re.search(rf"\b{name}\[[^\]]*\]\s*=\s*\{{([^}}]*)\}}", text)
    if not m:
        raise KeyError(f"{name} not found")
    return _cpp_list(m.group(1))


def _nested_array(text, name):
    m = re.search(rf"\b{name}\[6\]\[4\]\s*=\s*\{{(.*?)\}};", text, re.S)
    if not m:
        raise KeyError(f"{name} not found")
    return np.array([_cpp_list(row) for row in re.findall(r"\{([^{}]*)\}", m.group(1))])


def _set_gait_cases(text):
    """GaitType -> (offset[6], stand_frac) for every setGait case that assigns literals."""
    body = text[text.index("void setGait("):]
    cases = {}
    for m in re.finditer(r"case GaitType::(\w+):(.*?)break;", body, re.S):
        offsets = dict(re.findall(r"gait\.offset\[(\d)\]\s*=\s*([^;]+);", m.group(2)))
        frac = re.search(r"gait\.stand_frac\s*=\s*([^;]+);", m.group(2))
        if len(offsets) == 6 and frac and "tuned_gait" not in frac.group(1):
            cases[m.group(1)] = ([_cpp_value(offsets[str(i)]) for i in range(6)], _cpp_value(frac.group(1)))
    return cases


def parse_header(path):
    text = _read(path)
    out = {}
    # a `constexpr float` line may declare several comma-separated constants, so scan each such
    # line for every NAME = <float>f pair rather than assuming one (or two) per line
    for raw in text.splitlines():
        line = raw.split("//")[0]      # comments carry things like [-1,1] that look like arrays
        if not line.strip().startswith("constexpr float") or "[" in line:
            continue
        for m in re.finditer(r"(\w+)\s*=\s*(-?[\d.eE+-]+)f", line):
            out[m.group(1)] = float(m.group(2))
    arr = re.search(r"constexpr float OFFSET\[6\]\s*=\s*\{([^}]*)\}", text)
    out["OFFSET"] = [float(v.strip().rstrip("f")) for v in arr.group(1).split(",")]
    key = re.search(r"gait_library\.json key '([^']+)'", text)
    out["_key"] = key.group(1) if key else None
    # the normalized step height is embedded in the velocity_to_gait body, not a named constant
    nh = re.search(r"out\[3\] = (-?[\d.eE+-]+)f;", text)
    out["_step_height_norm"] = float(nh.group(1)) if nh else None
    return out


def header_velocity_to_gait(h, cmd):
    """The C++ tuned_gait::velocity_to_gait, evaluated from the header's parsed constants."""
    vx, vy, yaw = cmd
    speed = float(np.hypot(vx, vy))
    b = min(speed / h["BLEND_SPEED"], 1.0)
    gx = h["GX0"] + (h["GX1"] - h["GX0"]) * b
    gy = h["GY0"] + (h["GY1"] - h["GY0"]) * b
    gyaw = h["GYAW0"] + (h["GYAW1"] - h["GYAW0"]) * b

    def c1(v):
        return max(-1.0, min(1.0, v))

    return np.array([c1(vy / gy), c1(vx / gx), c1(yaw / gyaw + h["YAW_COMP"] * vx),
                     h["_step_height_norm"], b * 2 - 1,
                     c1(h["PR_BASE"] + h["PR_SLOPE"] * speed + h["PR_YAW"] * abs(yaw))])


# ------------------------------------------------------------------ header vs library
def header_library_mismatches(h, s):
    fails = []

    def chk(name, got, want):
        if got is None or abs(got - want) > TOL * max(1.0, abs(want)):
            fails.append(f"{name}: header {got} != library {want}")

    chk("STAND_FRAC", h.get("STAND_FRAC"), s.duty)
    for i, o in enumerate(s.offsets()):
        chk(f"OFFSET[{i}]", h["OFFSET"][i], float(o))
    chk("STEP_HEIGHT_MM", h.get("STEP_HEIGHT_MM"), float(np.interp(s.step_height, [-1, 1], PG_HEIGHT)))
    chk("STEP_DEPTH_MM", h.get("STEP_DEPTH_MM"), s.step_depth)
    chk("RIDE_MM", h.get("RIDE_MM"), s.ride_mm)
    for n, v in (("GX0", s.gx0), ("GX1", s.gx1), ("GY0", s.gy0), ("GY1", s.gy1),
                 ("GYAW0", s.gyaw0), ("GYAW1", s.gyaw1), ("BLEND_SPEED", s.blend_speed),
                 ("PR_BASE", s.pr_base), ("PR_SLOPE", s.pr_slope), ("PR_YAW", s.pr_yaw),
                 ("YAW_COMP", s.yaw_comp)):
        chk(n, h.get(n), v)
    # action scales the firmware decodes with must match the env the gait was searched in
    chk("STEP_XY_MM", h.get("STEP_XY_MM"), PG_STEP_XY)
    chk("STEP_ANGLE_RAD", h.get("STEP_ANGLE_RAD"), PG_STEP_ANGLE)
    chk("STEP_HEIGHT_MIN_MM", h.get("STEP_HEIGHT_MIN_MM"), PG_HEIGHT[0])
    chk("STEP_HEIGHT_MAX_MM", h.get("STEP_HEIGHT_MAX_MM"), PG_HEIGHT[1])
    chk("PHASE_RATE_MIN", h.get("PHASE_RATE_MIN"), PG_PHASE_RATE[0])
    chk("PHASE_RATE_MAX", h.get("PHASE_RATE_MAX"), PG_PHASE_RATE[1])
    chk("out[3] step height (normalized)", h.get("_step_height_norm"), s.step_height)
    return fails


def gait_action_parity(h, s):
    """Worst |diff| between the C++ velocity_to_gait port and GaitSchedule.gait_action."""
    return max(float(np.max(np.abs(header_velocity_to_gait(h, cmd) - s.gait_action(np.array(cmd)))))
               for cmd in PARITY_COMMANDS)


# ------------------------------------------------------------------ pytest
@pytest.fixture(scope="module")
def header():
    return parse_header(HDR)


@pytest.fixture(scope="module")
def gait_h():
    return _read(GAIT_H)


def _library_schedule(key):
    with open(LIB) as f:
        return GaitSchedule.from_dict(json.load(f)["gaits"][key]["params"])


def test_tuned_header_matches_its_library_entry(header):
    s = _library_schedule(header["_key"])
    assert header_library_mismatches(header, s) == []
    assert gait_action_parity(header, s) <= 1e-5


def test_sim_tuned_gait_is_the_one_the_header_ships(header):
    assert fg.TUNED_GAIT_KEY == header["_key"]
    tuned = fg.load_tuned_gait()
    np.testing.assert_allclose(tuned.offset, header["OFFSET"], atol=1e-6)
    assert tuned.stand_frac == pytest.approx(header["STAND_FRAC"], abs=1e-6)
    assert tuned.step_height_mm == pytest.approx(header["STEP_HEIGHT_MM"], abs=1e-4)
    assert tuned.step_depth_mm == pytest.approx(header["STEP_DEPTH_MM"], abs=1e-6)
    assert tuned.ride_mm == pytest.approx(header["RIDE_MM"], abs=1e-4)
    assert fg.TUNED_STEP_XY_MM == header["STEP_XY_MM"]
    assert fg.TUNED_STEP_ANGLE_RAD == header["STEP_ANGLE_RAD"]
    assert fg.TUNED_STEP_HEIGHT_MM == (header["STEP_HEIGHT_MIN_MM"], header["STEP_HEIGHT_MAX_MM"])
    assert fg.TUNED_PHASE_RATE == (header["PHASE_RATE_MIN"], header["PHASE_RATE_MAX"])


def test_gait_h_constants_match_the_port(gait_h):
    np.testing.assert_allclose(fg.DEFAULT_OFFSET, _array(gait_h, "default_offset"))
    assert fg.DEFAULT_STAND_FRAC == pytest.approx(_constexpr(gait_h, "default_stand_frac"))
    assert fg.STRIDE_DEADBAND_MM == _constexpr(gait_h, "GAIT_STRIDE_DEADBAND_MM")
    assert fg.YAW_DEADBAND_RAD == _constexpr(gait_h, "GAIT_YAW_DEADBAND_RAD")
    for name in ("GAIT_SNAP_MM", "GAIT_SNAP_RAD", "GAIT_SNAP_RATE", "GAIT_SNAP_DEPTH"):
        assert getattr(fg, name) == pytest.approx(_constexpr(gait_h, name)), name
    for name, value in _tuned_stick_constants(gait_h).items():
        assert getattr(fg, name) == pytest.approx(value), name
    assert fg.AUTO_RIPPLE_MAX == _constexpr(gait_h, "RIPPLE_MAX")
    assert fg.AUTO_TRI_MAX == _constexpr(gait_h, "TRI_MAX")
    assert fg.AUTO_HYSTERESIS == _constexpr(gait_h, "AUTO_HYSTERESIS")
    np.testing.assert_allclose(fg.BEZIER_STEPS, _array(gait_h, "BEZIER_STEPS"))
    np.testing.assert_allclose(fg.BEZIER_HEIGHTS, _array(gait_h, "BEZIER_HEIGHTS"))
    for name in ("defaultPosition", "targetDefaultPosition", "swingStartPosition"):
        np.testing.assert_allclose(fg.DEFAULT_FEET, _nested_array(gait_h, name), err_msg=name)


def test_set_gait_matches_every_firmware_case(gait_h):
    cases = _set_gait_cases(gait_h)
    assert set(cases) == {"TRI_GATE", "BI_GATE", "WAVE", "RIPPLE"}
    for name, (offset, stand_frac) in cases.items():
        gait = fg.GaitState(gait_type=getattr(fg, name))
        fg.set_gait(gait)
        np.testing.assert_allclose(gait.offset, offset, atol=1e-7, err_msg=name)
        assert gait.stand_frac == pytest.approx(stand_frac), name


def test_motion_h_constants_match_the_port():
    motion_h = _read(MOTION_H)
    assert fg.GAIT_COMMAND_TAU_S == _constexpr(motion_h, "GAIT_COMMAND_TAU_S")
    assert fg.DEFAULT_STEP_HEIGHT_MM == _constexpr(motion_h, "DEFAULT_STEP_HEIGHT_MM")
    assert fg.DEFAULT_STEP_DEPTH == _constexpr(motion_h, "DEFAULT_STEP_DEPTH")
    np.testing.assert_allclose(fg.DEFAULT_FEET, _nested_array(motion_h, "base_feet_pos"))


def test_kinematics_h_geometry_matches_the_port():
    kin_h = _read(KINEMATICS_H)
    m = re.search(r"hexapodConfig\s*=\s*\{(.*?)\};", kin_h, re.S)
    arrays = [_cpp_list(a) for a in re.findall(r"\{([^{}]*)\}", m.group(1))]
    scalars = _cpp_list(re.sub(r"\{[^{}]*\}", "", m.group(1)))
    np.testing.assert_allclose(fg.MOUNT_X, arrays[0])
    np.testing.assert_allclose(fg.MOUNT_Y, arrays[1])
    np.testing.assert_allclose(fg.MOUNT_ANGLE_DEG, arrays[2])
    assert [fg.ROOT_TO_J1, fg.J1_TO_J2, fg.J2_TO_J3, fg.J3_TO_TIP] == scalars


def test_tuned_command_mapping_follows_the_firmware_formula(header, gait_h):
    vx_max, vy_max, yaw_max, trim = _tuned_stick_constants(gait_h).values()
    for lx, ly, rx, s1 in [(0, 0.5, 0, 0), (0, 1, 0, 1), (-0.4, 0.2, 0.3, -1), (0, -0.6, -0.8, 0.5)]:
        gait = fg.GaitState(gait_type=fg.TUNED)
        zm = fg.command_to_walk_gait(lx, ly, rx, 0.0, s1, gait)
        a = header_velocity_to_gait(header, (ly * vx_max, -lx * vy_max, rx * yaw_max))
        assert gait.step_x == pytest.approx(a[0] * header["STEP_XY_MM"], abs=1e-3)
        assert gait.step_y == pytest.approx(a[1] * header["STEP_XY_MM"], abs=1e-3)
        assert gait.step_angle == pytest.approx(a[2] * header["STEP_ANGLE_RAD"], abs=1e-5)
        lift = np.clip(header["STEP_HEIGHT_MM"] + s1 * trim,
                       header["STEP_HEIGHT_MIN_MM"], header["STEP_HEIGHT_MAX_MM"])
        assert gait.step_height == pytest.approx(lift, abs=1e-3)
        rate = header["PHASE_RATE_MIN"] + (a[5] + 1) * 0.5 * (header["PHASE_RATE_MAX"] - header["PHASE_RATE_MIN"])
        assert gait.phase_rate == pytest.approx(rate, abs=1e-4)
        assert zm == pytest.approx(-header["RIDE_MM"], abs=1e-4)


def test_nominal_stance_ik_meets_the_firmware_host_test():
    """Mirrors test_inverse_kinematics_holds_the_nominal_stance and
    test_nominal_stance_is_inside_joint_travel in firmware/test/test_gait/test_gait.cpp; the C++
    tests pin properties, not literal angles, so the same properties are asserted here."""
    angles = fg.Kinematics().inverse_kinematics(fg.BodyState())
    assert np.all(np.isfinite(angles))
    servo_frame = angles + np.tile([0.0, 0.0, 90.0], 6)
    assert np.all(np.abs(servo_frame) <= 90.0)
    per_leg = angles.reshape(6, 3)
    for a, b in ((0, 2), (3, 5)):
        np.testing.assert_allclose(per_leg[a, 1:], per_leg[b, 1:], atol=0.01)
    limits = np.array(_array(_read(KINEMATICS_H), "JOINT_LIMIT_DEG"))
    assert np.all(np.abs(per_leg) <= limits)


# ------------------------------------------------------------------ behaviour, ported from test_gait.cpp
FIRMWARE_DT = 0.005


def _tripod():
    gait = fg.GaitState(gait_type=fg.TRI_GATE)
    fg.set_gait(gait)
    return gait


def _max_lift_while_repositioning(gc, gait, body, max_ticks):
    max_lift = np.zeros(6)
    ticks = 0
    while gc.has_pending_stance_change() and ticks < max_ticks:
        gc.step(gait, body, FIRMWARE_DT)
        max_lift = np.maximum(max_lift, body.feet[:, 2] - fg.DEFAULT_FEET[:, 2])
        ticks += 1
    return max_lift


def test_stance_change_walks_the_feet_to_the_new_target():
    gc, gait, body = fg.GaitController(), _tripod(), fg.BodyState()
    wider = fg.DEFAULT_FEET.copy()
    wider[:, :2] *= 1.25
    gc.set_default_foot_target(wider)
    assert gc.has_pending_stance_change()
    max_lift = _max_lift_while_repositioning(gc, gait, body, 2000)
    assert not gc.has_pending_stance_change(), "stance change never converged"
    assert np.all(max_lift > 1.0), "a foot slid to the new stance instead of stepping"


def test_settled_legs_hold_station_while_others_reposition():
    gc, gait, body = fg.GaitController(), _tripod(), fg.BodyState()
    target = fg.DEFAULT_FEET.copy()
    target[0, 0] += 30.0
    gc.set_default_foot_target(target)
    max_lift = _max_lift_while_repositioning(gc, gait, body, 4000)
    assert not gc.has_pending_stance_change(), "stance change never converged"
    assert max_lift[0] > 1.0, "the foot that had to move never stepped"
    assert np.all(max_lift[1:] < 1.0), "a leg already on target took an empty step"


def test_released_command_stops_the_cycle():
    cur = _tripod()
    cur.step_x, cur.step_y, cur.step_angle = -30.0, 80.0, 0.4
    target = fg.GaitState(**{**cur.__dict__, "offset": cur.offset.copy()})
    gc, body = fg.GaitController(), fg.BodyState()
    for _ in range(400):
        fg.approach_gait_command(cur, target, FIRMWARE_DT)
        gc.step(cur, body, FIRMWARE_DT)
    target.step_x = target.step_y = target.step_angle = 0.0
    for _ in range(600):
        fg.approach_gait_command(cur, target, FIRMWARE_DT)
        gc.step(cur, body, FIRMWARE_DT)
    assert cur.step_angle == 0.0, "yaw command must reach zero, not a subnormal"
    assert gc.phase == 0.0, "a released command must not keep the cycle running"
    np.testing.assert_allclose(body.feet[:, :3], fg.DEFAULT_FEET[:, :3], atol=0.5)


def test_command_ramp_is_independent_of_loop_rate():
    fine, coarse, target = fg.GaitState(), fg.GaitState(), fg.GaitState(step_y=100.0)
    for _ in range(20):
        fg.approach_gait_command(fine, target, 0.005)
    fg.approach_gait_command(coarse, target, 0.100)
    assert fine.step_y == pytest.approx(coarse.step_y, abs=0.5)
    assert 0.0 < fine.step_y < target.step_y


def test_command_ramp_switches_cadence_mode_outright():
    cur, target = fg.GaitState(phase_rate=1.2), fg.GaitState(phase_rate=0.0)
    fg.approach_gait_command(cur, target, FIRMWARE_DT)
    assert cur.phase_rate == 0.0, "leaving explicit cadence must be instant"
    target.phase_rate = 1.4
    fg.approach_gait_command(cur, target, FIRMWARE_DT)
    assert cur.phase_rate == 1.4, "entering explicit cadence must be instant"
    between = fg.GaitState(phase_rate=1.0)
    target.phase_rate = 2.0
    fg.approach_gait_command(between, target, FIRMWARE_DT)
    assert 1.0 < between.phase_rate < 1.2, "between two explicit rates the cadence should ease"


def test_explicit_phase_rate_overrides_the_stride_law():
    gait = _tripod()
    gait.step_y, gait.phase_rate = 100.0, 1.35
    gc = fg.GaitController()
    gc.advance_phase(gait, 0.1)
    assert gc.phase == pytest.approx(0.135)
    gait.phase_rate = 0.0
    gc.set_phase(0.0)
    gc.advance_phase(gait, 0.1)
    assert gc.phase == pytest.approx(0.1 * 1.5), "100 mm stride saturates the legacy law at 1.5"


def test_auto_gait_picks_by_speed_and_resists_flapping():
    assert fg.select_auto_gait(0.10, fg.RIPPLE) == fg.RIPPLE
    assert fg.select_auto_gait(0.50, fg.RIPPLE) == fg.TRI_GATE
    assert fg.select_auto_gait(0.90, fg.TRI_GATE) == fg.BI_GATE
    assert fg.select_auto_gait(0.32, fg.RIPPLE) == fg.RIPPLE
    assert fg.select_auto_gait(0.32, fg.TRI_GATE) == fg.TRI_GATE


# ------------------------------------------------------------------ CLI
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--header", default=HDR)
    ap.add_argument("--lib", default=LIB)
    ap.add_argument("--key", default=None)
    args = ap.parse_args()

    if not os.path.exists(args.header):
        raise SystemExit(f"missing {args.header}; run export_gait.py")
    h = parse_header(args.header)
    key = args.key or h["_key"]
    with open(args.lib) as f:
        lib = json.load(f)["gaits"]
    if key not in lib:
        raise SystemExit(f"header references '{key}', absent from {args.lib}")
    s = GaitSchedule.from_dict(lib[key]["params"])

    fails = header_library_mismatches(h, s)
    worst = gait_action_parity(h, s)
    if worst > 1e-5:
        fails.append(f"velocity_to_gait disagrees with GaitSchedule.gait_action by {worst:.2e}")

    print(f"header : {os.path.abspath(args.header)}")
    print(f"key    : {key}   (library score {lib[key].get('val_score', float('nan')):.3f})")
    print(f"gait_action parity: max |diff| = {worst:.2e} over {len(PARITY_COMMANDS)} commands")
    if fails:
        print(f"\nFAIL ({len(fails)}):")
        for f in fails:
            print("  -", f)
        print("\nre-run: python export_gait.py --key " + key)
        sys.exit(1)
    print("\nPASS - firmware header matches the searched gait")


if __name__ == "__main__":
    main()
