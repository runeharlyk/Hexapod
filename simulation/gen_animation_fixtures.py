"""Writes animations/fixtures/expected.json, expected.txt, validation.json, validation.txt and the fx_*.pb binaries from
the reference implementation.

The firmware and app parity tests read those files; test_animation_fixtures.py fails when they are stale.
    uv run python gen_animation_fixtures.py

expected.txt carries the same content as expected.json for the C++ test, which has no JSON parser.
It holds one record per line, with tokens separated by single spaces.
An empty parameter list is written as "-", and parameters as NAME=VALUE joined by ";".
    T <tolerance>
    E <animation> <params> <t> <mask> <a0> ... <a17>
    P <index> <animation> <params> <dt> <b0> ... <b5> <f0x> <f0y> <f0z> ... <f5z>
    V <step> stop
    V <step> play <animation> <params>
    S <state> <mask> <a0> ... <a17>
E rows are the evaluate samples in file order.
A P row opens a player case with its live pose (six body offsets, then six foot offsets).
The V rows that follow are its events, and the S rows that follow are its trace, one per step in step order.
Angles are degrees in IK order with six decimals; state is the State name.
The fx_*.pb files are the nanopb-decodable binaries of the fx_*.json animations.

Fixture contract for porters:
- evaluate cases: for each sample time t, pose_to_angles(evaluate(animation, params, t)).
- player cases: play(animation, params, live) once, then for each step: apply the event keyed by
  that step index (if any), then update(dt), then record the state name, angles and mask. An event
  applies before that step's update.
- a chained "play" event passes no live pose, so the entry starts from the player's last_pose.
- params are keyed by ParamId name; undeclared ids resolve to 1.
- angles are degrees in IK order: coxa, femur, tibia per leg, legs 0..5.
- mask bit leg * 3 + joint is set for a joint that hit its limit or was saturated by an unreachable foot.
- the fx_*.json values are protobuf float (32-bit): a port that parses the JSON rounds every number
  to float32 before use; dt, params and the live pose are used as written.
- states and masks must match exactly; angles within "tolerance" (degrees).

validation.json lists documents, each with the fragments that the first error validate() reports must all contain (none
for a valid document) and whether nanopb decodes it: a document past the schema's counts or string sizes fails to decode
on the robot, which rejects it before validate() runs. Each platform words its errors its own way, with or without
indices, so a fragment is the part all of them share. validation.txt carries the same cases for the C++ test, one per line:
    <case> <decodes 0|1> <hex of the encoded Animation, "-" when empty> <fragment>|<fragment>...   ("-" for none)
"""
import json
import math
from pathlib import Path

import numpy as np

from src.robot import animation as an
from google.protobuf import json_format

from src.robot.animation_files import from_proto, load_json, pb, to_proto
from src.robot.firmware_gait import BodyState, Kinematics

ROOT = Path(__file__).resolve().parents[1]
FIXTURE_DIR = ROOT / "animations" / "fixtures"
EXPECTED = FIXTURE_DIR / "expected.json"
EXPECTED_TXT = FIXTURE_DIR / "expected.txt"
VALIDATION = FIXTURE_DIR / "validation.json"
VALIDATION_TXT = FIXTURE_DIR / "validation.txt"
TOLERANCE = 1e-4
GRID_MARGIN_S = 1e-4

EVALUATE_CASES = [
    ("fx_mixed_legs", {}),
    ("fx_mixed_legs", {"FOOT_LIFT": 1.5, "BODY_ROLL": 0.5}),
    ("fx_overlay", {}),
    ("fx_overlay", {"OVERLAY_AMPLITUDE": 0.5}),
    ("fx_params", {}),
    ("fx_params", {"SPEED": 2.0, "BODY_X": 2.0, "BODY_Y": 0.5, "BODY_Z": 1.5, "BODY_ROLL": 0.0,
                   "BODY_PITCH": 2.0, "BODY_YAW": 0.5, "FOOT_LIFT": 2.0, "OVERLAY_AMPLITUDE": 0.25, "REPEAT": 3}),
    ("fx_single", {}),
]

DISPLACED_LIVE = {"body": [0.05, -0.03, 0.0, 8.0, -6.0, 12.0],
                  "feet": [[15.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, -10.0, 5.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [-5.0, 5.0, 0.0]]}

PLAYER_CASES = [
    {"animation": "fx_mixed_legs", "params": {}, "dt": 0.02, "live": DISPLACED_LIVE, "events": [], "steps": 170},
    {"animation": "fx_mixed_legs", "params": {}, "dt": 0.02, "live": DISPLACED_LIVE,
     "events": [{"step": 45, "action": "stop"}], "steps": 84},
    {"animation": "fx_mixed_legs", "params": {}, "dt": 0.02, "live": DISPLACED_LIVE,
     "events": [{"step": 40, "action": "play", "animation": "fx_overlay", "params": {"OVERLAY_AMPLITUDE": 1.5}}], "steps": 120},
    {"animation": "fx_single", "params": {}, "dt": 0.02, "live": DISPLACED_LIVE,
     "events": [{"step": 40, "action": "stop"}], "steps": 71},
    {"animation": "fx_params", "params": {"SPEED": 2.0, "REPEAT": 2}, "dt": 0.021, "live": DISPLACED_LIVE,
     "events": [], "steps": 105},
    {"animation": "fx_params", "params": {"REPEAT": 2.5, "SPEED": 2.0}, "dt": 0.021, "live": DISPLACED_LIVE,
     "events": [], "steps": 128},
    {"animation": "fx_overlay", "params": {}, "dt": 0.02, "live": DISPLACED_LIVE,
     "events": [{"step": 110, "action": "stop"}], "steps": 141},
]


def sample_times(anim: an.Animation) -> list[float]:
    times = [k.time for k in anim.keyframes]
    mids = [(a + b) / 2 for a, b in zip(times, times[1:])]
    return sorted(set(times + mids + [-0.1, anim.duration + 0.5, 0.137, 0.731]))


def assert_off_grid(anim: an.Animation, dt: float, speed: float, plays: int) -> None:
    """Fails when a transition the player's accumulated clock crosses lies within GRID_MARGIN_S of a
    multiple of dt, where float rounding alone decides the step it flips on and a correct float32 or
    float64 port could not match. Checked: the entry and exit blend ends, and (n * duration + e) / speed
    for every keyframe time and overlay window edge e and n in 0..plays-1 (the last play ends at
    e = duration). Values <= 0 are skipped:
    the clock starts at 0 and window starts are inclusive, so nothing is crossed there."""
    edges = [k.time for k in anim.keyframes] + [x for o in anim.overlays for x in (o.start, o.end)]
    values = [anim.entry_seconds(), anim.exit_seconds()]
    values += [(n * anim.duration + e) / speed for n in range(plays) for e in edges]
    for v in values:
        if v > 0.0 and abs(v - round(v / dt) * dt) < GRID_MARGIN_S:
            raise SystemExit(f"{anim.name}: transition at {v:.6f} s is on the {dt} s step grid")


def reachable_plays(anim: an.Animation, params: np.ndarray, dt: float, steps: int) -> int:
    """REPEAT plays, or for a loop the number of passes steps * dt of playing clock can cover."""
    if anim.loop:
        return math.ceil(steps * dt * params[an.ParamId.SPEED] / anim.duration) if anim.duration > 0.0 else 1
    return max(1, math.floor(params[an.ParamId.REPEAT] + 0.5))


def assert_samples_clear_of_edges(anim: an.Animation) -> None:
    """evaluate is a pure function of t, so a sample exactly on an overlay window edge is fine; one
    within GRID_MARGIN_S of it would flip on float rounding."""
    for t in sample_times(anim):
        for o in anim.overlays:
            for edge in (o.start, o.end):
                if t != edge and abs(t - edge) < GRID_MARGIN_S:
                    raise SystemExit(f"{anim.name}: sample {t} is within {GRID_MARGIN_S} s of window edge {edge}")


def assert_case_off_grid(case: dict, anims: dict[str, an.Animation]) -> None:
    played = [(case["animation"], case["params"])]
    played += [(e["animation"], e["params"]) for e in case["events"] if e["action"] == "play"]
    for name, values in played:
        a = anims[name]
        params = an.resolve_params(a, param_values(values))
        assert_off_grid(a, case["dt"], params[an.ParamId.SPEED], reachable_plays(a, params, case["dt"], case["steps"]))


def param_values(values: dict) -> dict[an.ParamId, float]:
    return {an.ParamId[k]: float(v) for k, v in values.items()}


def live_pose(live: dict) -> an.Pose:
    b = BodyState(omega=live["body"][0], phi=live["body"][1], psi=live["body"][2],
                  xm=live["body"][3], ym=live["body"][4], zm=live["body"][5])
    b.feet[:, :3] += np.array(live["feet"])
    return an.capture_pose(b)


def generate() -> dict:
    kin = Kinematics()
    anims = {p.stem: load_json(p) for p in sorted(FIXTURE_DIR.glob("fx_*.json"))}
    for name, a in anims.items():
        err = an.validate(a)
        if err:
            raise SystemExit(f"{name}: {err}")
        assert_samples_clear_of_edges(a)
    for case in PLAYER_CASES:
        assert_case_off_grid(case, anims)

    evaluate = []
    for name, values in EVALUATE_CASES:
        a = anims[name]
        params = an.resolve_params(a, param_values(values))
        samples = []
        for t in sample_times(a):
            angles, mask = an.pose_to_angles(an.evaluate(a, params, t, kin), kin)
            samples.append({"t": t, "angles": [round(float(x), 6) for x in angles], "mask": mask})
        evaluate.append({"animation": name, "params": values, "samples": samples})

    player = []
    for case in PLAYER_CASES:
        p = an.Player(kin)
        p.play(anims[case["animation"]], param_values(case["params"]), live_pose(case["live"]))
        events = {e["step"]: e for e in case["events"]}
        trace = []
        for step in range(case["steps"]):
            e = events.get(step)
            if e and e["action"] == "stop":
                p.stop()
            elif e and e["action"] == "play":
                p.play(anims[e["animation"]], param_values(e["params"]))
            angles, mask = an.pose_to_angles(p.update(case["dt"]), kin)
            trace.append({"state": p.state.name, "angles": [round(float(x), 6) for x in angles], "mask": mask})
        player.append({k: v for k, v in case.items() if k != "steps"} | {"trace": trace})

    return {"tolerance": TOLERANCE, "evaluate": evaluate, "player": player}


def _params_token(values: dict) -> str:
    return "-" if not values else ";".join(f"{k}={v}" for k, v in values.items())


def _floats(xs) -> str:
    return " ".join(f"{float(x):.6f}" for x in xs)


def text_dump(expected: dict) -> str:
    """The same content as expected.json in the line format firmware/test/test_animation parses."""
    lines = [f"T {expected['tolerance']}"]
    for case in expected["evaluate"]:
        for s in case["samples"]:
            lines.append(f"E {case['animation']} {_params_token(case['params'])} {s['t']} {s['mask']} {_floats(s['angles'])}")
    for index, case in enumerate(expected["player"]):
        live = case["live"]
        feet = [v for foot in live["feet"] for v in foot]
        lines.append(f"P {index} {case['animation']} {_params_token(case['params'])} {case['dt']} "
                     f"{_floats(live['body'])} {_floats(feet)}")
        for e in case["events"]:
            if e["action"] == "stop":
                lines.append(f"V {e['step']} stop")
            else:
                lines.append(f"V {e['step']} play {e['animation']} {_params_token(e['params'])}")
        for s in case["trace"]:
            lines.append(f"S {s['state']} {s['mask']} {_floats(s['angles'])}")
    return "\n".join(lines) + "\n"


def write_fixture_binaries() -> None:
    """The C++ parity test decodes these with nanopb, exactly as the robot decodes an upload."""
    from src.robot.animation_files import load_json, save_binary
    for path in sorted(FIXTURE_DIR.glob("fx_*.json")):
        save_binary(load_json(path), path.with_suffix(".pb"))


def _base() -> pb.Animation:
    return to_proto(an.Animation(name="t", keyframes=[an.Keyframe(0.0), an.Keyframe(1.0)]))


def _overlay(msg: pb.Animation, **fields) -> None:
    msg.overlays.add(**({"amplitude": 1, "frequency": 1, "start": 0, "end": 1} | fields))


def _param(msg: pb.Animation, id: int, min: float, default_value: float, max: float) -> None:
    msg.params.add(id=id, min=min, default_value=default_value, max=max)


def _legs(msg: pb.Animation, count: int) -> list:
    legs = [msg.keyframes[1].legs.add() for _ in range(count)]
    for leg in legs:
        leg.foot.SetInParent()
    return legs


# (case, change to a valid two-keyframe document, fragments of the first error). The order follows validate().
VALIDATION_CASES = [
    ("minimal", lambda m: None, []),
    ("description_of_96_utf8_bytes", lambda m: setattr(m, "description", "\u00e9" * 48), []),
    ("ten_params", lambda m: [_param(m, i, 1, 1, 2) for i in range(10)], []),
    ("schema", lambda m: setattr(m, "schema", 2), ["schema", "is not 1"]),
    ("name_charset", lambda m: setattr(m, "name", "Bad Name"), ["name must be 1-32 characters of [a-z0-9_-]"]),
    ("name_length", lambda m: setattr(m, "name", "x" * 33), ["name must be 1-32 characters of [a-z0-9_-]"]),
    ("description_of_97_utf8_bytes", lambda m: setattr(m, "description", "\u00e9" * 48 + "d"),
     ["description longer than 96 bytes"]),
    ("loop_and_hold_end", lambda m: (setattr(m, "loop", True), setattr(m, "hold_end", True)),
     ["loop and hold_end cannot both be set"]),
    ("no_keyframes", lambda m: m.ClearField("keyframes"), ["at least one keyframe is required"]),
    ("keyframes_33", lambda m: [m.keyframes.add(time=float(t)) for t in range(2, 33)], ["more than 32 keyframes"]),
    ("overlays_9", lambda m: [_overlay(m, body_axis=0) for _ in range(9)], ["more than 8 overlays"]),
    ("params_11", lambda m: [_param(m, i % 10, 1, 1, 2) for i in range(11)], ["more than 10 params"]),
    ("legs_3", lambda m: _legs(m, 3), ["must have 0 or 6 legs"]),
    ("ease_4", lambda m: setattr(m.keyframes[1], "ease", 4), ["ease out of range"]),
    ("entry_time_infinite", lambda m: setattr(m, "entry_time", math.inf), ["entry_time and exit_time must be finite"]),
    ("ride_height_nan", lambda m: setattr(m, "ride_height", math.nan), ["ride_height must be finite"]),
    ("keyframe_time_nan", lambda m: setattr(m.keyframes[1], "time", math.nan), ["keyframe", "has a non-finite value"]),
    ("keyframe_body_infinite", lambda m: setattr(m.keyframes[1].body, "z", math.inf),
     ["keyframe", "has a non-finite value"]),
    ("keyframe_leg_nan", lambda m: setattr(_legs(m, 6)[2].foot, "y", math.nan), ["keyframe", "has a non-finite value"]),
    ("overlay_nan", lambda m: _overlay(m, body_axis=0, amplitude=math.nan), ["overlay", "has a non-finite value"]),
    ("param_infinite", lambda m: _param(m, 1, 0, 1, math.inf), ["param", "has a non-finite value"]),
    ("first_keyframe_late", lambda m: setattr(m.keyframes[0], "time", 0.1), ["first keyframe must be at time 0"]),
    ("keyframe_time_repeated", lambda m: setattr(m.keyframes[1], "time", 0.0), ["time must increase"]),
    ("overlay_without_channel", lambda m: _overlay(m), ["needs exactly one channel"]),
    ("overlay_body_axis_6", lambda m: _overlay(m, body_axis=6), ["body_axis out of range"]),
    ("overlay_foot_channel_18", lambda m: _overlay(m, foot_channel=18), ["foot_channel out of range"]),
    ("overlay_empty_window", lambda m: _overlay(m, body_axis=0, start=0.5, end=0.5),
     ["window must have 0 <= start < end"]),
    ("overlay_past_the_end", lambda m: _overlay(m, body_axis=0, end=1.5), ["end is after the last keyframe"]),
    ("param_id_10", lambda m: _param(m, 10, 0.5, 1, 2), ["id out of range"]),
    ("param_repeated", lambda m: [_param(m, 0, 0.5, 1, 2) for _ in range(2)], ["is not unique"]),
    ("param_default_above_max", lambda m: _param(m, 3, 0.5, 3, 2), ["needs min <= default_value <= max"]),
    ("speed_min_zero", lambda m: _param(m, 0, 0, 1, 2), ["param SPEED needs a positive min"]),
    ("repeat_min_zero", lambda m: _param(m, 9, 0, 1, 2), ["param REPEAT needs min >= 1"]),
]

# platform_shared/animation.options: nanopb's string buffers hold a terminator, and its arrays have a fixed capacity.
NAME_BYTES, DESCRIPTION_BYTES, KEYFRAMES, OVERLAYS, PARAMS, LEGS = 32, 96, 32, 8, 10, 6


def _decodes(msg: pb.Animation) -> bool:
    return (len(msg.name.encode()) <= NAME_BYTES and len(msg.description.encode()) <= DESCRIPTION_BYTES
            and len(msg.keyframes) <= KEYFRAMES and len(msg.overlays) <= OVERLAYS and len(msg.params) <= PARAMS
            and all(len(k.legs) <= LEGS for k in msg.keyframes))


def generate_validation() -> list[dict]:
    """Every case through the reference validate(), whose error must contain the case's fragments."""
    cases = []
    for name, change, fragments in VALIDATION_CASES:
        msg = _base()
        change(msg)
        err = an.validate(from_proto(msg))
        if (err is None) != (not fragments) or any(f not in (err or "") for f in fragments):
            raise SystemExit(f"validation case {name}: the reference reports {err!r}, not {fragments}")
        cases.append({"case": name, "fragments": fragments, "decodes": _decodes(msg),
                      "animation": json_format.MessageToDict(msg)})
    return cases


def validation_dump(cases: list[dict]) -> str:
    """The same content as validation.json in the line format firmware/test/test_animation parses."""
    lines = []
    for c in cases:
        encoded = json_format.ParseDict(c["animation"], pb.Animation()).SerializeToString().hex() or "-"
        lines.append(f"{c['case']} {int(c['decodes'])} {encoded} {'|'.join(c['fragments']) or '-'}")
    return "\n".join(lines) + "\n"


def main() -> None:
    expected = generate()
    EXPECTED.write_text(json.dumps(expected, indent=1) + "\n", newline="\n")
    EXPECTED_TXT.write_text(text_dump(expected), newline="\n")
    write_fixture_binaries()
    validation = generate_validation()
    VALIDATION.write_text(json.dumps(validation, indent=1) + "\n", newline="\n")
    VALIDATION_TXT.write_text(validation_dump(validation), newline="\n")
    print(f"wrote {EXPECTED}, {EXPECTED_TXT}, {VALIDATION}, {VALIDATION_TXT} and the fixture .pb files")


if __name__ == "__main__":
    main()
