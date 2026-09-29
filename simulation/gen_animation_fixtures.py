"""Writes animations/fixtures/expected.json from the reference implementation.

The firmware and app parity tests read that file; test_animation_fixtures.py fails when it is stale.
    uv run python gen_animation_fixtures.py
"""
import json
from pathlib import Path

import numpy as np

from src.robot import animation as an
from src.robot.animation_files import load_json
from src.robot.firmware_gait import BodyState, Kinematics

ROOT = Path(__file__).resolve().parents[1]
FIXTURE_DIR = ROOT / "animations" / "fixtures"
EXPECTED = FIXTURE_DIR / "expected.json"
TOLERANCE = 1e-4

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
     "events": [{"step": 45, "action": "stop"}], "steps": 90},
    {"animation": "fx_mixed_legs", "params": {}, "dt": 0.02, "live": DISPLACED_LIVE,
     "events": [{"step": 40, "action": "play", "animation": "fx_overlay", "params": {"OVERLAY_AMPLITUDE": 1.5}}], "steps": 120},
    {"animation": "fx_single", "params": {}, "dt": 0.02, "live": DISPLACED_LIVE,
     "events": [{"step": 40, "action": "stop"}], "steps": 80},
    {"animation": "fx_params", "params": {"SPEED": 2.0, "REPEAT": 2}, "dt": 0.025, "live": DISPLACED_LIVE,
     "events": [], "steps": 100},
    {"animation": "fx_overlay", "params": {}, "dt": 0.02, "live": DISPLACED_LIVE,
     "events": [{"step": 110, "action": "stop"}], "steps": 150},
]


def sample_times(anim: an.Animation) -> list[float]:
    times = [k.time for k in anim.keyframes]
    mids = [(a + b) / 2 for a, b in zip(times, times[1:])]
    return sorted(set(times + mids + [-0.1, anim.duration + 0.5, 0.137, 0.731]))


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


def main() -> None:
    EXPECTED.write_text(json.dumps(generate(), indent=1) + "\n", newline="\n")
    print(f"wrote {EXPECTED}")


if __name__ == "__main__":
    main()
