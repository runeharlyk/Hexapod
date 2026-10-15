"""Writes app/tests/unit/fixtures/servo_model.json from HexapodSim's servo model, the reference the app's
simulation view ports (app/src/lib/simulation/servo.ts).

    uv run python scripts/make_servo_fixture.py

test_servo_fixture.py fails when the file no longer matches the model.
Each sample is one joint's target, position and velocity (rad, rad/s) and the torque _apply_servo commands
for it (N.m), with the frictionloss it sets on the joint.
"""
import json
import sys
from pathlib import Path

import numpy as np

SIMULATION = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIMULATION))

from src.sim import mj_runtime  # noqa: E402
from src.sim.mj_runtime import HexapodSim  # noqa: E402

OUT = Path(__file__).resolve().parents[2] / "app" / "tests" / "unit" / "fixtures" / "servo_model.json"


def fixture() -> dict:
    sim = HexapodSim()
    rng = np.random.default_rng(0)
    samples = []
    # Far from the target the torque sits on the motor's cap; within a few milliradians and a fraction of a
    # rad/s it follows the speed and stopping laws, which is what the samples must pin.
    for error_rad, speed_rad_s in [(0.6, 20.0)] * 4 + [(0.003, 0.4)] * 8:
        sim.joint_target = rng.uniform(-1.2, 1.2, 18)
        sim.data.qpos[sim.qpos_adr] = sim.joint_target + rng.uniform(-error_rad, error_rad, 18)
        sim.data.qvel[sim.qvel_adr] = rng.uniform(-speed_rad_s, speed_rad_s, 18)
        sim._apply_servo()
        for j in range(18):
            samples.append({
                "target": float(sim.joint_target[j]),
                "position": float(sim.data.qpos[sim.qpos_adr[j]]),
                "velocity": float(sim.data.qvel[sim.qvel_adr[j]]),
                "torque": float(sim.data.ctrl[sim.act_ids[j]]),
                "frictionloss": float(sim.model.dof_frictionloss[sim.qvel_adr[j]]),
                "base_frictionloss": float(sim.base_frictionloss[j]),
            })
    constants = {name: getattr(mj_runtime, f"SERVO_{name.upper()}")
                 for name in ("kp", "kv", "j_eff", "stall", "k_hold", "noload")}
    return {"constants": constants, "samples": samples}


def main():
    content = fixture()
    OUT.write_text(json.dumps(content, indent=1) + "\n")
    print(f"wrote {OUT} ({len(content['samples'])} samples)")


if __name__ == "__main__":
    main()
