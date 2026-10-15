"""Measure the firmware gait's body velocity per stick in MuJoCo -> src/resources/speed_model.json.

The Robot facade (src/hexapod) turns a REP-103 velocity into the sticks the firmware takes, so it needs the
velocity each stick deflection produces. No closed form is accurate: the classical gait's cadence law is
piecewise (motion.h / gait.h: speed = step_speed * clip(stride / 25, 0.75, 1.5)), the feet slip, and the TUNED
gait's stick -> velocity-command map overshoots what it walks. So each axis is measured on its own, from rest,
through the same firmware control loop the sim backend runs:

    ly -> forward speed vx    lx -> leftward speed vy    rx -> counter-clockwise yaw rate

over `--settle` seconds of ramp-up and then a `--window` of steady walking, in the robot's body frame.
This is the simulated robot; the real one slips differently, so check a distance on the floor before relying on it.

  uv run python measure_speed_model.py              # all modelled gaits, ~1 min on 8 workers
"""
import argparse
import json
import math
from concurrent.futures import ProcessPoolExecutor
from itertools import product

from src.hexapod.speed_model import AXES, MODEL_PATH
from src.robot.firmware_gait import BI_GATE, RIPPLE, TRI_GATE, TUNED, WAVE
from src.sim.firmware_loop import FIRMWARE_DT, FirmwareLoop
from src.sim.mj_runtime import HexapodSim

GAITS = {"TRI_GATE": TRI_GATE, "BI_GATE": BI_GATE, "WAVE": WAVE, "RIPPLE": RIPPLE, "TUNED": TUNED}
MAGNITUDES = (0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0)
STICKS = tuple(sorted({0.0, *MAGNITUDES, *(-m for m in MAGNITUDES)}))
SETTLE_S = 1.5
WINDOW_S = 6.0


def measure(gait: int, axis: str, stick: float, settle_s: float = SETTLE_S, window_s: float = WINDOW_S) -> float:
    """Steady-state REP-103 velocity along `axis` (m/s or rad/s) for one stick held from rest."""
    sim = HexapodSim()
    sim.reset_to_stand()
    loop = FirmwareLoop(sim, gait)
    sticks = {"lx": 0.0, "ly": 0.0, "rx": 0.0}
    sticks[AXES[axis]] = stick
    loop.walk(**sticks)
    for _ in range(int(round(settle_s / FIRMWARE_DT))):
        loop.tick()
    forward = left = turned = 0.0
    position, yaw = sim.data.qpos[:2].copy(), sim.base_yaw()
    for _ in range(int(round(window_s / FIRMWARE_DT))):
        loop.tick()
        step = sim.data.qpos[:2] - position
        # The body faces world +Y at reset, so after a turn by `yaw` forward is (-sin, cos) and left (-cos, -sin).
        forward += -step[0] * math.sin(yaw) + step[1] * math.cos(yaw)
        left += -step[0] * math.cos(yaw) - step[1] * math.sin(yaw)
        now = sim.base_yaw()
        turned += (now - yaw + math.pi) % (2 * math.pi) - math.pi
        position, yaw = sim.data.qpos[:2].copy(), now
    return {"vx": forward, "vy": left, "yaw_rate": turned}[axis] / window_s


def _job(args):
    gait, axis, stick = args
    return measure(GAITS[gait], axis, stick)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--workers", type=int, default=8)
    args = ap.parse_args()
    jobs = list(product(GAITS, AXES, STICKS))
    with ProcessPoolExecutor(args.workers) as pool:
        results = dict(zip(jobs, pool.map(_job, jobs)))
    model = {
        "source": "measure_speed_model.py: each stick held alone from rest, through src/sim/firmware_loop.py",
        "settle_s": SETTLE_S,
        "window_s": WINDOW_S,
        "sticks": list(STICKS),
        "gaits": {g: {a: [round(results[g, a, s], 5) for s in STICKS] for a in AXES} for g in GAITS},
    }
    MODEL_PATH.write_text(json.dumps(model, indent=1) + "\n")
    for g in GAITS:
        top = {a: max(abs(v) for v in model["gaits"][g][a]) for a in AXES}
        print(f"{g:9s} top vx {top['vx']:.3f} m/s  vy {top['vy']:.3f} m/s  yaw {top['yaw_rate']:.2f} rad/s")
    print(f"-> {MODEL_PATH}")


if __name__ == "__main__":
    main()
