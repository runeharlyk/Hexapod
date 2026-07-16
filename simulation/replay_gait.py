"""Replay the classical firmware gait in MuJoCo (no RL).

If the analytic gait+IK walks stably here, the model, actuators and gait port are
transfer-ready.

Usage:
  python replay_gait.py                  # interactive viewer (needs a display)
  python replay_gait.py --headless       # print walk metrics, no window
  python replay_gait.py --ly 0.5 --rx 0.3   # forward + turn

Command axes mirror the firmware CommandMsg (see firmware/include/motion.h):
  lx = strafe, ly = forward, rx = turn, s = speed, s1 = step height.
"""

import argparse
import time
import numpy as np

from src.sim.mj_runtime import HexapodSim, CONTROL_DT
from src.robot.firmware_gait import GaitController, GaitState, BodyState, set_gait, command_to_walk_gait


def make_gait(args):
    gait = GaitState()
    command_to_walk_gait(lx=args.lx, ly=args.ly, rx=args.rx, s=args.s, s1=args.s1, gait=gait)
    set_gait(gait)
    return gait


def run_headless(args):
    sim = HexapodSim()
    sim.reset_to_stand()
    gait, gc, body = make_gait(args), GaitController(), BodyState()
    x0, y0 = sim.data.qpos[0], sim.data.qpos[1]
    heights, rolls = [], []
    n = int(args.seconds / CONTROL_DT)
    for _ in range(n):
        gc.step(gait, body, CONTROL_DT)
        sim.set_joint_targets(sim.body_targets_from_feet(body))
        sim.step_physics()
        heights.append(sim.base_height())
        rolls.append(sim.base_quat())
    dx = (sim.data.qpos[0] - x0) * 1000
    dy = (sim.data.qpos[1] - y0) * 1000
    dist = np.hypot(dx, dy)
    print(f"command lx={args.lx} ly={args.ly} rx={args.rx} for {args.seconds}s:")
    print(f"  travel: {dist:.0f} mm  (dx={dx:.0f}, dy={dy:.0f})  ->  {dist/args.seconds/1000:.3f} m/s")
    print(f"  height: mean {np.mean(heights)*1000:.1f} mm, min {np.min(heights)*1000:.1f} mm")
    print(f"  final quat (w x y z): {np.round(sim.base_quat(),3)}  (w~1 = upright)")
    upright = sim.base_quat()[0] > 0.9 and np.min(heights) > 0.04
    print("  RESULT:", "PASS (stable walk)" if upright and dist > 50 else "CHECK")


def run_viewer(args):
    import mujoco.viewer

    sim = HexapodSim()
    sim.reset_to_stand()
    gait, gc, body = make_gait(args), GaitController(), BodyState()
    with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
        while viewer.is_running():
            t0 = time.time()
            gc.step(gait, body, CONTROL_DT)
            sim.set_joint_targets(sim.body_targets_from_feet(body))
            sim.step_physics()
            viewer.sync()
            dt = CONTROL_DT - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--lx", type=float, default=0.0)
    ap.add_argument("--ly", type=float, default=0.5)
    ap.add_argument("--rx", type=float, default=0.0)
    ap.add_argument("--s", type=float, default=0.0)
    ap.add_argument("--s1", type=float, default=0.0)
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--headless", action="store_true")
    args = ap.parse_args()
    (run_headless if args.headless else run_viewer)(args)
