"""Replay the classical firmware gait in MuJoCo (no RL).

If the analytic gait+IK walks stably here, the model, actuators and gait port are
transfer-ready.

Usage:
  python replay_gait.py                  # interactive viewer (needs a display)
  python replay_gait.py --headless       # print walk metrics, no window
  python replay_gait.py --ly 0.5 --rx 0.3   # forward + turn

Command axes mirror the firmware CommandMsg (see firmware/include/motion.h):
  lx = strafe, ly = forward, rx = turn, s = speed, s1 = step height.

Like the firmware control task, the gait runs on a 5 ms tick and the stick command reaches it
through the same first-order ramp (approachGaitCommand), starting from rest.
"""

import argparse
import time
import numpy as np

from src.sim.mj_runtime import HexapodSim
from src.robot.firmware_gait import (AUTO, BI_GATE, RIPPLE, TRI_GATE, TUNED, WAVE, BodyState,
                                     GaitController, GaitState, approach_gait_command,
                                     command_speed01, command_to_walk_gait, select_auto_gait,
                                     set_gait)

FIRMWARE_DT = 0.005          # control-task period (firmware/src/main.cpp)
BODY_SMOOTHING = 0.06        # MotionService::smoothing_factor
GAITS = {"tri": TRI_GATE, "bi": BI_GATE, "wave": WAVE, "ripple": RIPPLE, "tuned": TUNED, "auto": AUTO}


class FirmwareLoop:
    """MotionService's WALK tick for a constant stick command: ramp, gait step, body lerp."""

    def __init__(self, args):
        requested = GAITS[args.gait]
        self.target = GaitState(gait_type=TUNED if requested == TUNED else TRI_GATE)
        self.target_zm = command_to_walk_gait(lx=args.lx, ly=args.ly, rx=args.rx, s=args.s,
                                              s1=args.s1, gait=self.target)
        self.live = GaitState()
        # The robot starts at rest, which is when a gait selection (manual or AUTO) is applied.
        self.live.gait_type = (select_auto_gait(command_speed01(self.target), self.live.gait_type)
                               if requested == AUTO else requested)
        set_gait(self.live)
        self.target.gait_type = self.live.gait_type
        self.target.offset = self.live.offset.copy()
        self.target.stand_frac = self.live.stand_frac
        self.gc, self.body = GaitController(), BodyState()
        self.ticks = 0
        self.physics_steps = 0

    def tick(self, sim):
        approach_gait_command(self.live, self.target, FIRMWARE_DT)
        self.body.zm += (self.target_zm - self.body.zm) * BODY_SMOOTHING
        self.gc.step(self.live, self.body, FIRMWARE_DT)
        sim.set_joint_targets(sim.body_targets_from_feet(self.body))
        self.ticks += 1
        # 5 ms is not a whole number of physics steps, so keep physics time locked to tick time.
        due = int(round(self.ticks * FIRMWARE_DT / sim.model.opt.timestep))
        sim.step_physics(due - self.physics_steps)
        self.physics_steps = due


def run_headless(args):
    sim = HexapodSim()
    sim.reset_to_stand()
    loop = FirmwareLoop(args)
    x0, y0 = sim.data.qpos[0], sim.data.qpos[1]
    heights = []
    n = int(round(args.seconds / FIRMWARE_DT))
    for _ in range(n):
        loop.tick(sim)
        heights.append(sim.base_height())
    dx = (sim.data.qpos[0] - x0) * 1000
    dy = (sim.data.qpos[1] - y0) * 1000
    dist = np.hypot(dx, dy)
    print(f"command lx={args.lx} ly={args.ly} rx={args.rx} gait={args.gait} for {args.seconds}s:")
    print(f"  travel: {dist:.0f} mm  (dx={dx:.0f}, dy={dy:.0f})  ->  {dist/args.seconds/1000:.3f} m/s")
    print(f"  height: mean {np.mean(heights)*1000:.1f} mm, min {np.min(heights)*1000:.1f} mm")
    print(f"  final quat (w x y z): {np.round(sim.base_quat(),3)}  (w~1 = upright)")
    upright = sim.base_quat()[0] > 0.9 and np.min(heights) > 0.04
    print("  RESULT:", "PASS (stable walk)" if upright and dist > 50 else "CHECK")


def run_viewer(args):
    import mujoco.viewer

    sim = HexapodSim()
    sim.reset_to_stand()
    loop = FirmwareLoop(args)
    with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
        while viewer.is_running():
            t0 = time.time()
            loop.tick(sim)
            viewer.sync()
            dt = FIRMWARE_DT - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--lx", type=float, default=0.0)
    ap.add_argument("--ly", type=float, default=0.5)
    ap.add_argument("--rx", type=float, default=0.0)
    ap.add_argument("--s", type=float, default=0.0)
    ap.add_argument("--s1", type=float, default=0.0)
    ap.add_argument("--gait", choices=sorted(GAITS), default="tri")
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--headless", action="store_true")
    args = ap.parse_args()
    (run_headless if args.headless else run_viewer)(args)
