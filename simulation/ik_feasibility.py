"""Does a gait command joint angles the servos cannot reach?

The IK clamps its acos arguments, and MuJoCo clamps to `jnt_range`, so an infeasible command does
not throw -- it silently becomes a different foot trajectory. A gait can therefore score well while
relying on commands the hardware will never execute, and the sim-to-real gap shows up as "it looks
nothing like the video".

Foot lift is limited by the femur's +-90 deg range, and that limit MOVES with ride height, which is
why the searched gaits raise the body: `__single___wide3` asks for 68 mm of lift and raises 18.8 mm,
which takes its out-of-range commands from 9.2% (at ride 0) down to 1.5%.

  python ik_feasibility.py                       # every non-curb library gait
  python ik_feasibility.py --gaits __single___wide3 --stride 60
"""

import argparse
import json
import os

import numpy as np

from src.robot.gait_schedule import GaitSchedule
from src.sim.ik_feasibility_lite import joint_ranges, lift_mm, sample_joint_angles

LIB = os.path.join(os.path.dirname(__file__), "src", "resources", "gait_library.json")
JOINTS = ("coxa", "femur", "tibia")


def check(sched: GaitSchedule, ranges, stride_mm=60.0, samples=200):
    ang = sample_joint_angles(sched, stride_mm, samples)
    excess = np.maximum(ranges[:, 0] - ang, ang - ranges[:, 1])   # > 0 where a command is clamped
    out = excess > 0
    per_joint = {jn: int(np.count_nonzero(out[:, j::3])) for j, jn in enumerate(JOINTS)}
    worst = float(np.rad2deg(excess[out].max())) if out.any() else 0.0
    return {"pct_out": 100.0 * np.count_nonzero(out) / out.size, "worst_deg": worst,
            "lift_mm": lift_mm(sched), "ride_mm": sched.ride_mm, "per_joint": per_joint}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gaits", nargs="*", default=None)
    ap.add_argument("--stride", type=float, default=60.0, help="step_y in mm to test at")
    args = ap.parse_args()

    ranges = joint_ranges()
    lib = json.load(open(LIB))["gaits"]
    keys = args.gaits or [k for k, v in lib.items() if not v.get("curb")]

    print(f"femur range +-{np.rad2deg(ranges[JOINTS.index('femur')][1]):.0f} deg, "
          f"coxa +-{np.rad2deg(ranges[JOINTS.index('coxa')][1]):.1f} deg, stride {args.stride:.0f} mm")
    print(f"\n{'gait':22s} {'lift_mm':>8s} {'ride_mm':>8s} {'%out':>7s} {'worst':>7s}  offending joints")
    rows = []
    for k in keys:
        m = check(GaitSchedule.from_dict(lib[k]["params"]), ranges, args.stride)
        rows.append((m["pct_out"], k, m))
    for pct, k, m in sorted(rows):
        off = ", ".join(f"{j}x{c}" for j, c in m["per_joint"].items() if c) or "-"
        print(f"{k:22s} {m['lift_mm']:8.1f} {m['ride_mm']:8.1f} {pct:6.1f}% {m['worst_deg']:6.1f}deg  {off}")
    print("\n%out = fraction of commanded joint angles outside the model's jnt_range. Anything above")
    print("0 means the gait relies on clamped commands, and the real trajectory will differ.")


if __name__ == "__main__":
    main()
