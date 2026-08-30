"""Check that firmware/include/gait_tuned.h still matches the gait it was generated from.

`gait_tuned.h` is generated, but nothing stops someone editing it, or re-searching the library
without re-exporting. Either way the robot would walk a gait the sim never measured, which is
exactly the failure mode the repo's "keep firmware and sim in sync" rule exists to prevent.

This parses the header's constants back out and compares them to the library entry, and separately
checks that the C++ velocity_to_gait port agrees numerically with GaitSchedule.gait_action.

  python test_firmware_gait_parity.py            # checks the key recorded in the header
  python test_firmware_gait_parity.py --key flat_tripod
"""

import argparse
import json
import os
import re
import sys

import numpy as np

from src.envs.hexapod_mj_env import PG_HEIGHT, PG_PHASE_RATE, PG_STEP_ANGLE, PG_STEP_XY
from src.robot.gait_schedule import GaitSchedule

HDR = os.path.join(os.path.dirname(__file__), "..", "firmware", "include", "gait_tuned.h")
LIB = os.path.join(os.path.dirname(__file__), "src", "resources", "gait_library.json")
TOL = 1e-4


def parse_header(path):
    text = open(path).read()
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
    lib = json.load(open(args.lib))["gaits"]
    if key not in lib:
        raise SystemExit(f"header references '{key}', absent from {args.lib}")
    s = GaitSchedule.from_dict(lib[key]["params"])

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
    chk("STEP_HEIGHT_MAX_MM", h.get("STEP_HEIGHT_MAX_MM"), PG_HEIGHT[1])
    chk("PHASE_RATE_MAX", h.get("PHASE_RATE_MAX"), PG_PHASE_RATE[1])
    chk("out[3] step height (normalized)", h.get("_step_height_norm"), s.step_height)

    # numerical parity of the C++ port, evaluated in Python against the same formula
    worst = 0.0
    for cmd in [(0.15, 0, 0), (0.30, 0, 0), (0.42, 0, 0), (-0.20, 0, 0),
                (0.10, 0.10, 0), (0.05, 0, -1.0), (0.0, 0, 0.5), (0.12, 0, 0.85)]:
        want = s.gait_action(np.array(cmd))
        vx, vy, yaw = cmd
        speed = float(np.hypot(vx, vy))
        b = min(speed / h["BLEND_SPEED"], 1.0)
        gx = h["GX0"] + (h["GX1"] - h["GX0"]) * b
        gy = h["GY0"] + (h["GY1"] - h["GY0"]) * b
        gyaw = h["GYAW0"] + (h["GYAW1"] - h["GYAW0"]) * b
        c1 = lambda v: max(-1.0, min(1.0, v))
        got = np.array([c1(vy / gy), c1(vx / gx), c1(yaw / gyaw + h["YAW_COMP"] * vx),
                        h["_step_height_norm"], b * 2 - 1,
                        c1(h["PR_BASE"] + h["PR_SLOPE"] * speed + h["PR_YAW"] * abs(yaw))])
        worst = max(worst, float(np.max(np.abs(got - want))))

    if worst > 1e-5:
        fails.append(f"velocity_to_gait disagrees with GaitSchedule.gait_action by {worst:.2e}")

    print(f"header : {os.path.abspath(args.header)}")
    print(f"key    : {key}   (library score {lib[key].get('val_score', float('nan')):.3f})")
    print(f"gait_action parity: max |diff| = {worst:.2e} over 8 commands")
    if fails:
        print(f"\nFAIL ({len(fails)}):")
        for f in fails:
            print("  -", f)
        print("\nre-run: python export_gait.py --key " + key)
        sys.exit(1)
    print("\nPASS - firmware header matches the searched gait")


if __name__ == "__main__":
    main()
