"""Runs animations through the calibrated servo model and reports what a kinematic preview cannot:
clamped joints, commanded joint speed against the servo's no-load speed, body tilt, how much of the
time something other than a foot touches the ground, and whether the robot tipped over.

    uv run python check_animation.py                  # every file in ../animations
    uv run python check_animation.py ../animations/play_dead.json --dt 0.02

Exits 1 when any animation is invalid or falls, so it can gate CI. A statically stable hexapod
rarely tips; the tilt and knock columns are the numbers to read for everything short of that.
"""
import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from src.robot import animation as an
from src.robot.animation_files import load_json
from src.robot.firmware_gait import Kinematics
from src.sim.mj_runtime import CONTROL_DT, SERVO_NOLOAD, HexapodSim

LIBRARY = Path(__file__).resolve().parents[1] / "animations"
FALL_TILT_DEG = 45.0
LOOP_PLAYS = 2
MAX_SECONDS = 60.0


@dataclass
class Report:
    name: str
    error: str | None = None
    clamped_mask: int = 0
    peak_joint_speed: float = 0.0  # rad/s, commanded
    tilt_max_deg: float = 0.0
    knock_fraction: float = 0.0    # fraction of steps with a non-foot ground contact
    fell: bool = False
    steps: int = 0

    def ok(self) -> bool:
        return self.error is None and not self.fell


def check(anim: an.Animation, sim: HexapodSim, kin: Kinematics, dt: float = CONTROL_DT, settle: float = 1.0) -> Report:
    report = Report(anim.name)
    report.error = an.validate(anim)
    if report.error:
        return report
    sim.reset_to_stand()
    player = an.Player(kin)
    player.play(anim)
    previous = None
    stop_at = anim.duration * LOOP_PLAYS + anim.entry_seconds() if anim.loop or anim.hold_end else None
    settle_left = None
    elapsed = 0.0
    knock_steps = 0
    while elapsed < MAX_SECONDS:
        if stop_at is not None and elapsed >= stop_at and player.state in (an.State.PLAYING, an.State.HOLD):
            player.stop()
        angles_deg, mask = an.pose_to_angles(player.update(dt), kin)
        report.clamped_mask |= mask
        angles = np.radians(angles_deg)
        if previous is not None:
            report.peak_joint_speed = max(report.peak_joint_speed, float(np.max(np.abs(angles - previous)) / dt))
        previous = angles
        sim.set_joint_targets(angles)
        sim.step_physics()
        report.steps += 1
        elapsed += dt
        report.tilt_max_deg = max(report.tilt_max_deg, sim.body_tilt_deg())
        knock_steps += sim.contact_state()[1] > 0.0
        if player.state == an.State.IDLE:
            settle_left = settle if settle_left is None else settle_left - dt
            if settle_left <= 0.0:
                break
    report.knock_fraction = knock_steps / report.steps
    report.fell = report.tilt_max_deg > FALL_TILT_DEG
    return report


def format_row(r: Report) -> str:
    if r.error:
        return f"{r.name:16s} INVALID  {r.error}"
    verdict = "FELL" if r.fell else "ok"
    clamps = "none" if r.clamped_mask == 0 else f"{r.clamped_mask:018b}"
    return (f"{r.name:16s} {verdict:5s} tilt {r.tilt_max_deg:5.1f} deg  peak joint {r.peak_joint_speed:5.1f} rad/s "
            f"(servo {SERVO_NOLOAD:.0f})  knock {r.knock_fraction:4.2f}  clamped {clamps}  {r.steps} steps")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("files", nargs="*", help="animation JSON files (default: every file in the library)")
    ap.add_argument("--dt", type=float, default=CONTROL_DT, help="control step in seconds")
    ap.add_argument("--settle", type=float, default=1.0, help="seconds to keep simulating after the exit")
    args = ap.parse_args(argv)
    files = [Path(f) for f in args.files] or sorted(LIBRARY.glob("*.json"))
    sim, kin = HexapodSim(), Kinematics()
    failed = 0
    for path in files:
        r = check(load_json(path), sim, kin, dt=args.dt, settle=args.settle)
        print(format_row(r))
        failed += 0 if r.ok() else 1
    print(f"{len(files) - failed}/{len(files)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
