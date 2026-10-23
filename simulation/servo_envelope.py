"""Measures the servo envelope on the bench (roadmap 1.7) and writes its tables for docs/servo-envelope.md:

    uv run python servo_envelope.py --host 192.168.1.151 --out envelope-20261025 move
    uv run python servo_envelope.py --port COM5 --out envelope-20261025 travel
    uv run python servo_envelope.py --sim --out runs/envelope-sim move travel

What the telemetry can and cannot tell. The servos report nothing back: a tick's `angles` and `pwm` are what the robot
commanded and wrote, `servo_write_right` and `servo_write_left` whether the PCA9685 acknowledged the write, and
`servo_write_us` how long the I2C write took. None of them says where a horn is, or when it got there. The IMU sees the
body move, at 40 Hz (a sample every 25 ms), and on a stand that holds the body still it sees at most the stand flex.
So the move time comes from a slow-motion video: the frames from the leg's first motion to its stop, over the frame
rate. At 240 fps a frame is 4.2 ms; at 30 fps an 80 ms move is two or three frames, and a stopwatch cannot time it at
all. The script makes the moves repeatable and measures what the telemetry does carry: the tick each command took
effect on, that the board acknowledged it, the dwell the robot really held between two commands (so the link's
latency does not enter), and the IMU's response, as a coarse cross-check with a 25 ms resolution.

Both routines start from DEACTIVATED with the robot on a stand, its legs free. The robot is stood (STAND drives all 18
servos, so the rail carries every servo's holding current, as on the floor) and switched to IDLE, which leaves the
servos powered where they are and takes single joints (JointAngleData) for the joint under test. Every joint move of
more than STEP_DEG is ramped in STEP_DEG steps, except the swing being timed. After each routine, and on Ctrl-C or any
error, the robot is switched off, its legs going limp on the stand.

move: the femur under test (--servo, default 1: leg 0's) swings between --from-deg and --from-deg + SPAN_DEG, --repeats
times with --dwell-s at each end, for the video. Then the sweep: the same swing with the dwell shortened through
--sweep-ms, CYCLES times each, and the operator says each time whether the leg still reached both end marks. The swing's
reversal cuts short a move that has not finished, so the shortest dwell that still reaches both marks bounds the move
time from above and the next shorter one from below. A mass hung from the foot (the leg's share of the robot's weight
in a tripod stance, with the femur horizontal) makes it the move under the robot's own weight; --load-g records it.

travel: per joint of one leg (--leg), from the servo's calibrated centre pulse (coxa and femur 0, tibia -90 deg in
the angles the telemetry reports, at the factory calibration) it steps STEP_DEG at a time toward one end of the joint's
range and then toward the other. The range is the servo's calibrated pulse range, SERVO_HALF_RANGE_DEG either side of
its centre pulse at its counts per degree, mapped into the telemetry's angles through the robot's servo calibration
(centre pulse, centre angle, direction and conversion, as the firmware's toPwm applies them) and kept within the
firmware's JOINT_LIMIT_DEG, to which the robot clamps every joint: the tibia's telemetry limit of +149 deg is 239 deg
past the servo's centre, far beyond its pulse range, so its + side ends at 0. At each step the operator looks and
answers explicitly whether the joint moved freely; Enter stops the side, as does the end stop (the servo buzzes and
holds short) or the body or another leg in the way, and q ends the joint. The last free angle each way is the travel
reached with the current calibration. measure_pwm_per_degree.py's counts per degree (--scale) turn each into true
degrees: commanded x conversion / scale. Every command checks that the robot is still in IDLE, which alone takes them,
so a robot someone switched out of it stops the routine instead of answering for moves it never made.

The output directory holds move.hexrec and travel.hexrec (record.py's format), envelope.json and envelope.md.
"""
import argparse
import json
import math
import sys
from contextlib import ExitStack
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np

from session_prompts import Scripted, Terminal
from src.hexapod import Mode, Robot, RobotError
from src.hexapod.servos import ServoCalibration
from src.robot.animation import JOINT_LIMIT_DEG
from src.robot.firmware_gait import BodyState, Kinematics
from src.sim import recording

STAND_S = 1.5
SETTLE_S = 0.5
STEP_DEG = 5.0
STEP_S = 0.15  # between the steps of a ramp: a 5 deg step takes an MG92B under 20 ms
SPAN_DEG = 60.0
FROM_DEG = -30.0
REPEATS = 5
DWELL_S = 1.0
SWEEP_MS = (400, 250, 160, 120, 100, 80, 60)
CYCLES = 4
JOINTS = ("coxa", "femur", "tibia")
LEGS = 6
# The servo's travel either side of its calibrated centre pulse, at the calibration's counts per degree: a hobby
# servo's 180 deg, which the firmware's joint limits and the simulation's joint ranges assume as well.
SERVO_HALF_RANGE_DEG = 90.0
FACTORY_CONVERSION = 2.0  # PWM counts per degree the firmware assumes (servo_factory.h)
IMU_FLOOR_RAD_S = 0.05  # a gyro response must rise this far above the still body's spread to count
IMU_SAMPLE_MS = 25.0
BEFORE_S = 0.5  # the still stretch before the first command the IMU's spread is taken from
# Enter, the first, stops: only an explicit answer steps on.
TRAVEL_ANSWERS = {"": "stopped by the operator", "f": "moved freely", "s": "stopped on its end stop",
                  "b": "blocked by the body or a leg", "q": "quit this joint"}
FREE, QUIT = "f", "q"


class Refused(Exception):
    pass


# --- on the robot ---------------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class JointRange:
    """Where one joint may be commanded, in the telemetry's angles."""

    centre: float  # at the servo's calibrated centre pulse, within the range
    low: float
    high: float
    servo_bound: tuple[bool, bool]  # whether the servo's pulse range, not the firmware's limit, ends low and high


def joint_range(calibration: ServoCalibration, servo: int) -> JointRange:
    """The servo's calibrated pulse range, SERVO_HALF_RANGE_DEG either side of its centre pulse at its conversion,
    mapped into the telemetry's angles through the calibration (ServoCalibration.angles, toPwm's inverse), within
    the firmware's JOINT_LIMIT_DEG."""
    def angle_at(pwm: float) -> float:
        pwms = np.array(calibration.center_pwm, dtype=float)
        pwms[servo] = pwm
        return float(calibration.angles(pwms)[servo])

    centre_pwm = calibration.center_pwm[servo]
    span = SERVO_HALF_RANGE_DEG * abs(calibration.conversion[servo])
    servo_low, servo_high = sorted((angle_at(centre_pwm - span), angle_at(centre_pwm + span)))
    limit = float(JOINT_LIMIT_DEG[servo % 3])
    low, high = max(servo_low, -limit), min(servo_high, limit)
    if low >= high:
        raise Refused(f"servo {servo}'s calibrated range, {servo_low:+.1f} to {servo_high:+.1f} deg, lies outside "
                      f"the firmware's +-{limit:g} deg: check its calibration")
    return JointRange(float(np.clip(angle_at(centre_pwm), low, high)), low, high,
                      (servo_low > -limit, servo_high < limit))


def require_idle(robot: Robot) -> None:
    mode = robot.state().mode
    if mode is not Mode.IDLE:
        raise Refused(f"the robot left IDLE for {mode.name if mode is not None else 'an unknown mode'}: it takes no "
                      "single joints there, so the routine stops")


class Joints:
    """The angles this script has sent, so every larger move is ramped from where the joint was sent last. Every
    command checks first that the robot is still in IDLE, the one mode that takes it."""

    def __init__(self, robot: Robot, stance: dict[int, float]):
        self.robot, self.sent = robot, dict(stance)

    def jump(self, servo: int, degrees: float) -> None:
        require_idle(self.robot)
        self.robot.set_joint(servo, degrees)
        self.sent[servo] = degrees

    def ramp(self, servo: int, degrees: float) -> None:
        start = self.sent[servo]
        steps = max(1, math.ceil(abs(degrees - start) / STEP_DEG))
        for k in range(1, steps + 1):
            self.jump(servo, start + (degrees - start) * k / steps)
            self.robot.sleep(STEP_S)


def require_deactivated(robot: Robot) -> None:
    mode = robot.state().mode
    if mode is not Mode.DEACTIVATED:
        raise Refused(f"the robot is {mode.name if mode is not None else 'in an unknown mode'}: switch it off "
                      "(DEACTIVATED) first")


def idle_on_the_stand(robot: Robot, operator, what: str) -> Joints:
    """Stands the robot on its stand and switches it to IDLE, from DEACTIVATED only."""
    require_deactivated(robot)
    operator.confirm(f"{what}: robot on its stand, every leg free of the bench")
    robot.stand()
    robot.sleep(STAND_S)
    robot.idle()
    robot.sleep(SETTLE_S)
    require_idle(robot)
    return Joints(robot, dict(enumerate(stance_angles())))


def stance_angles() -> list[float]:
    """The joint angles STAND holds at the default posture, which IDLE keeps."""
    return [float(a) for a in Kinematics().inverse_kinematics(BodyState())]


@dataclass
class SweepStep:
    dwell_ms: float
    reached: bool | None = None  # the operator's answer: both end marks reached at this dwell


@dataclass
class Move:
    servo: int
    from_deg: float
    to_deg: float
    load_g: float | None
    repeats: int
    dwell_s: float
    sweep: list[SweepStep] = field(default_factory=list)
    video_ms: float | None = None
    rail_v_holding: float | None = None
    rail_v_moving: float | None = None


def run_move(robot: Robot, operator, directory: Path, servo: int, from_deg: float, repeats: int, dwell_s: float,
             sweep_ms, load_g: float | None) -> Move:
    move = Move(servo, from_deg, from_deg + SPAN_DEG, load_g, repeats, dwell_s)
    reach = joint_range(robot.servo_calibration(), servo)
    if not reach.low <= move.from_deg < move.to_deg <= reach.high:
        raise Refused(f"the swing {move.from_deg:+g} to {move.to_deg:+g} deg leaves servo {servo}'s range, "
                      f"{reach.low:+.1f} to {reach.high:+.1f} deg")
    joints = idle_on_the_stand(robot, operator, f"femur move of servo {servo}")
    joints.ramp(servo, move.from_deg)
    operator.confirm(f"mark the femur's ends at {move.from_deg:+g} and {move.to_deg:+g} deg; start the slow-motion "
                     "video (240 fps or more), the leg and the marks in frame")
    move.rail_v_holding = operator.number("servo rail voltage at a servo connector, all servos holding (V)")
    with robot.recording(directory / "move.hexrec"):
        robot.sleep(BEFORE_S + SETTLE_S)
        for _ in range(repeats):
            for end in (move.to_deg, move.from_deg):
                joints.jump(servo, end)
                robot.sleep(dwell_s)
        for dwell in sweep_ms:
            for _ in range(CYCLES):
                for end in (move.to_deg, move.from_deg):
                    joints.jump(servo, end)
                    robot.sleep(dwell / 1000.0)
            robot.sleep(SETTLE_S)
            answer = operator.choice(f"at a dwell of {dwell} ms, did the leg reach both marks every time",
                                     {"y": "yes", "n": "no"})
            move.sweep.append(SweepStep(float(dwell), answer == "y"))
        robot.sleep(SETTLE_S)
    move.video_ms = operator.number("move time from the video: frames from the first motion to the stop, x 1000 / "
                                    "fps (ms)")
    move.rail_v_moving = operator.number("lowest servo rail voltage seen during the swings (V)")
    return move


@dataclass
class Travel:
    joint: str
    servo: int
    low_deg: float  # the range it was stepped within (joint_range)
    high_deg: float
    reached: dict[str, float] = field(default_factory=dict)  # "+" and "-": the last angle the joint moved freely to
    stopped_by: dict[str, str] = field(default_factory=dict)


def run_travel(robot: Robot, operator, directory: Path, leg: int, joint_names) -> list[Travel]:
    if not 0 <= leg < LEGS:
        raise ValueError(f"no leg {leg} (0-{LEGS - 1})")
    calibration = robot.servo_calibration()
    joints = idle_on_the_stand(robot, operator, f"joint travel of leg {leg}")
    results = []
    with robot.recording(directory / "travel.hexrec"):
        for name in joint_names:
            servo = 3 * leg + JOINTS.index(name)
            reach = joint_range(calibration, servo)
            travel = Travel(name, servo, reach.low, reach.high)
            joints.ramp(servo, reach.centre)
            for end, side, servo_bound in ((reach.high, "+", reach.servo_bound[1]),
                                           (reach.low, "-", reach.servo_bound[0])):
                travel.reached[side], travel.stopped_by[side] = step_out(robot, operator, joints, servo, name,
                                                                         reach.centre, end, servo_bound)
                joints.ramp(servo, reach.centre)
                if travel.stopped_by[side] == TRAVEL_ANSWERS[QUIT]:
                    break
            joints.ramp(servo, stance_angles()[servo])
            results.append(travel)
    return results


def step_out(robot: Robot, operator, joints: Joints, servo: int, name: str, centre: float, end: float,
             servo_bound: bool) -> tuple[float, str]:
    """Steps from the centre toward `end`, never past it; the last angle the operator saw it move freely to, and why
    it stopped."""
    reached, angle = centre, centre
    while angle != end:
        angle = min(angle + STEP_DEG, end) if end > centre else max(angle - STEP_DEG, end)
        joints.jump(servo, angle)
        robot.sleep(SETTLE_S)
        answer = operator.choice(f"{name} (servo {servo}) at {angle:+.1f} deg", TRAVEL_ANSWERS)
        if answer != FREE:
            return reached, TRAVEL_ANSWERS[answer]
        reached = angle
    return reached, "the servo's calibrated range" if servo_bound else "the firmware's limit"


def session(robot: Robot, operator, directory: Path, routines, args) -> dict:
    """Runs the routines and leaves the robot switched off; envelope.json and envelope.md are written on
    the way out of any error or Ctrl-C as well."""
    robot.connect()
    try:
        require_deactivated(robot)  # before any mode or stick: connecting sets only the gait
    except BaseException:
        robot.close()
        raise
    directory.mkdir(parents=True, exist_ok=True)
    results = {"features": robot.features(), "scale": args.scale}
    with ExitStack() as leaving:
        leaving.push(robot)  # Robot.__exit__: rests the robot and closes the link, on an error and Ctrl-C too
        try:
            for routine in routines:
                if routine == "move":
                    results["move"] = asdict(run_move(robot, operator, directory, args.servo, args.from_deg,
                                                      args.repeats, args.dwell_s, args.sweep_ms, args.load_g))
                else:
                    results["travel"] = [asdict(t) for t in run_travel(robot, operator, directory, args.leg,
                                                                       args.joints)]
                robot.rest()
        finally:
            (directory / "envelope.json").write_text(json.dumps(results, indent=1) + "\n")
            (directory / "envelope.md").write_text(report(directory, results) + "\n")
    return results


# --- from the recordings --------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class Command:
    t_s: float  # the tick it took effect on, robot clock
    degrees: float
    acknowledged: bool | None  # None: the recording carries no write result (a host build, or an older firmware)
    pwm: int | None


def commands(rec: recording.Recording, servo: int) -> list[Command]:
    """Every change of the joint's commanded angle in the recording, at the tick it took effect."""
    t = rec.ticks
    angle = t["angles"][:, servo]
    changed = np.flatnonzero(np.diff(angle) != 0.0) + 1
    side = "servo_write_right" if servo < 9 else "servo_write_left"
    out = []
    for k in changed:
        ack = None
        if side in t:
            ack = int(t[side][k]) in (1, 4)  # SERVO_WRITE_OK, SERVO_WRITE_RESTARTED
        pwm = (int(t["pwm"][k, servo]) or None) if "pwm" in t else None  # 0: not written on that tick
        out.append(Command(float(t["t_us"][k]) / 1e6, float(angle[k]), ack, pwm))
    return out


def imu_response_ms(rec: recording.Recording, starts: list[float], ends: list[float]) -> list[float]:
    """Per command, from its tick to the last fresh gyro sample above the still body's spread before the next command
    (ms); NaN where the gyro never rose. The IMU's 40 Hz makes each good to about IMU_SAMPLE_MS."""
    t = rec.ticks
    fresh = t["imu_fresh"].astype(bool)
    time = t["t_us"][fresh] / 1e6
    rate = np.linalg.norm(t["gyro"][fresh], axis=1)
    still = rate[(time < starts[0]) & (time >= starts[0] - BEFORE_S)] if starts else rate[:0]
    threshold = (float(still.mean() + 4 * still.std()) if len(still) > 1 else 0.0) + IMU_FLOOR_RAD_S
    out = []
    for start, end in zip(starts, ends):
        moving = (time >= start) & (time < end) & (rate > threshold)
        out.append(1000.0 * (float(time[moving][-1]) - start) if moving.any() else math.nan)
    return out


def _f(value, digits: int = 1) -> str:
    return "-" if value is None or (isinstance(value, float) and math.isnan(value)) else f"{value:.{digits}f}"


def _median(values) -> float:
    values = [v for v in values if not math.isnan(v)]
    return float(np.median(values)) if values else math.nan


def sweep_bounds(sweep: list[dict]) -> tuple[float | None, float | None]:
    """The move time's bounds from the sweep: under the shortest dwell that reached both marks, over the longest one
    shorter than that which did not."""
    reached = [s["dwell_ms"] for s in sweep if s["reached"]]
    upper = min(reached) if reached else None
    missed = [s["dwell_ms"] for s in sweep if s["reached"] is False and (upper is None or s["dwell_ms"] < upper)]
    return upper, max(missed) if missed else None


def move_table(directory: Path, move: dict) -> list[str]:
    path = directory / "move.hexrec"
    lines = [f"### 60 deg femur move, servo {move['servo']}, {move['from_deg']:+g} to {move['to_deg']:+g} deg", "",
             f"Load on the foot: {_f(move['load_g'], 0)} g; rail at a servo connector: {_f(move['rail_v_holding'], 2)}"
             f" V holding, {_f(move['rail_v_moving'], 2)} V lowest while swinging.", ""]
    rec = recording.load(path) if path.exists() else None
    if rec is None or not len(rec.ticks["t_us"]):
        return lines + ["No telemetry was recorded.", ""]
    sent = commands(rec, move["servo"])
    phases = [("repeats", move["dwell_s"] * 1000.0, 2 * move["repeats"])]
    phases += [(f"sweep {s['dwell_ms']:g} ms", s["dwell_ms"], 2 * CYCLES) for s in move["sweep"]]
    answers = {f"sweep {s['dwell_ms']:g} ms": s["reached"] for s in move["sweep"]}
    lines += ["| Phase | Dwell asked (ms) | Commands | Acknowledged | Dwell on the robot (ms, median) | "
              f"IMU response (ms, median, +-{IMU_SAMPLE_MS:g}) | Both marks reached |", "|---|---|---|---|---|---|---|"]
    index = 0
    for name, dwell_ms, count in phases:
        block = sent[index:index + count]
        index += count
        if not block:
            continue
        starts = [c.t_s for c in block]
        ends = starts[1:] + [starts[-1] + dwell_ms / 1000.0]
        held = [1000.0 * (b - a) for a, b in zip(starts, ends[:-1])]
        acks = [c.acknowledged for c in block if c.acknowledged is not None]
        reached = answers.get(name)
        lines.append(f"| {name} | {dwell_ms:g} | {len(block)} | {sum(acks) if acks else '-'} | "
                     f"{_f(_median(held), 0)} | {_f(_median(imu_response_ms(rec, starts, ends)), 0)} | "
                     f"{'-' if reached is None else 'yes' if reached else 'no'} |")
    upper, lower = sweep_bounds(move["sweep"])
    lines += ["", f"Move time from the video: {_f(move['video_ms'], 0)} ms.",
              f"From the sweep: over {_f(lower, 0)} ms and under {_f(upper, 0)} ms.",
              "The IMU response is when the body last moved above its still spread after a command, not the servo's "
              "travel; on a rigid stand it shows nothing.", ""]
    return lines


def travel_table(travels: list[dict], scale: float | None) -> list[str]:
    true = f" | True + / - (deg at {scale:g} counts/deg)" if scale else ""
    lines = ["| Joint | Servo | Stepped within (deg) | Reached + (deg) | Stopped + | Reached - (deg) | "
             f"Stopped -{true} |", "|---|---|---|---|---|---|---|" + ("---|" if scale else "")]
    for t in travels:
        row = (f"| {t['joint']} | {t['servo']} | {t['low_deg']:+.1f} to {t['high_deg']:+.1f} | "
               f"{_f(t['reached'].get('+'))} | {t['stopped_by'].get('+', '-')} | {_f(t['reached'].get('-'))} | "
               f"{t['stopped_by'].get('-', '-')}")
        if scale:
            factor = FACTORY_CONVERSION / scale
            row += f" | {_f(factor * t['reached'].get('+', math.nan))} / {_f(factor * t['reached'].get('-', math.nan))}"
        lines.append(row + " |")
    return lines + ["", "Angles as the telemetry reports them: coxa and femur 0 and tibia -90 at the servo's centre at "
                    "the factory calibration. Each joint was stepped within its servo's calibrated pulse range, "
                    f"+-{SERVO_HALF_RANGE_DEG:g} deg about its centre pulse, and the firmware's joint limit.", ""]


def report(directory: Path, results: dict) -> str:
    features = results.get("features", {})
    firmware = ("simulation " + features.get("firmwareVersion", "?") if features.get("backend") == "simulation" else
                f"{features.get('firmwareVersion', '?')} `{features.get('firmwareElfSha256', '')[:12]}`")
    lines = ["## Servo envelope on the bench", "", f"Firmware {firmware}.", ""]
    if "move" in results:
        lines += move_table(directory, results["move"])
    if "travel" in results:
        lines += ["### Mechanical travel", ""] + travel_table(results["travel"], results.get("scale"))
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    where = parser.add_mutually_exclusive_group(required=True)
    where.add_argument("--port", help="the robot's native USB port, e.g. COM5")
    where.add_argument("--host", help="the robot's address, e.g. 192.168.1.151")
    where.add_argument("--sim", action="store_true", help="the MuJoCo simulation, with no questions asked")
    parser.add_argument("--out", type=Path, required=True, help="output directory")
    parser.add_argument("--servo", type=int, default=1, help="the femur to time (1, 4, 7, 10, 13 or 16)")
    parser.add_argument("--from-deg", type=float, default=FROM_DEG, help="the swing's first end; it spans 60 deg up")
    parser.add_argument("--repeats", type=int, default=REPEATS)
    parser.add_argument("--dwell-s", type=float, default=DWELL_S)
    parser.add_argument("--sweep-ms", type=float, nargs="*", default=list(SWEEP_MS))
    parser.add_argument("--load-g", type=float, help="the mass hung from the foot, if any")
    parser.add_argument("--leg", type=int, default=0, choices=range(LEGS), metavar="0-5", help="the leg to step")
    parser.add_argument("--joints", nargs="*", default=list(JOINTS), choices=JOINTS)
    parser.add_argument("--scale", type=float, help="the servos' PWM counts per degree (measure_pwm_per_degree.py)")
    parser.add_argument("routines", nargs="+", choices=("move", "travel"))
    args = parser.parse_args()
    if args.servo % 3 != 1 or not 0 <= args.servo < 18:
        parser.error(f"servo {args.servo} is not a femur")
    if not (-90.0 <= args.from_deg and args.from_deg + SPAN_DEG <= 90.0):
        parser.error("the swing must stay within the femur's +-90 deg")
    target = "simulation" if args.sim else args.port or args.host
    try:
        session(Robot(target), Scripted() if args.sim else Terminal(), args.out, args.routines, args)
    except Refused as refusal:
        print(f"refused: {refusal}")
        return 2
    except RobotError as error:
        print(f"stopped: {error}")
        return 2
    except KeyboardInterrupt:
        print("interrupted: a robot the routine stood was switched off as far as the link allowed; check that its "
              "legs are limp")
        return 130
    print(f"wrote {args.out / 'envelope.md'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
