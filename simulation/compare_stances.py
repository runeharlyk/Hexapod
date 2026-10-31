"""How the two stances walk: the default stance against the crab, every leg straight out to its side, sideways and, for
reference, forward, on every classic timing, through the firmware's own motion code.

Two parts, both on the firmware's code (src/sim/virtual_robot.py), never a port:

1. MuJoCo walks (src/sim/firmware_loop.py): from a stand on the stance, each timing walks the stick held at a share of
   full speed (speed slider at 1, stride 0, neutral posture), and after SETTLE_S of walking, over WINDOW_S:
   - speed: the mean body-frame velocity along the stick's axis, mm/s;
   - sway: the body's roll and pitch, RMS about their mean, deg, named as the IMU names them (docs/conventions.md): roll
     about the lateral x axis, nose up and down, and pitch about the forward y axis, side to side; and the centre of
     mass across the direction of travel, RMS about the straight line fitted to its path on the floor, mm;
   - power: the mean of the servos' mechanical power, sum over the 18 actuators of |torque x joint velocity| (W,
     HexapodSim.actuator_power), and the cost of transport, power / (mass x g x speed);
   - drift: the heading's change per metre travelled along the axis, deg/m;
   - plan: the speed the firmware's odometry reckons from its own feet (stroke x cadence once steady), mm/s, and the
     servos' tracking error, the RMS over the 18 joints and the window of target less joint angle, deg.
   The simulated servo (src/sim/mj_runtime.py) is not validated against the real MG92B, so a speed lost to it here
   need not be lost on the robot.
2. Static stability margin at the walkable extremes, kinematic, no physics: the firmware walks with full stick on each
   axis (sideways both ways, forward, turning) at the neutral posture and the eight corners of the (s1, h, fd) cube,
   which the firmware clamps onto its stance's walkable region, at stride +1 (the longest stroke). Each tick its joint
   angles pose the MuJoCo model, level, and the margin is the distance from the centre of mass (the model's, legs
   included) projected on the floor to the nearest edge of the convex hull of the feet within TOUCH_MM of the lowest;
   the minimum over MARGIN_WINDOW_S of steady walking is reported, with the odometry's travel over the window and the
   plan's (classicWalkTarget's stroke x cadence / stand fraction at the clamped posture), the feet having first been
   checked onto the stance. BI_GATE has two feet down for part of its cycle, so it has no static margin by design (a
   dynamic timing) and is left out.

  uv run --no-sync python compare_stances.py            # both parts: 4 s on 8 workers, 19 s on one (2026-10-31)
  uv run --no-sync python compare_stances.py --workers 1
"""
import argparse
import ctypes
import json
import math
import os
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from itertools import product
from pathlib import Path

import mujoco
import numpy as np

from src.robot.firmware_gait import (BI_GATE, RIPPLE, STANCE_CRAB, STANCE_DEFAULT, STANCE_NAMES, TRI_GATE,
                                     WAVE)
from src.sim.firmware_loop import FIRMWARE_DT, FirmwareLoop
from src.sim.mj_runtime import HexapodSim
from src.sim.virtual_robot import STAND, WALK, VirtualRobot, native_library

GAIT_NAMES = {TRI_GATE: "TRI", BI_GATE: "BI", WAVE: "WAVE", RIPPLE: "RIPPLE"}
STATICALLY_STABLE = (TRI_GATE, WAVE, RIPPLE)
STANCES = (STANCE_DEFAULT, STANCE_CRAB)
CRAB_MIN_MARGIN_MM = 25.0  # spec 2026-10-30-crab-stance section 6

# Part 1: the stick's share of full speed, on the axis walked; +lx strafes right, +ly walks forward.
SHARES = (1 / 3, 2 / 3, 1.0)
DIRECTIONS = {"sideways": "lx", "forward": "ly"}
STAND_S = 0.5     # from the spawn to a stand
STANCE_S = 2.5    # the feet step onto the crab: two gait cycles of the 1 Hz repositioning (Robot.set_stance)
SETTLE_S = 1.5    # the walk's command ramp and the first strides
WINDOW_S = 5.0

# Part 2.
POSTURES = ((0.0, 0.0, 0.0), *product((-1.0, 1.0), repeat=3))  # (s1, h, fd)
MARGIN_AXES = {"sideways right": {"lx": 1.0}, "sideways left": {"lx": -1.0}, "forward": {"ly": 1.0},
               "turn": {"rx": 1.0}}
TOUCH_MM = 0.5         # a foot this close to the lowest is on the floor
POSTURE_S = 3.0        # stand, step onto the stance and take the posture
WALK_RAMP_S = 2.5      # the walk's start, and the settle of a posture the walk clamps further
MARGIN_WINDOW_S = 4.0  # a 10 s window moves no margin by more than 0.14 mm (measured 2026-10-31)
HEARTBEAT_TICKS = 4    # the app's 20 ms stick period, which keeps the firmware's dead-man stop off
AXIS_INDEX = {"sideways right": 0, "sideways left": 0, "forward": 1, "turn": 2}  # odometry x mm, y mm, yaw rad
# Each classic timing's share of a cycle a foot stands, by GaitType (gait_envelope.h STAND_FRAC, from its generator).
STAND_FRAC = json.loads((Path(__file__).parent / "src/resources/gait_envelope.json").read_text())["stand_frac"]

# The crab's resting coxas (gait_speed.h stanceFeet): each leg straight out across the body. The default stance's stay
# within 5 deg of 0 at every corner of the posture cube (measured 2026-10-31), so 10 deg tells the two apart.
CRAB_COXAS_DEG = np.array([-45.0, 0.0, 45.0, 45.0, 0.0, -45.0])
DEFAULT_COXA_LIMIT_DEG = 10.0


def feet_on(stance: int, angles_deg, tolerance_deg: float) -> bool:
    """Whether joint angles stand the legs on `stance`: the crab's coxas within `tolerance_deg` of their 45 deg, or
    the default stance's well short of them."""
    coxas = np.asarray(angles_deg)[0::3]
    if stance == STANCE_CRAB:
        return bool(np.all(np.abs(coxas - CRAB_COXAS_DEG) <= tolerance_deg))
    return bool(np.all(np.abs(coxas) <= DEFAULT_COXA_LIMIT_DEG))


def _require_feet_on(stance: int, angles_deg, tolerance_deg: float, what: str) -> None:
    if not feet_on(stance, angles_deg, tolerance_deg):
        coxas = np.round(np.asarray(angles_deg)[0::3], 1)
        raise RuntimeError(f"{what}: the feet do not stand on the {STANCE_NAMES[stance]} stance (coxas {coxas} deg)")


def _ticks(seconds: float) -> int:
    return int(round(seconds / FIRMWARE_DT))


def support_margin(com_xy: np.ndarray, feet_mm: np.ndarray) -> float | None:
    """The distance (mm) from `com_xy` to the nearest edge of the convex hull of the feet within TOUCH_MM of the lowest,
    negative outside it; None with fewer than three such feet, or all on one line, which hold no static balance."""
    floor = feet_mm[:, 2] <= feet_mm[:, 2].min() + TOUCH_MM
    hull = _convex_hull(feet_mm[floor, :2])
    if len(hull) < 3:
        return None
    edges = np.roll(hull, -1, axis=0) - hull
    to_com = com_xy - hull
    # Counter-clockwise hull: the cross product is the distance to the left of each edge, times the edge's length.
    inside = (edges[:, 0] * to_com[:, 1] - edges[:, 1] * to_com[:, 0]) / np.linalg.norm(edges, axis=1)
    return float(inside.min())


def _convex_hull(points: np.ndarray) -> np.ndarray:
    """Andrew's monotone chain, counter-clockwise, without collinear points."""
    pts = sorted(map(tuple, points))
    if len(pts) < 3:
        return np.array(pts)

    def half(seq):
        chain = []
        for p in seq:
            while len(chain) >= 2 and ((chain[-1][0] - chain[-2][0]) * (p[1] - chain[-2][1])
                                       - (chain[-1][1] - chain[-2][1]) * (p[0] - chain[-2][0])) <= 1e-9:
                chain.pop()
            chain.append(p)
        return chain

    lower, upper = half(pts), half(reversed(pts))
    return np.array(lower[:-1] + upper[:-1])


class LevelPose:
    """The MuJoCo model posed at a set of joint angles with its base level at the origin: the body frame (x lateral,
    y forward, z up, mm)."""

    def __init__(self):
        self.sim = HexapodSim()
        self.base = mujoco.mj_name2id(self.sim.model, mujoco.mjtObj.mjOBJ_BODY, "base")

    def __call__(self, angles_deg) -> tuple[np.ndarray, np.ndarray]:
        """The centre of mass on the floor (x, y) and the six feet (x, y, z), in mm."""
        m, d = self.sim.model, self.sim.data
        d.qpos[:] = 0.0
        d.qpos[3] = 1.0
        d.qpos[self.sim.qpos_adr] = np.radians(angles_deg)
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)
        return d.subtree_com[self.base][:2] * 1000.0, d.site_xpos[self.sim.foot_site_ids] * 1000.0


@dataclass(frozen=True)
class StaticMargin:
    stance: int
    gait: int
    posture: tuple[float, float, float]
    axis: str
    margin_mm: float                       # the least over the window; -inf where fewer than three feet held it
    travelled: tuple[float, float, float]  # the odometry's x and y (mm) and kinematic yaw (rad) over the window
    planned: tuple[float, float, float]    # the plan's over the window, likewise


def _plan(gait: int, stance: int, posture, sticks: dict, seconds: float) -> tuple[float, float, float]:
    """classicWalkTarget's walk at the posture the firmware clamps into the stance, as x and y (mm) and yaw (rad)
    over `seconds`: a planted foot travels the stroke in the stance share of each cycle, so the body travels stroke x
    cadence / stand fraction."""
    lib = native_library()
    clamped = (ctypes.c_float * 3)(*posture)
    lib.clampPosture(clamped, stance)
    s1, h, fd = clamped
    out = (ctypes.c_float * 4)()
    lib.classicWalkTarget(gait, sticks.get("lx", 0.0), sticks.get("ly", 0.0), sticks.get("rx", 0.0), 1.0, s1, h, fd,
                          1.0, stance, out)
    step_x, step_y, step_angle, rate = out
    cycles = rate * seconds / STAND_FRAC[gait]
    return step_x * cycles, step_y * cycles, step_angle * cycles


def _static_margin(job) -> StaticMargin:
    stance, gait, posture, axis = job
    robot, pose = VirtualRobot(), LevelPose()
    robot.set_imu((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), (0.0, 0.0, 0.0))
    robot.set_gait(gait)
    s1, h, fd = posture
    robot.set_mode(STAND)
    robot.set_stance(stance)
    held = dict(s1=s1, h=h, fd=fd, stride=1.0)

    def run(seconds, sticks, observe=None):
        for tick in range(_ticks(seconds)):
            if tick % HEARTBEAT_TICKS == 0:
                robot.command(**sticks)
            robot.tick()
            if observe:
                observe()

    run(POSTURE_S, held)
    _require_feet_on(stance, robot.servo_angles(), 0.5, f"{GAIT_NAMES[gait]} at {posture}")
    robot.set_mode(WALK)
    walk = {**held, **MARGIN_AXES[axis], "s": 1.0}
    run(WALK_RAMP_S, walk)
    start = robot.odometry()
    margins = []
    yaw = [start[3], 0.0]  # the last kinematic yaw, and the turn summed from tick to tick past the wrap at +-pi

    def observe():
        margins.append(support_margin(*pose(robot.servo_angles())))
        now = robot.odometry()[3]
        yaw[1] += (now - yaw[0] + math.pi) % (2 * math.pi) - math.pi
        yaw[0] = now

    run(MARGIN_WINDOW_S, walk, observe)
    least = -math.inf if None in margins else min(margins)
    end = robot.odometry()
    travelled = ((end[0] - start[0]) * 1000.0, (end[1] - start[1]) * 1000.0, yaw[1])
    return StaticMargin(stance, gait, posture, axis, least, travelled,
                        _plan(gait, stance, posture, MARGIN_AXES[axis], MARGIN_WINDOW_S))


def static_margins(stance: int, workers: int = 1) -> list[StaticMargin]:
    """The least static margin of every statically stable timing at each posture and axis of part 2, in `stance`."""
    jobs = list(product((stance,), STATICALLY_STABLE, POSTURES, MARGIN_AXES))
    if workers <= 1:
        return [_static_margin(job) for job in jobs]
    native_library()  # built once here, not by each worker at once
    with ProcessPoolExecutor(workers) as pool:
        return list(pool.map(_static_margin, jobs))


@dataclass(frozen=True)
class Walk:
    stance: int
    gait: int
    direction: str
    share: float
    speed_mm_s: float
    roll_rms_deg: float
    pitch_rms_deg: float
    com_sway_mm: float
    power_w: float
    cost_of_transport: float
    drift_deg_m: float
    plan_mm_s: float
    tracking_rms_deg: float


def _roll_pitch(quat):
    w, x, y, z = quat
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return roll, pitch


def _walk(job) -> Walk:
    stance, gait, direction, share = job
    sim = HexapodSim()
    sim.reset_to_stand()
    loop = FirmwareLoop(sim, gait)
    base = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, "base")

    def run(seconds, observe=None):
        for _ in range(_ticks(seconds)):
            loop.tick()
            if observe:
                observe()

    loop.hold()
    run(STAND_S)
    loop.select_stance(stance)
    run(STANCE_S)
    what = f"{GAIT_NAMES[gait]} before walking {direction}"
    _require_feet_on(stance, loop.robot.servo_angles(), 0.5, what)
    _require_feet_on(stance, np.degrees(sim.data.qpos[sim.qpos_adr]), 2.0, what + ", in MuJoCo")
    loop.walk(**{"lx": 0.0, "ly": 0.0, "rx": 0.0, DIRECTIONS[direction]: share}, s=1.0)
    run(SETTLE_S)

    yaw0 = sim.base_yaw()
    along, turned, yaw = 0.0, 0.0, yaw0
    position = sim.data.qpos[:2].copy()
    roll, pitch, com, power, tracking = [], [], [], [], []
    axis = 0 if direction == "sideways" else 1
    odometry0 = loop.odometry()[axis]

    def observe():
        nonlocal along, turned, yaw, position
        step = sim.data.qpos[:2] - position
        # The body faces world +Y at reset: after a turn by `yaw`, its right is (cos, sin) and its forward (-sin, cos).
        right = step[0] * math.cos(yaw) + step[1] * math.sin(yaw)
        forward = -step[0] * math.sin(yaw) + step[1] * math.cos(yaw)
        along += right if direction == "sideways" else forward
        now = sim.base_yaw()
        turned += (now - yaw + math.pi) % (2 * math.pi) - math.pi
        position, yaw = sim.data.qpos[:2].copy(), now
        r, p = _roll_pitch(sim.base_quat())
        roll.append(r)
        pitch.append(p)
        com.append(sim.data.subtree_com[base][:2].copy())
        power.append(sim.actuator_power())
        tracking.append(sim.joint_target - sim.data.qpos[sim.qpos_adr])

    run(WINDOW_S, observe)
    if sim.body_tilt_deg() > 30.0:
        raise RuntimeError(f"{STANCE_NAMES[stance]} {GAIT_NAMES[gait]} fell walking {direction}")
    speed = along / WINDOW_S
    path = np.array(com)
    heading = (path[-1] - path[0]) / np.linalg.norm(path[-1] - path[0])
    across = path @ np.array([-heading[1], heading[0]])
    ticks = np.arange(len(across))
    off_line = across - np.polyval(np.polyfit(ticks, across, 1), ticks)
    mass = mujoco.mj_getTotalmass(sim.model)
    mean_power = float(np.mean(power))
    return Walk(
        stance, gait, direction, share,
        speed_mm_s=speed * 1000.0,
        roll_rms_deg=math.degrees(float(np.std(roll))),
        pitch_rms_deg=math.degrees(float(np.std(pitch))),
        com_sway_mm=float(np.sqrt(np.mean(off_line ** 2))) * 1000.0,
        power_w=mean_power,
        cost_of_transport=mean_power / (mass * 9.81 * abs(speed)),
        drift_deg_m=math.degrees(turned) / abs(along),
        plan_mm_s=(loop.odometry()[axis] - odometry0) * 1000.0 / WINDOW_S,
        tracking_rms_deg=math.degrees(float(np.sqrt(np.mean(np.square(tracking))))),
    )


def walks(workers: int) -> list[Walk]:
    jobs = list(product(STANCES, GAIT_NAMES, DIRECTIONS, SHARES))
    native_library()
    with ProcessPoolExecutor(workers) as pool:
        return list(pool.map(_walk, jobs))


def _rounded(value: float, digits: int) -> str:
    """`value` to `digits` significant digits, as a plain number."""
    return f"{float(f'{value:.{digits}g}'):g}"


def _walk_table(rows: list[Walk]) -> str:
    """BI's rows, a dynamic timing on two feet part of each cycle, are given to two significant digits: with the window
    opened 1.5 to 6.5 s into the walk, its crab at full speed walked 89 to 117 mm/s (31 %), where its other rows and
    every TRI row moved under 3 % (measured 2026-10-31)."""
    lines = ["| direction | gait | share | stance | speed mm/s | plan mm/s | tracking RMS deg | roll RMS deg "
             "| pitch RMS deg | CoM sway mm | power W | CoT | drift deg/m |",
             "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for w in sorted(rows, key=lambda w: (list(DIRECTIONS).index(w.direction), w.gait, w.share, w.stance)):
        if w.gait == BI_GATE:
            cells = [_rounded(v, 2) for v in (w.speed_mm_s, w.plan_mm_s, w.tracking_rms_deg, w.roll_rms_deg,
                                              w.pitch_rms_deg, w.com_sway_mm, w.power_w, w.cost_of_transport,
                                              w.drift_deg_m)]
        else:
            cells = [f"{w.speed_mm_s:.0f}", f"{w.plan_mm_s:.0f}", f"{w.tracking_rms_deg:.2f}", f"{w.roll_rms_deg:.2f}",
                     f"{w.pitch_rms_deg:.2f}", f"{w.com_sway_mm:.1f}", f"{w.power_w:.2f}",
                     f"{w.cost_of_transport:.2f}", f"{w.drift_deg_m:+.1f}"]
        lines.append(f"| {w.direction} | {GAIT_NAMES[w.gait]} | {w.share:.2f} | {STANCE_NAMES[w.stance]} | "
                     + " | ".join(cells) + " |")
    return "\n".join(lines)


def _margin_table(rows: list[StaticMargin]) -> str:
    lines = ["| stance | gait | axis | neutral mm | least at the extremes mm | at posture (s1, h, fd) |",
             "|---|---|---|---|---|---|"]
    for stance, gait, axis in product(STANCES, STATICALLY_STABLE, MARGIN_AXES):
        cases = [m for m in rows if (m.stance, m.gait, m.axis) == (stance, gait, axis)]
        neutral = next(m for m in cases if m.posture == POSTURES[0])
        worst = min(cases, key=lambda m: m.margin_mm)
        lines.append(f"| {STANCE_NAMES[stance]} | {GAIT_NAMES[gait]} | {axis} | {neutral.margin_mm:.1f} | "
                     f"{worst.margin_mm:.1f} | {worst.posture} |")
    return "\n".join(lines)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--workers", type=int, default=min(8, os.cpu_count() or 1))
    args = ap.parse_args()
    print(_walk_table(walks(args.workers)))
    print()
    margins = [m for stance in STANCES for m in static_margins(stance, args.workers)]
    print(_margin_table(margins))
    crab = min((m for m in margins if m.stance == STANCE_CRAB), key=lambda m: m.margin_mm)
    print(f"\nleast crab margin {crab.margin_mm:.1f} mm ({GAIT_NAMES[crab.gait]}, {crab.axis}, posture {crab.posture});"
          f" the spec asks at least {CRAB_MIN_MARGIN_MM:.0f} mm")


if __name__ == "__main__":
    main()
