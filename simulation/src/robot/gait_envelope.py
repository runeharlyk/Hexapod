"""Top body speed per gait as a function of posture (step height, body height, feet distance).

The stance and swing curves depend only on their normalized progress, so the peak joint rate splits into two
gait-independent geometry factors, kappa_swing and kappa_stance (rad per unit of phase progress). A gait with stance
fraction d at cadence f then peaks at f * max(kappa_swing / (1 - d), kappa_stance / d) and walks at stroke * f / d.
The cadence is capped by the servo speed through that peak and by CADENCE_MAX; the stroke by joint travel, less
TRAVEL_MARGIN_DEG, and reach.
The factors are read off the port's own curves at sampled progress, so one posture costs two short sweeps, not a rollout.
"""

from functools import lru_cache

import numpy as np

from src.robot import firmware_gait as fg
from src.robot.animation import JOINT_LIMIT_DEG, foot_reachable

DEFAULT_SERVO_RAD_S = 13.1  # MG92B 0.08 s / 60 deg at the 6 V servo rail
CADENCE_MAX = 3.0  # cycles/s, the ceiling of the legacy step_speed law (2 x 1.5)
# Travel kept in reserve on every joint. The table is measured at grid points and sampled curves, and the firmware
# interpolates it between them, where the feasible stroke can fall short of the interpolated one: up to 1.05 deg of
# tibia past travel without a margin (measured over 400 walkable postures).
TRAVEL_MARGIN_DEG = 2.0
DEFAULT_SPEED_FRACTION = 0.6  # of v_max at s = 0; s = +1 reaches 1.0

AXES = ("forward", "lateral", "yaw")
GAIT_NAMES = ("TRI_GATE", "BI_GATE", "WAVE", "RIPPLE")
GRID = np.linspace(-1.0, 1.0, 9)

STROKES = {
    "forward": 4.0 * np.arange(1, 26),  # mm
    "lateral": 4.0 * np.arange(1, 26),  # mm
    "yaw": 0.02 * np.arange(1, 41),  # rad
}
WALKABLE_STROKE_MM = 20.0
SWING_SAMPLES = 24
STANCE_SAMPLES = 12

LIFT_MM_PER_S1 = 20.0  # lift = (s1 + 1) * 20 mm
BODY_HEIGHT_MM = 50.0  # zm = h * 50 mm
FEET_SCALE_RANGE = (0.75, 1.25)

_KIN = fg.Kinematics()


def _stand_frac(gait_type: int) -> float:
    gait = fg.GaitState(gait_type=gait_type)
    fg.set_gait(gait)
    return gait.stand_frac


STAND_FRAC = tuple(_stand_frac(getattr(fg, name)) for name in GAIT_NAMES)


def posture(s1: float, h: float, fd: float) -> tuple[float, float, float]:
    """Lift (mm), body height zm (mm) and feet scale of a grid point in [-1, 1]^3."""
    lift = (s1 + 1.0) * LIFT_MM_PER_S1
    low, high = FEET_SCALE_RANGE
    return lift, h * BODY_HEIGHT_MM, low + (fd + 1.0) / 2.0 * (high - low)


def _leg_strokes(axis: str, stroke: float, feet: np.ndarray) -> np.ndarray:
    """Per-leg (x, y) stroke vector, as generate_feet forms it from step_x, step_y and step_angle."""
    if axis == "forward":
        return np.tile([0.0, stroke], (6, 1))
    if axis == "lateral":
        return np.tile([stroke, 0.0], (6, 1))
    return stroke * np.column_stack([-feet[:, 1], feet[:, 0]])


def _sample_joints(curve, amplitude: float, strokes: np.ndarray, feet: np.ndarray, zm: float, progress: np.ndarray):
    """Joint angles (samples, leg, joint) in rad along one curve, and whether every sampled foot is reachable."""
    lengths = np.hypot(strokes[:, 0], strokes[:, 1])
    directions = np.arctan2(strokes[:, 1], strokes[:, 0])
    body6 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, zm])
    angles = np.empty((len(progress), 6, 3))
    reachable = True
    for k, p in enumerate(progress):
        placed = feet.copy()
        for leg in range(6):
            delta = [0.0, 0.0, 0.0]
            curve(lengths[leg] / 2.0, directions[leg], amplitude, p, delta)
            placed[leg, :3] += delta
            reachable = reachable and foot_reachable(_KIN, body6, np.array(delta), leg, feet)
        angles[k] = _KIN.inverse_kinematics(fg.BodyState(zm=zm, feet=placed), degrees=False).reshape(6, 3)
    return angles, reachable


def _peak_rate_per_progress(angles: np.ndarray) -> float:
    return float(np.abs(np.diff(angles, axis=0)).max() * (len(angles) - 1))


def geometry_factors(axis: str, stroke: float, lift: float, zm: float, feet_scale: float) -> tuple[float, float, bool]:
    """(kappa_swing, kappa_stance, feasible): peak joint rate per unit of swing and stance progress, and whether
    every sampled angle is within the joint travel less TRAVEL_MARGIN_DEG and every sampled foot within reach."""
    feet = fg.DEFAULT_FEET.copy()
    feet[:, :2] *= feet_scale
    strokes = _leg_strokes(axis, stroke, feet)
    swing, swing_reachable = _sample_joints(
        fg.GaitController._bezier_curve, lift, strokes, feet, zm, np.linspace(0.0, 1.0, SWING_SAMPLES))
    stance, stance_reachable = _sample_joints(
        fg.GaitController._stance_curve, -fg.DEFAULT_STEP_DEPTH, strokes, feet, zm,
        np.linspace(0.0, 1.0, STANCE_SAMPLES))
    limit = np.radians(JOINT_LIMIT_DEG - TRAVEL_MARGIN_DEG)
    within_travel = bool((np.abs(swing) <= limit).all() and (np.abs(stance) <= limit).all())
    feasible = within_travel and swing_reachable and stance_reachable
    return _peak_rate_per_progress(swing), _peak_rate_per_progress(stance), feasible


@lru_cache(maxsize=None)
def _feasible_strokes(axis: str, s1: float, h: float, fd: float) -> tuple[tuple[float, float, float], ...]:
    """(stroke, kappa_swing, kappa_stance) for each stroke up to the first infeasible one."""
    lift, zm, feet_scale = posture(s1, h, fd)
    feasible = []
    for stroke in STROKES[axis]:
        kappa_swing, kappa_stance, ok = geometry_factors(axis, float(stroke), lift, zm, feet_scale)
        if not ok:
            break
        feasible.append((float(stroke), kappa_swing, kappa_stance))
    return tuple(feasible)


def is_walkable(s1: float, h: float, fd: float) -> bool:
    """Whether the posture can stride WALKABLE_STROKE_MM forward within joint travel and reach."""
    strokes = _feasible_strokes("forward", float(s1), float(h), float(fd))
    return bool(strokes) and strokes[-1][0] >= WALKABLE_STROKE_MM


def top_speed(gait_type: int, axis: str, s1: float, h: float, fd: float, servo_rad_s: float) -> tuple[float, float]:
    """(v_max, stroke): the fastest body speed over the feasible strokes (mm/s, or rad/s for yaw), and the stroke
    that reaches it. (0, 0) when no stroke is feasible."""
    d = STAND_FRAC[gait_type]
    best = (0.0, 0.0)
    for stroke, kappa_swing, kappa_stance in _feasible_strokes(axis, float(s1), float(h), float(fd)):
        cadence = min(servo_rad_s / max(kappa_swing / (1.0 - d), kappa_stance / d), CADENCE_MAX)
        speed = stroke * cadence / d
        if speed > best[0]:
            best = (speed, stroke)
    return best


def build_table(servo_rad_s: float) -> dict:
    """The envelope over the posture grid, indexed [s1][h][fd]."""
    n = len(GRID)
    walkable = [[[is_walkable(s1, h, fd) for fd in GRID] for h in GRID] for s1 in GRID]
    gaits = {}
    for gait_type, name in enumerate(GAIT_NAMES):
        gaits[name] = {"stand_frac": STAND_FRAC[gait_type]}
        for axis in AXES:
            cells = [[[top_speed(gait_type, axis, s1, h, fd, servo_rad_s) for fd in GRID] for h in GRID]
                     for s1 in GRID]
            gaits[name][axis] = {
                "v_max": [[[round(cells[i][j][k][0], 4) for k in range(n)] for j in range(n)] for i in range(n)],
                "stroke": [[[round(cells[i][j][k][1], 4) for k in range(n)] for j in range(n)] for i in range(n)],
            }
    return {
        "grid": [float(g) for g in GRID],
        "servo_rad_s": servo_rad_s,
        "cadence_max": CADENCE_MAX,
        "travel_margin_deg": TRAVEL_MARGIN_DEG,
        "default_speed_fraction": DEFAULT_SPEED_FRACTION,
        "walkable": walkable,
        "gaits": gaits,
    }
