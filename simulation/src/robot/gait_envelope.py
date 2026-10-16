"""The stroke geometry the gait speeds derive from, as a function of posture (step height, body height, feet distance).

The stance and swing curves depend only on their normalized progress, so the peak joint rate splits into two
gait-independent geometry factors, kappa_swing and kappa_stance (rad per unit of phase progress). A gait with stance
fraction d at cadence f then peaks at f * max(kappa_swing / (1 - d), kappa_stance / d) and walks at stroke * f / d.
The cadence is capped by the servo speed through that peak and by CADENCE_MAX; the stroke by joint travel, less
TRAVEL_MARGIN_DEG, and reach.
The factors are read off the port's own curves at sampled progress, so one posture costs two short sweeps, not a rollout.
The table keeps the factors themselves, sampled along each axis's stroke up to the longest feasible one, so a consumer
derives the top cadence of any gait at any stroke.
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
DEFAULT_SPEED_FRACTION = 0.6  # of the top cadence at s = 0; s = +1 reaches 1.0

AXES = ("forward", "lateral", "yaw")
GAIT_NAMES = ("TRI_GATE", "BI_GATE", "WAVE", "RIPPLE")
GRID = np.linspace(-1.0, 1.0, 9)

STROKES = {
    "forward": 4.0 * np.arange(1, 26),  # mm
    "lateral": 4.0 * np.arange(1, 26),  # mm
    "yaw": 0.02 * np.arange(1, 41),  # rad
}
REFINED_STEP = {"forward": 0.5, "lateral": 0.5, "yaw": 0.005}  # resolution of the longest feasible stroke
STROKE_SAMPLES = 10  # kappa is sampled at (k + 1) / STROKE_SAMPLES of the longest feasible stroke
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


@lru_cache(maxsize=None)
def longest_stroke(axis: str, s1: float, h: float, fd: float) -> float:
    """The longest feasible stroke: the sweep's last feasible one, refined to REFINED_STEP by bisection. 0 when no
    stroke is feasible."""
    feasible = _feasible_strokes(axis, s1, h, fd)
    if not feasible:
        return 0.0
    step = REFINED_STEP[axis]
    low = round(feasible[-1][0] / step)
    if feasible[-1][0] == STROKES[axis][-1]:
        return low * step
    high = low + round((STROKES[axis][1] - STROKES[axis][0]) / step)
    lift, zm, feet_scale = posture(s1, h, fd)
    while high - low > 1:
        middle = (low + high) // 2
        if geometry_factors(axis, middle * step, lift, zm, feet_scale)[2]:
            low = middle
        else:
            high = middle
    return low * step


def is_walkable(s1: float, h: float, fd: float) -> bool:
    """Whether the posture can stride WALKABLE_STROKE_MM forward within joint travel and reach."""
    strokes = _feasible_strokes("forward", float(s1), float(h), float(fd))
    return bool(strokes) and strokes[-1][0] >= WALKABLE_STROKE_MM


def _geometry_samples(axis: str, s1: float, h: float, fd: float) -> tuple[float, list[float], list[float]]:
    """(longest stroke, kappa_swing, kappa_stance) at STROKE_SAMPLES strokes up to it."""
    stroke_max = longest_stroke(axis, s1, h, fd)
    if stroke_max == 0.0:
        return 0.0, [0.0] * STROKE_SAMPLES, [0.0] * STROKE_SAMPLES
    lift, zm, feet_scale = posture(s1, h, fd)
    factors = [geometry_factors(axis, stroke_max * (k + 1) / STROKE_SAMPLES, lift, zm, feet_scale)
               for k in range(STROKE_SAMPLES)]
    return stroke_max, [f[0] for f in factors], [f[1] for f in factors]


def _round_cells(cells: list, field: int) -> list:
    """One field of the [s1][h][fd] cells of _geometry_samples, rounded to the 4 decimals the header carries."""
    return [[[np.round(cell[field], 4).tolist() for cell in row] for row in plane] for plane in cells]


def build_table(servo_rad_s: float) -> dict:
    """The envelope over the posture grid, indexed [s1][h][fd]; the geometry samples add [k] for the stroke."""
    walkable = [[[is_walkable(s1, h, fd) for fd in GRID] for h in GRID] for s1 in GRID]
    geometry = {axis: [[[_geometry_samples(axis, s1, h, fd) for fd in GRID] for h in GRID] for s1 in GRID]
                for axis in AXES}
    return {
        "grid": [float(g) for g in GRID],
        "servo_rad_s": servo_rad_s,
        "cadence_max": CADENCE_MAX,
        "travel_margin_deg": TRAVEL_MARGIN_DEG,
        "default_speed_fraction": DEFAULT_SPEED_FRACTION,
        "stand_frac": list(STAND_FRAC),
        "walkable": walkable,
        "stroke_max": {axis: _round_cells(cells, 0) for axis, cells in geometry.items()},
        "kappa_swing": {axis: _round_cells(cells, 1) for axis, cells in geometry.items()},
        "kappa_stance": {axis: _round_cells(cells, 2) for axis, cells in geometry.items()},
    }
