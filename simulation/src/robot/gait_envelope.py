"""The stroke geometry the gait speeds derive from, as a function of stance and posture (step height, body height, feet
distance).

The stance and swing curves depend only on their normalized progress, so the peak joint rate splits into two
gait-independent geometry factors, kappa_swing and kappa_stance (rad per unit of phase progress). A gait with stance
fraction d at cadence f then peaks at f * max(kappa_swing / (1 - d), kappa_stance / d) and walks at stroke * f / d.
The cadence is capped by the servo speed through that peak and by CADENCE_MAX; the stroke by joint travel, less
TRAVEL_MARGIN_DEG, by reach, and by the legs' clearance: every classic timing's cycle at the stroke keeps every leg pair
LEG_CLEARANCE_MM apart (src/robot/clearance.py, the check firmware/include/leg_clearance.h ports).
The factors are read off the port's own curves at sampled progress, so one posture costs two short sweeps and one
sampled cycle per timing, not a rollout.
The table keeps the factors themselves, sampled along each axis's stroke up to the longest feasible one, so a consumer
derives the top cadence of any gait at any stroke.
Each stance has its own table, and is walkable where it strides WALKABLE_STROKE_MM along the axis it walks: forward in
the default stance, sideways in the crab.
"""

import math
from concurrent.futures import ProcessPoolExecutor
from functools import lru_cache

import numpy as np

from src.robot import clearance
from src.robot import firmware_gait as fg
from src.robot.animation import JOINT_LIMIT_DEG

DEFAULT_SERVO_RAD_S = 13.1  # MG92B 0.08 s / 60 deg at the 6 V servo rail
CADENCE_MAX = 3.0  # cycles/s, the ceiling of the legacy step_speed law (2 x 1.5)
# Travel kept in reserve on every joint. The table is measured at grid points and sampled curves, and the firmware
# interpolates it between them, where the feasible stroke can fall short of the interpolated one: with the coxa's
# 50.5 deg, 0.21 deg of tibia past travel at a 2 deg margin over 400 walkable postures of each stance (every classic
# gait and axis, at the speed-optimal and the longest stride), none at this one. It does not bound every cell: over 400
# other postures the crab still takes a middle tibia 0.55 deg past travel where its lateral stroke falls from 124 to
# 29 mm across one cell (s1 -0.84, h 0.17, fd -0.30).
TRAVEL_MARGIN_DEG = 2.5
DEFAULT_SPEED_FRACTION = 0.6  # of the top cadence at s = 0; s = +1 reaches 1.0
LEG_CLEARANCE_MM = clearance.LEG_CLEARANCE_MM
# Clearance kept in reserve above LEG_CLEARANCE_MM by every stroke, for the same reason: without it the walks between
# the grid points came 0.21 mm closer than LEG_CLEARANCE_MM (measured as the travel above).
CLEARANCE_RESERVE_MM = 0.5

AXES = ("forward", "lateral", "yaw")
GAIT_NAMES = ("TRI_GATE", "BI_GATE", "WAVE", "RIPPLE")
GRID = np.linspace(-1.0, 1.0, 9)
# The axis each stance walks, on which its walkability is judged.
WALK_AXIS = {fg.STANCE_DEFAULT: "forward", fg.STANCE_CRAB: "lateral"}

# The swept strokes, past the longest any posture's travel, reach and clearance allow, so that the geometry and not the
# sweep ends every stroke.
STROKES = {
    "forward": 4.0 * np.arange(1, 51),  # mm
    "lateral": 4.0 * np.arange(1, 51),  # mm
    "yaw": 0.02 * np.arange(1, 61),  # rad
}
REFINED_STEP = {"forward": 0.5, "lateral": 0.5, "yaw": 0.005}  # resolution of the longest feasible stroke
STROKE_SAMPLES = 10  # kappa is sampled at (k + 1) / STROKE_SAMPLES of the longest feasible stroke
WALKABLE_STROKE_MM = 20.0
# Curve samples kappa and the travel are read at. Kappa, the largest joint step between two samples, understates the
# peak rate between them by about 1/n: 9.7% at 24 swing samples on the neutral posture's longest forward stroke, under
# 1% at these.
SWING_SAMPLES = 480
STANCE_SAMPLES = 240
# Phases a timing's cycle is sampled at for the legs' clearance. The timings' offsets are sixths and halves and their
# stand fractions 3.1/6, 2.1/6 and 5/6, all multiples of 1/120, so every lift-off and touchdown is a sample.
CYCLE_PHASES = 120

LIFT_MM_PER_S1 = 20.0  # lift = (s1 + 1) * 20 mm
BODY_HEIGHT_MM = 50.0  # zm = h * 50 mm
FEET_SCALE_RANGE = (0.75, 1.25)

_KIN = fg.Kinematics()


def _timing(gait_type: int) -> tuple[np.ndarray, float]:
    gait = fg.GaitState(gait_type=gait_type)
    fg.set_gait(gait)
    return gait.offset, gait.stand_frac


TIMINGS = tuple(_timing(getattr(fg, name)) for name in GAIT_NAMES)
STAND_FRAC = tuple(stand_frac for _, stand_frac in TIMINGS)
_BEZIER_WEIGHTS = np.array([math.comb(11, i) for i in range(12)], dtype=float)


def posture(s1: float, h: float, fd: float) -> tuple[float, float, float]:
    """Lift (mm), body height zm (mm) and feet scale of a grid point in [-1, 1]^3."""
    lift = (s1 + 1.0) * LIFT_MM_PER_S1
    low, high = FEET_SCALE_RANGE
    return lift, h * BODY_HEIGHT_MM, low + (fd + 1.0) / 2.0 * (high - low)


def _leg_strokes(axis: str, stroke: float, feet: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Each leg's half stroke (6,) and unit stroke direction (6, 2), as generate_feet forms them from step_x, step_y
    and step_angle."""
    if axis == "forward":
        strokes = np.tile([0.0, stroke], (6, 1))
    elif axis == "lateral":
        strokes = np.tile([stroke, 0.0], (6, 1))
    else:
        strokes = stroke * np.column_stack([-feet[:, 1], feet[:, 0]])
    angle = np.arctan2(strokes[:, 1], strokes[:, 0])
    return np.hypot(strokes[:, 0], strokes[:, 1]) / 2.0, np.column_stack([np.cos(angle), np.sin(angle)])


def _stance_offsets(half: np.ndarray, progress: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """gait.h's stanceCurve at the progress, broadcast: the step along the stroke and the press below the stance."""
    step = half * (1.0 - 2.0 * progress)
    pressed = -fg.DEFAULT_STEP_DEPTH * np.cos(np.pi * step / (2.0 * np.where(half != 0.0, half, 1.0)))
    return step, np.where(half != 0.0, pressed, 0.0)


def _swing_offsets(half: np.ndarray, lift: float, progress: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """gait.h's bezierCurve at the progress, broadcast: the step along the stroke and the lift."""
    powers = np.arange(12)
    weights = _BEZIER_WEIGHTS * progress[..., None] ** powers * (1.0 - progress[..., None]) ** (11 - powers)
    return (weights * fg.BEZIER_STEPS).sum(-1) * half, (weights * fg.BEZIER_HEIGHTS).sum(-1) * lift


def _placed(feet: np.ndarray, direction: np.ndarray, step: np.ndarray, z: np.ndarray) -> np.ndarray:
    """The resting feet moved by step along each leg's stroke direction and by z: (..., 6, 3)."""
    placed = np.empty(step.shape + (3,))
    placed[..., :2] = feet[:, :2] + step[..., None] * direction
    placed[..., 2] = feet[:, 2] + z
    return placed


def _leg_plane(feet: np.ndarray, zm: float):
    """Each foot as Kinematics.inverse_kinematics resolves it under a level, unshifted body at height zm: its offset
    from the coxa axis (dx, dy) in the leg's mount frame, and the femur-tibia pair's radial and vertical reach."""
    wx, wy = feet[..., 0] - _KIN.mount_x, feet[..., 1] - _KIN.mount_y
    dx = wx * _KIN.ca + wy * _KIN.sa - _KIN.root_j1
    dy = wx * _KIN.sa - wy * _KIN.ca
    return dx, dy, np.hypot(dx, dy) - _KIN.j1_j2, feet[..., 2] + zm


def level_inverse_kinematics(feet: np.ndarray, zm: float) -> np.ndarray:
    """Kinematics.inverse_kinematics of a level, unshifted body at height zm, broadcast over feet (..., 6, 3): the joint
    angles (..., 6, 3) in rad."""
    dx, dy, radial, vertical = _leg_plane(feet, zm)
    lr2 = radial * radial + vertical * vertical
    lr = np.sqrt(lr2)
    c1 = np.clip((lr2 + _KIN.j2_j3 ** 2 - _KIN.j3_tip ** 2) / (2 * _KIN.j2_j3 * lr), -1.0, 1.0)
    c2 = np.clip((lr2 - _KIN.j2_j3 ** 2 + _KIN.j3_tip ** 2) / (2 * _KIN.j3_tip * lr), -1.0, 1.0)
    a1, a2 = np.arccos(c1), np.arccos(c2)
    return np.stack([-np.arctan2(dy, dx), np.arctan2(vertical, radial) + a1, -(a1 + a2)], axis=-1)


def level_reachable(feet: np.ndarray, zm: float) -> np.ndarray:
    """foot_reachable (animation.py) of each foot under a level, unshifted body at height zm, broadcast over feet
    (..., 6, 3): the femur-tibia pair reaches it without either acos argument of the IK saturating."""
    _, _, radial, vertical = _leg_plane(feet, zm)
    reach = np.hypot(radial, vertical)
    return (abs(_KIN.j2_j3 - _KIN.j3_tip) <= reach) & (reach <= _KIN.j2_j3 + _KIN.j3_tip)


def _peak_rate_per_progress(angles: np.ndarray) -> float:
    return float(np.abs(np.diff(angles, axis=0)).max() * (len(angles) - 1))


def kappas(axis: str, stroke: float, lift: float, zm: float, feet: np.ndarray) -> tuple[float, float, bool]:
    """(kappa_swing, kappa_stance, within): peak joint rate per unit of swing and stance progress over the resting feet,
    and whether every sampled angle is within the joint travel less TRAVEL_MARGIN_DEG and every sampled foot within
    reach."""
    half, direction = _leg_strokes(axis, stroke, feet)
    swing_progress = np.linspace(0.0, 1.0, SWING_SAMPLES)[:, None]
    stance_progress = np.linspace(0.0, 1.0, STANCE_SAMPLES)[:, None]
    swing_feet = _placed(feet, direction, *_swing_offsets(half, lift, swing_progress))
    stance_feet = _placed(feet, direction, *_stance_offsets(half, stance_progress))
    swing, stance = level_inverse_kinematics(swing_feet, zm), level_inverse_kinematics(stance_feet, zm)
    limit = np.radians(JOINT_LIMIT_DEG - TRAVEL_MARGIN_DEG)
    within_travel = bool((np.abs(swing) <= limit).all() and (np.abs(stance) <= limit).all())
    reachable = bool(level_reachable(swing_feet, zm).all() and level_reachable(stance_feet, zm).all())
    return _peak_rate_per_progress(swing), _peak_rate_per_progress(stance), within_travel and reachable


def cycle_feet(axis: str, stroke: float, lift: float, feet: np.ndarray) -> np.ndarray:
    """The feet (timing, phase, leg, xyz) every classic timing's cycle places at CYCLE_PHASES phases for a walk at the
    stroke along the axis over the resting feet, as GaitController.generate_feet places them once the walk runs: a leg
    in stance on gait.h's stance curve, a swinging one on its Bezier curve at the lift."""
    half, direction = _leg_strokes(axis, stroke, feet)
    phases = np.arange(CYCLE_PHASES)[:, None] / CYCLE_PHASES
    placed = np.empty((len(TIMINGS), CYCLE_PHASES, 6, 3))
    for t, (offset, stand_frac) in enumerate(TIMINGS):
        phase = (phases + offset) % 1.0
        swinging = phase >= stand_frac
        stance_step, stance_z = _stance_offsets(half, phase / stand_frac)
        swing_step, swing_z = _swing_offsets(half, lift, np.where(swinging, (phase - stand_frac) / (1.0 - stand_frac),
                                                                  0.0))
        placed[t] = _placed(feet, direction, np.where(swinging, swing_step, stance_step),
                            np.where(swinging, swing_z, stance_z))
    return placed


def cycle_clearance(axis: str, stroke: float, lift: float, zm: float, feet: np.ndarray) -> float:
    """The least leg-pair surface clearance in mm over every classic timing's sampled cycle at the stroke, on the
    angles the servos reach, the IK's clipped to joint travel."""
    limit = np.radians(JOINT_LIMIT_DEG)
    angles = np.clip(level_inverse_kinematics(cycle_feet(axis, stroke, lift, feet), zm), -limit, limit)
    chains = clearance.leg_chains(_KIN, angles)
    pairs = np.array(clearance.NEIGHBOUR_PAIRS)
    centre = clearance.segment_pair_distances(chains[..., pairs[:, 0], :, :], chains[..., pairs[:, 1], :, :])
    return float(clearance.surface_clearance(centre).min())


def geometry_factors(axis: str, stroke: float, lift: float, zm: float, feet_scale: float,
                     stance: int = fg.STANCE_DEFAULT) -> tuple[float, float, bool]:
    """(kappa_swing, kappa_stance, feasible) over the stance's resting feet: the peak joint rate per unit of swing and
    stance progress, and whether the stroke keeps every sampled angle within the joint travel less TRAVEL_MARGIN_DEG,
    every sampled foot within reach, and every leg pair LEG_CLEARANCE_MM apart, with CLEARANCE_RESERVE_MM to spare,
    over every classic timing's cycle."""
    feet = fg.stance_feet(stance, feet_scale)
    kappa_swing, kappa_stance, within = kappas(axis, stroke, lift, zm, feet)
    feasible = within and cycle_clearance(axis, stroke, lift, zm, feet) >= LEG_CLEARANCE_MM + CLEARANCE_RESERVE_MM
    return kappa_swing, kappa_stance, feasible


@lru_cache(maxsize=None)
def _feasible_strokes(axis: str, s1: float, h: float, fd: float,
                      stance: int = fg.STANCE_DEFAULT) -> tuple[tuple[float, float, float], ...]:
    """(stroke, kappa_swing, kappa_stance) for each stroke up to the first infeasible one."""
    lift, zm, feet_scale = posture(s1, h, fd)
    feasible = []
    for stroke in STROKES[axis]:
        kappa_swing, kappa_stance, ok = geometry_factors(axis, float(stroke), lift, zm, feet_scale, stance)
        if not ok:
            break
        feasible.append((float(stroke), kappa_swing, kappa_stance))
    return tuple(feasible)


@lru_cache(maxsize=None)
def longest_stroke(axis: str, s1: float, h: float, fd: float, stance: int = fg.STANCE_DEFAULT) -> float:
    """The longest feasible stroke: the sweep's last feasible one, refined to REFINED_STEP by bisection. 0 when no
    stroke is feasible."""
    feasible = _feasible_strokes(axis, s1, h, fd, stance)
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
        if geometry_factors(axis, middle * step, lift, zm, feet_scale, stance)[2]:
            low = middle
        else:
            high = middle
    return low * step


def is_walkable(s1: float, h: float, fd: float, stance: int = fg.STANCE_DEFAULT) -> bool:
    """Whether the posture can stride WALKABLE_STROKE_MM along the axis the stance walks within joint travel, reach and
    the legs' clearance."""
    strokes = _feasible_strokes(WALK_AXIS[stance], float(s1), float(h), float(fd), stance)
    return bool(strokes) and strokes[-1][0] >= WALKABLE_STROKE_MM


def _geometry_samples(axis: str, s1: float, h: float, fd: float, stance: int) -> tuple[float, list[float], list[float]]:
    """(longest stroke, kappa_swing, kappa_stance) at STROKE_SAMPLES strokes up to it."""
    stroke_max = longest_stroke(axis, s1, h, fd, stance)
    if stroke_max == 0.0:
        return 0.0, [0.0] * STROKE_SAMPLES, [0.0] * STROKE_SAMPLES
    lift, zm, feet_scale = posture(s1, h, fd)
    feet = fg.stance_feet(stance, feet_scale)
    factors = [kappas(axis, stroke_max * (k + 1) / STROKE_SAMPLES, lift, zm, feet) for k in range(STROKE_SAMPLES)]
    return stroke_max, [f[0] for f in factors], [f[1] for f in factors]


def _round_cells(cells: list, field: int) -> list:
    """One field of the [s1][h][fd] cells of _geometry_samples, rounded to the 4 decimals the header carries."""
    return [[[np.round(cell[field], 4).tolist() for cell in row] for row in plane] for plane in cells]


def _plane(stance: int, s1: float) -> tuple[list, dict]:
    """One s1 plane of a stance's envelope: walkable, and the geometry samples per axis, each indexed [h][fd]."""
    walkable = [[is_walkable(s1, h, fd, stance) for fd in GRID] for h in GRID]
    geometry = {axis: [[_geometry_samples(axis, s1, h, fd, stance) for fd in GRID] for h in GRID] for axis in AXES}
    return walkable, geometry


def build_table(servo_rad_s: float) -> dict:
    """The envelope of every stance over the posture grid, keyed by its name in fg.STANCE_NAMES and indexed
    [s1][h][fd]; the geometry samples add [k] for the stroke. The s1 planes are independent and swept in parallel."""
    jobs = [(stance, float(s1)) for stance in range(len(fg.STANCE_NAMES)) for s1 in GRID]
    with ProcessPoolExecutor() as pool:
        planes = list(pool.map(_plane, *zip(*jobs)))
    stances = {}
    for stance, name in enumerate(fg.STANCE_NAMES):
        walkable, geometry = zip(*planes[stance * len(GRID):(stance + 1) * len(GRID)])
        cells = {axis: [plane[axis] for plane in geometry] for axis in AXES}
        stances[name] = {
            "walk_axis": WALK_AXIS[stance],
            "walkable": list(walkable),
            "stroke_max": {axis: _round_cells(cells[axis], 0) for axis in AXES},
            "kappa_swing": {axis: _round_cells(cells[axis], 1) for axis in AXES},
            "kappa_stance": {axis: _round_cells(cells[axis], 2) for axis in AXES},
        }
    return {
        "grid": [float(g) for g in GRID],
        "servo_rad_s": servo_rad_s,
        "cadence_max": CADENCE_MAX,
        "travel_margin_deg": TRAVEL_MARGIN_DEG,
        "leg_clearance_mm": LEG_CLEARANCE_MM,
        "link_radius_mm": dict(clearance.LINK_RADIUS_MM),
        "clearance_reserve_mm": CLEARANCE_RESERVE_MM,
        "default_speed_fraction": DEFAULT_SPEED_FRACTION,
        "stand_frac": list(STAND_FRAC),
        "stances": stances,
    }
