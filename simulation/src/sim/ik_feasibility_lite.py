"""Fraction of a gait's commanded joint angles that fall outside the servo range.

The gait sampler shared with ik_feasibility.py lives here, so search workers can import it without
pulling in argparse/CLI code, and so the joint ranges are loaded once per process rather than per
candidate.
"""

import mujoco
import numpy as np

from src.envs.hexapod_mj_env import PG_HEIGHT
from src.robot.firmware_gait import BodyState, GaitController, GaitState, Kinematics
from src.sim.mj_runtime import JOINT_NAMES, HexapodSim

_RANGES = None
_KIN = None


def joint_ranges() -> np.ndarray:
    """(18, 2) model joint ranges in radians, ordered as JOINT_NAMES."""
    global _RANGES
    if _RANGES is None:
        sim = HexapodSim()
        _RANGES = np.array([sim.model.jnt_range[
            mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in JOINT_NAMES])
    return _RANGES


def lift_mm(sched) -> float:
    return float(np.interp(sched.step_height, [-1, 1], PG_HEIGHT))


def sample_joint_angles(sched, stride_mm=60.0, samples=60) -> np.ndarray:
    """(samples, 18) IK joint angles in radians over one gait cycle, walking forward at stride_mm."""
    global _KIN
    if _KIN is None:
        _KIN = Kinematics()
    body = BodyState()
    body.zm = -sched.ride_mm
    gait = GaitState()
    gait.step_y = stride_mm
    gait.step_height = lift_mm(sched)
    gait.stand_frac = sched.duty
    gait.offset = sched.offsets()
    gait.step_depth = sched.step_depth
    gc = GaitController()
    angles = np.empty((samples, 18))
    for k in range(samples):
        gc.set_phase(k / samples)
        gc.generate_feet(gait, body)
        angles[k] = _KIN.inverse_kinematics(body, degrees=False)
    return angles


def out_of_range_fraction(sched, stride_mm=60.0, samples=60) -> float:
    ranges = joint_ranges()
    ang = sample_joint_angles(sched, stride_mm, samples)
    return float(np.count_nonzero((ang < ranges[:, 0]) | (ang > ranges[:, 1]))) / ang.size
