"""Fraction of a gait's commanded joint angles that fall outside the servo range.

Split from ik_feasibility.py so search workers can import it without pulling in argparse/CLI code,
and so the joint ranges are loaded once per process rather than per candidate.
"""

import mujoco
import numpy as np

from src.envs.hexapod_mj_env import PG_HEIGHT
from src.robot.firmware_gait import BodyState, GaitController, GaitState, Kinematics
from src.sim.mj_runtime import JOINT_NAMES, HexapodSim

_RANGES = None
_KIN = None


def _ranges():
    global _RANGES, _KIN
    if _RANGES is None:
        sim = HexapodSim()
        _RANGES = np.array([sim.model.jnt_range[
            mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in JOINT_NAMES])
        _KIN = Kinematics()
    return _RANGES, _KIN


def out_of_range_fraction(sched, stride_mm=60.0, samples=60) -> float:
    ranges, kin = _ranges()
    body = BodyState()
    body.zm = -sched.ride_mm
    gait = GaitState()
    gait.step_y = stride_mm
    gait.step_height = float(np.interp(sched.step_height, [-1, 1], PG_HEIGHT))
    gait.stand_frac = sched.duty
    gait.offset = sched.offsets()
    gait.step_depth = sched.step_depth
    gc = GaitController()
    bad = 0
    for k in range(samples):
        gc.set_phase(k / samples)
        gc.generate_feet(gait, body)
        ang = kin.inverse_kinematics(body, degrees=False)
        bad += int(np.count_nonzero((ang < ranges[:, 0]) | (ang > ranges[:, 1])))
    return bad / float(samples * 18)
