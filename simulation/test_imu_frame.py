"""The simulated IMU against the robot's: the MPU6050 DMP's angle signs (firmware/include/imu_frame.h), what the
simulation reports in its place, and what the policy sees after the firmware turns them back."""
import math
import re
from pathlib import Path

import numpy as np

from src.envs.hexapod_mj_env import BaseGait, firmware_policy_params
from src.sim.firmware_loop import imu_reading
from src.sim.firmware_policy import DMP_ANGLE_SIGNS, FirmwarePolicy, dmp_angles

IMU_FRAME_H = Path(__file__).resolve().parents[1] / "firmware" / "include" / "imu_frame.h"
TILT = math.radians(15)


def rotation(axis, angle):
    """The base quaternion (w, x, y, z) of a right-handed rotation about a body axis."""
    q = np.zeros(4)
    q[0] = math.cos(angle / 2)
    q[1 + "xyz".index(axis)] = math.sin(angle / 2)
    return q


def test_the_dmp_signs_are_the_firmwares():
    text = IMU_FRAME_H.read_text()
    signs = tuple(float(re.search(rf"{name}_SIGN = ([+-][\d.]+)f;", text).group(1)) for name in ("ROLL", "PITCH", "YAW"))
    assert signs == DMP_ANGLE_SIGNS


# Read on the robot on 2026-10-16: right side down 17.7 deg gave pitch -17.7 deg and world up (-0.30, 0, 0.95); nose
# down 12 deg gave roll -12.0 deg and world up (-0.03, -0.21, 0.98); turning counter-clockwise, yaw fell.
def test_the_simulated_imu_reads_as_the_robot_did():
    (roll, pitch, _), up, _ = imu_reading(rotation("y", TILT), np.zeros(3))  # +x, the right side, down
    assert math.isclose(pitch, -TILT, abs_tol=1e-9) and abs(roll) < 1e-9
    assert up[0] < -0.25 and abs(up[1]) < 1e-9

    (roll, pitch, _), up, _ = imu_reading(rotation("x", -TILT), np.zeros(3))  # +y, the nose, down
    assert math.isclose(roll, -TILT, abs_tol=1e-9) and abs(pitch) < 1e-9
    assert up[1] < -0.25

    (_, _, yaw), _, _ = imu_reading(rotation("z", TILT), np.zeros(3))  # counter-clockwise seen from above
    assert yaw < 0


def test_the_policy_sees_the_simulations_own_angles():
    policy = FirmwarePolicy(firmware_policy_params("residual_gait", 1, BaseGait(None)))
    policy.reset()
    rpy = (0.1, 0.05, -0.2)
    obs = policy.observe([0.0, 0.0, 1.0], [0.0, 0.0, 0.0], dmp_angles(rpy), 0.02)
    np.testing.assert_allclose(obs[6:9], rpy, atol=1e-6)
