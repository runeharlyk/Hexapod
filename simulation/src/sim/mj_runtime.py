"""Shared MuJoCo runtime for the hexapod, reused by replay + the RL env.

Verified facts:
  - Actuator order is [coxa_i, femur_i, tibia_i for i in 0..5] which is exactly the
    flatten order of Kinematics.inverse_kinematics(...).reshape(6,3).
  - The MJCF joint convention matches the firmware IK 1:1 (sign mapping (1,1,1),
    0.0 mm foot error), so ctrl[joint] = ik_angle_in_radians directly.
  - Physics timestep 0.002 s; policy/gait control at 0.02 s -> frame_skip = 10.
"""

import os
import numpy as np
import mujoco

from src.robot.firmware_gait import Kinematics, BodyState

MODEL_PATH = os.path.join(os.path.dirname(__file__), "..", "resources", "model.xml")
TERRAIN_MODEL_PATH = os.path.join(os.path.dirname(__file__), "..", "resources", "model_terrain.xml")
CONTROL_DT = 0.02
JOINT_NAMES = [f"{j}_{i}" for i in range(6) for j in ("coxa", "femur", "tibia")]


class HexapodSim:
    """Thin wrapper: loads the model, maps IK angles (rad) to actuators, steps physics."""

    def __init__(self, model_path: str = MODEL_PATH):
        self.model = mujoco.MjModel.from_xml_path(os.path.abspath(model_path))
        self.data = mujoco.MjData(self.model)
        self.kin = Kinematics()
        self.frame_skip = int(round(CONTROL_DT / self.model.opt.timestep))

        self.act_ids = np.array(
            [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in JOINT_NAMES]
        )
        self.qpos_adr = np.array(
            [
                self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)]
                for n in JOINT_NAMES
            ]
        )
        self.imu_site = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "imu")
        self.foot_site_ids = np.array(
            [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, f"foot_{i}") for i in range(6)]
        )
        self.foot_geom_ids = np.array(
            [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f"foot_{i}") for i in range(6)]
        )
        self.ground_geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "ground")

    @property
    def stand_pose(self) -> np.ndarray:
        """18 joint angles (rad) for the resting/standby pose."""
        return self.kin.inverse_kinematics(BodyState(), degrees=False)

    def reset_to_stand(self):
        mujoco.mj_resetData(self.model, self.data)
        stand = self.stand_pose
        self.data.qpos[self.qpos_adr] = stand
        mujoco.mj_forward(self.model, self.data)
        self.data.ctrl[self.act_ids] = stand

    def set_joint_targets(self, angles_rad: np.ndarray):
        self.data.ctrl[self.act_ids] = angles_rad

    def step_physics(self, n: int | None = None):
        for _ in range(n if n is not None else self.frame_skip):
            mujoco.mj_step(self.model, self.data)

    def body_targets_from_feet(self, body: BodyState) -> np.ndarray:
        return self.kin.inverse_kinematics(body, degrees=False)

    # --- hardware-available state (for the RL observation) ---
    def base_quat(self) -> np.ndarray:
        return self.data.qpos[3:7].copy()  # w, x, y, z

    def base_height(self) -> float:
        return float(self.data.qpos[2])

    def gyro(self) -> np.ndarray:
        return self.data.sensordata[
            self._sensor_slice("imu_gyro")
        ].copy()

    def _sensor_slice(self, name):
        sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name)
        adr = self.model.sensor_adr[sid]
        dim = self.model.sensor_dim[sid]
        return slice(adr, adr + dim)

    def actuator_power(self) -> float:
        """Mechanical power |torque * joint_velocity| summed over actuators (W).

        Penalizing this is the cost-of-transport signal that makes the policy trade step length
        against step frequency / gait pattern efficiently (fast cadence -> high joint speeds -> power)."""
        return float(np.sum(np.abs(self.data.actuator_force * self.data.actuator_velocity)))

    def foot_slip_sq(self) -> float:
        """Sum of squared horizontal foot speed over feet currently touching the ground.

        Penalizing this discourages feet sliding while planted -> cleaner stepping, better
        transfer (slipping in sim is a classic sim-only exploit).
        """
        d = self.data
        contact_feet = set()
        for c in range(d.ncon):
            g1, g2 = d.contact[c].geom1, d.contact[c].geom2
            for i, fg in enumerate(self.foot_geom_ids):
                if (g1 == fg and g2 == self.ground_geom_id) or (g2 == fg and g1 == self.ground_geom_id):
                    contact_feet.add(i)
        total = 0.0
        res = np.zeros(6)
        for i in contact_feet:
            mujoco.mj_objectVelocity(
                self.model, d, mujoco.mjtObj.mjOBJ_SITE, int(self.foot_site_ids[i]), res, 0
            )
            total += res[3] ** 2 + res[4] ** 2  # world-frame x,y linear velocity
        return float(total)
