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

# --- servo model (MG92B) ---
# Motor (torque) actuators driven HERE by a cascaded position->velocity->torque loop, mimicking a hobby
# servo's internal controller: position error sets a target velocity (capped at no-load speed), and a
# stiff velocity loop applies up to stall torque to reach it -- full power on any error, a natural
# top-speed cap, firm non-backdrivable holding, and the velocity loop damps position-PD shaking.
SERVO_KP = 250.0      # position->velocity gain (1/s): commands full no-load speed at ~3 deg error
SERVO_KV = 1.5        # velocity->torque gain (N.m per rad/s): saturates to stall on small vel error
SERVO_J_EFF = 0.004   # kg.m^2, reflected joint inertia; sets the torque-limited STOPPING profile so the
                      # joint brakes to the target with no overshoot/ringing
SERVO_STALL = 0.50    # N.m torque envelope, flat (available at any speed); calibrated to observed behaviour
SERVO_NOLOAD = 16.0   # rad/s no-load speed cap (MG92B @6V); the joint's top slew speed


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
        self.qvel_adr = np.array(
            [
                self.model.jnt_dofadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)]
                for n in JOINT_NAMES
            ]
        )
        # servo params (mutable so domain randomization can scale them per episode)
        self.kp, self.kv = SERVO_KP, SERVO_KV
        self.stall, self.noload = SERVO_STALL, SERVO_NOLOAD
        self.j_eff = SERVO_J_EFF
        self.deadband = 0.0   # rad: gear-lash / PWM deadband -- no corrective torque within this of target
        self.joint_target = self.stand_pose.copy()
        self.imu_site = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "imu")
        self.foot_site_ids = np.array(
            [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, f"foot_{i}") for i in range(6)]
        )
        self.foot_geom_ids = np.array(
            [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f"foot_{i}") for i in range(6)]
        )
        self.ground_geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
        # chassis box: for measuring jump height from the LOWEST point of the body (not the center)
        self.chassis_geom = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "chassis")
        self.chassis_half = self.model.geom_size[self.chassis_geom].copy()
        self._box_corners = self.chassis_half * np.array(
            [[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float)

    @property
    def stand_pose(self) -> np.ndarray:
        """18 joint angles (rad) for the resting/standby pose."""
        return self.kin.inverse_kinematics(BodyState(), degrees=False)

    @property
    def joint_limits(self) -> tuple[np.ndarray, np.ndarray]:
        """(lo, hi) hinge limits (rad) for the 18 joints, in JOINT_NAMES order."""
        ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in JOINT_NAMES]
        r = self.model.jnt_range[ids]
        return r[:, 0].copy(), r[:, 1].copy()

    def set_servo_scale(self, kp=1.0, stall=1.0, noload=1.0):
        """Domain-randomization hook: scale servo params for the episode."""
        self.kp = SERVO_KP * kp
        self.stall = SERVO_STALL * stall
        self.noload = SERVO_NOLOAD * noload

    def reset_to_stand(self):
        mujoco.mj_resetData(self.model, self.data)
        stand = self.stand_pose
        self.data.qpos[self.qpos_adr] = stand
        self.joint_target = stand.copy()
        mujoco.mj_forward(self.model, self.data)
        self.data.ctrl[self.act_ids] = 0.0  # motor actuators: ctrl is torque, set by the servo loop

    def set_joint_targets(self, angles_rad: np.ndarray):
        self.joint_target = np.asarray(angles_rad, dtype=np.float64).copy()

    def _apply_servo(self):
        """PD position loop with a FLAT torque envelope + a speed cap (per physics step).
        Up to +/-stall torque is available at any speed (unlike a raw DC motor's torque-speed droop);
        the servo just can't accelerate past its no-load speed."""
        q = self.data.qpos[self.qpos_adr]
        qd = self.data.qvel[self.qvel_adr]
        err = self.joint_target - q
        if self.deadband > 0.0:  # gear lash / PWM deadband: shrink error toward 0 -> free play near target
            err = np.sign(err) * np.maximum(np.abs(err) - self.deadband, 0.0)
        # cascaded servo: position error -> target velocity -> stiff velocity loop -> torque (<=stall).
        # The target velocity is capped by (a) no-load speed (back-EMF) and (b) the max speed from which
        # the joint can still brake to a stop at the target with stall torque -> NO overshoot/ringing.
        # target velocity capped by no-load speed AND by the max speed from which stall torque can still
        # brake to a stop at the target -> no overshoot/ringing
        ae = np.abs(err)
        v_stop = np.sqrt(2.0 * (self.stall / self.j_eff) * ae)  # torque-limited stopping profile
        vel_des = np.sign(err) * np.minimum(self.kp * ae, np.minimum(v_stop, self.noload))
        tau = np.clip(self.kv * (vel_des - qd), -self.stall, self.stall)
        self.data.ctrl[self.act_ids] = tau

    def step_physics(self, n: int | None = None):
        for _ in range(n if n is not None else self.frame_skip):
            self._apply_servo()  # recompute torque each substep (PD runs at the physics rate)
            mujoco.mj_step(self.model, self.data)

    def body_targets_from_feet(self, body: BodyState) -> np.ndarray:
        return self.kin.inverse_kinematics(body, degrees=False)

    # --- hardware-available state (for the RL observation) ---
    def base_quat(self) -> np.ndarray:
        return self.data.qpos[3:7].copy()  # w, x, y, z

    def base_height(self) -> float:
        return float(self.data.qpos[2])

    def base_vz(self) -> float:
        """World-frame vertical velocity of the base (m/s)."""
        return float(self.data.qvel[2])

    def base_lowest_z(self) -> float:
        """World z of the lowest chassis-box corner. Jump height measured from here (not the base
        center) stays honest under body tilt."""
        pos = self.data.geom_xpos[self.chassis_geom]
        matz = self.data.geom_xmat[self.chassis_geom].reshape(3, 3)[2]  # world z of the box axes
        return float(pos[2] + (self._box_corners @ matz).min())

    def feet_in_contact(self) -> np.ndarray:
        """Boolean per-foot mask of feet touching the ground (jump env uses it to detect flight)."""
        d = self.data
        mask = np.zeros(6, dtype=bool)
        for c in range(d.ncon):
            g1, g2 = d.contact[c].geom1, d.contact[c].geom2
            for i, fg in enumerate(self.foot_geom_ids):
                if (g1 == fg and g2 == self.ground_geom_id) or (g2 == fg and g1 == self.ground_geom_id):
                    mask[i] = True
        return mask

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
        """Mechanical power |torque * joint_velocity| summed over actuators (W); the cost-of-transport
        signal for trading step length against cadence."""
        return float(np.sum(np.abs(self.data.actuator_force * self.data.actuator_velocity)))

    def foot_slip_sq(self) -> float:
        """Sum of squared horizontal foot speed over planted feet; penalizing it discourages the
        sim-only slip exploit and improves transfer."""
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
