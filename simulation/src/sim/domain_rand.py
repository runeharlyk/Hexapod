"""Domain randomization for sim-to-real transfer.

Applied by `HexapodMjEnv` when `randomize=True`. Per-episode it perturbs the MODEL
(masses/inertia, CoM, friction, motor strength) and sets per-episode sensor biases,
action latency, and a push schedule. Per-step it adds IMU noise and applies pushes.

The most important range for THIS robot (WiFi/BLE + open-loop servos) is action latency.
"""

import numpy as np
import mujoco


class DomainRandomizer:
    def __init__(self, model):
        self.base_body_mass = model.body_mass.copy()
        self.base_body_inertia = model.body_inertia.copy()
        self.base_body_ipos = model.body_ipos.copy()
        self.base_geom_friction = model.geom_friction.copy()
        self.base_gainprm = model.actuator_gainprm.copy()
        self.base_biasprm = model.actuator_biasprm.copy()
        self.base_forcerange = model.actuator_forcerange.copy()
        self.base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")

        # per-episode state
        self.gyro_bias = np.zeros(3)
        self.rpy_bias = np.zeros(3)
        self.action_latency_steps = 0
        self.next_push = 10**9
        self.push_steps_left = 0
        self.push_force = np.zeros(3)

    # -------------------------------------------------- per-episode (at reset)
    def reset_episode(self, model, rng):
        scale = rng.uniform(0.8, 1.2, size=model.nbody)
        model.body_mass[:] = self.base_body_mass * scale
        model.body_inertia[:] = self.base_body_inertia * scale[:, None]
        # base CoM offset (battery placement uncertainty): +/-15mm xy, +/-10mm z
        model.body_ipos[self.base_id] = self.base_body_ipos[self.base_id] + rng.uniform(
            [-0.015, -0.015, -0.010], [0.015, 0.015, 0.010]
        )
        model.geom_friction[:, 0] = self.base_geom_friction[:, 0] * rng.uniform(0.6, 1.4)
        # motor strength: position actuator kp lives in gainprm[:,0] and biasprm[:,1] = -kp
        kp = rng.uniform(0.8, 1.2)
        model.actuator_gainprm[:, 0] = self.base_gainprm[:, 0] * kp
        model.actuator_biasprm[:, 1] = self.base_biasprm[:, 1] * kp
        model.actuator_forcerange[:] = self.base_forcerange * rng.uniform(0.85, 1.15)

        # per-episode sensor biases + latency
        self.gyro_bias = rng.normal(0.0, 0.05, size=3)
        self.rpy_bias = np.deg2rad(rng.uniform(-5.0, 5.0, size=3))
        self.action_latency_steps = int(rng.integers(1, 4))  # 1..3 control steps (~20-60ms hobby-servo lag)

        # push schedule (every ~3-5 s at 50 Hz)
        self.next_push = int(rng.integers(150, 250))
        self.push_steps_left = 0
        self.push_force[:] = 0.0

    # -------------------------------------------------- per-step
    def maybe_push(self, model, data, rng, step):
        if step >= self.next_push and self.push_steps_left == 0:
            self.push_force = rng.uniform(-1, 1, size=3) * np.array([3.0, 3.0, 1.0])  # N
            self.push_steps_left = 5
            self.next_push = step + int(rng.integers(150, 250))
        if self.push_steps_left > 0:
            data.xfrc_applied[self.base_id, :3] = self.push_force
            self.push_steps_left -= 1
        else:
            data.xfrc_applied[self.base_id, :3] = 0.0

    def noisy_imu(self, grav, gyro, rpy, rng):
        gyro = gyro + self.gyro_bias + rng.normal(0.0, 0.05, 3)
        rpy = rpy + self.rpy_bias + np.deg2rad(rng.normal(0.0, 1.5, 3))
        grav = grav + rng.normal(0.0, 0.02, 3)
        return grav, gyro, rpy
