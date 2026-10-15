"""Domain randomization for sim-to-real transfer.

Applied by `HexapodMjEnv` when `randomize=True`. Per-episode it perturbs the MODEL
(masses/inertia, CoM, friction) and sets per-episode sensor biases, action latency, and a
push schedule. Per-step it adds IMU noise and applies pushes.

Servo strength is deliberately NOT randomized here. The model's actuators are plain torque
motors (build_model.py) driven by the servo model in mj_runtime, so the strength factors are the
env's `set_servo_scale(stall=..., hold=...)`, drawn alongside stiffness and no-load speed: `stall`
scales the driving torque, and `hold` the holding/stall ratio of the non-back-drivable gear train.
Scaling the motor gain here as well would stack a second, unlogged strength factor on top.
Their ranges, and the deadband's, are part of `Ranges` all the same, so that identify.py can re-centre them.

The most important range for THIS robot (WiFi/BLE + open-loop servos) is action latency.

The ranges are assumptions until a recording replaces them: identify.py fits the servo, latency and IMU terms to
one and writes them as a JSON file, which `Ranges.identified` re-centres the ranges on.
"""

import json
import math
from dataclasses import dataclass

import numpy as np
import mujoco

CONTROL_DT = 0.02  # s, the env's nominal control step the action latency counts in
# An identified value narrows its range to 3 sigma around it, but no narrower than this share of today's range: one
# recording of one robot does not cover the next robot or the next battery.
MIN_RANGE_SHARE = 0.25


def _around(value: float, std: float, default: tuple[float, float]) -> tuple[float, float]:
    half = max(3.0 * std, MIN_RANGE_SHARE * (default[1] - default[0]) / 2)
    return max(value - half, 0.0), value + half


@dataclass(frozen=True)
class Ranges:
    """What the randomization draws from. Servo terms are `set_servo_scale` factors, except the deadband (rad)."""

    gyro_bias_mean: tuple[float, float, float] = (0.0, 0.0, 0.0)  # rad/s
    gyro_bias_std: float = 0.05     # rad/s, per episode
    gyro_noise_std: float = 0.05    # rad/s, per step
    rpy_bias_deg: float = 5.0       # per episode, uniform +-
    rpy_noise_deg: float = 1.5      # per step
    gravity_noise_std: float = 0.02  # per step, per component of the unit vector
    kp: tuple[float, float] = (0.65, 1.35)
    stall: tuple[float, float] = (0.80, 1.20)
    noload: tuple[float, float] = (0.85, 1.15)
    hold: tuple[float, float] = (0.80, 1.20)  # holding/stall ratio 2.0-3.0 around the 2.5 placeholder
    deadband: tuple[float, float] = (0.005, 0.026)  # ~0.3-1.5 deg gear lash / PWM deadband
    action_latency_steps: tuple[int, int] = (1, 4)  # rng.integers bounds: 1..3 control steps (~20-60 ms)

    @classmethod
    def identified(cls, path) -> "Ranges":
        """The default ranges, re-centred on the estimates identify.py wrote to `path`; a term it could not identify
        keeps its default. The gyro bias centres on the recorded robot's but keeps the default spread, since one
        recording shows one boot's calibration."""
        found = json.loads(open(path).read())["estimates"]
        value = lambda name: found[name]["value"]
        std = lambda name: found[name]["std"] or 0.0
        default = cls()
        changes = {}
        if all(f"gyro_bias_{axis}" in found for axis in "xyz"):
            changes["gyro_bias_mean"] = tuple(value(f"gyro_bias_{axis}") for axis in "xyz")
        for name in ("gyro_noise_std", "rpy_noise_deg", "gravity_noise_std"):
            if name in found:
                changes[name] = value(name)
        for name in ("kp", "stall", "noload", "hold", "deadband"):
            if name in found:
                changes[name] = _around(value(name), std(name), getattr(default, name))
        if {"lag_ms", "sensor_to_servo_ms_p50", "sensor_to_servo_ms_p99"} <= found.keys():
            lag, sigma = value("lag_ms"), 3.0 * std("lag_ms")
            step_ms = CONTROL_DT * 1000.0
            low_ms = value("sensor_to_servo_ms_p50") + max(lag - sigma, 0.0)
            high_ms = value("sensor_to_servo_ms_p99") + lag + sigma
            changes["action_latency_steps"] = (math.floor(low_ms / step_ms), math.ceil(high_ms / step_ms) + 1)
        return cls(**changes)


class DomainRandomizer:
    def __init__(self, model, ranges: Ranges = Ranges()):
        self.ranges = ranges
        self.base_body_mass = model.body_mass.copy()
        self.base_body_inertia = model.body_inertia.copy()
        self.base_body_ipos = model.body_ipos.copy()
        self.base_geom_friction = model.geom_friction.copy()
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

        # per-episode sensor biases + latency
        ranges = self.ranges
        self.gyro_bias = rng.normal(ranges.gyro_bias_mean, ranges.gyro_bias_std, size=3)
        self.rpy_bias = np.deg2rad(rng.uniform(-ranges.rpy_bias_deg, ranges.rpy_bias_deg, size=3))
        self.action_latency_steps = int(rng.integers(*ranges.action_latency_steps))

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
        ranges = self.ranges
        gyro = gyro + self.gyro_bias + rng.normal(0.0, ranges.gyro_noise_std, 3)
        rpy = rpy + self.rpy_bias + np.deg2rad(rng.normal(0.0, ranges.rpy_noise_deg, 3))
        grav = grav + rng.normal(0.0, ranges.gravity_noise_std, 3)
        return grav, gyro, rpy
