import math
import numpy as np
from typing import TypedDict
from enum import Enum

from src.robot.kinematics import BodyStateT


class GaitType(Enum):
    TRI_GATE = 0
    BI_GATE = 1
    WAVE = 2
    RIPPLE = 3


default_offset = {
    # GaitType.TRI_GATE: [0.00, 0.52, 0.08, 0.58, 0.16, 0.66],
    GaitType.TRI_GATE: [0.00, 0.5, 0.0, 0.5, 0.0, 0.5],
    GaitType.BI_GATE: [0, 1 / 3, 2 / 3, 2 / 3, 1 / 3, 0],
    GaitType.WAVE: [0, 1 / 6, 2 / 6, 5 / 6, 4 / 6, 3 / 6],
    GaitType.RIPPLE: [0, 4 / 6, 2 / 6, 1 / 6, 5 / 6, 3 / 6],
}

default_stand_frac = {
    GaitType.TRI_GATE: 3.1 / 6,
    GaitType.BI_GATE: 2.1 / 6,
    GaitType.WAVE: 5 / 6,
    GaitType.RIPPLE: 5 / 6,
}

gait_speed_ranges = {
    GaitType.TRI_GATE: (0.9, 2.0),
    GaitType.BI_GATE: (1.3, 2.3),
    GaitType.WAVE: (0.4, 1.2),
    GaitType.RIPPLE: (0.4, 1.2),
}

gait_duty_cycles = {
    GaitType.TRI_GATE: 0.5,
    GaitType.BI_GATE: 0.67,
    GaitType.WAVE: 0.83,
    GaitType.RIPPLE: 0.83,
}

MAX_STEP_LENGTH = 70.0


class GaitStateT(TypedDict):
    step_height: float
    velocity_mm_s: float
    direction_rad: float
    turn_rate_rad_s: float
    step_depth: float
    stand_frac: float
    offset: list[float]
    gait_type: GaitType


length_multipliers = np.array([-1.4, -1.0, -1.5, -1.5, -1.5, 0.0, 0.0, 0.0, 1.5, 1.5, 1.4, 1.0])
height_profile = np.array([0.0, 0.0, 0.9, 0.9, 0.9, 0.9, 0.9, 1.1, 1.1, 1.1, 0.0, 0.0])


def sine_curve(length, angle, height, phase):
    x, z = length * (1 - 2 * phase) * np.cos(angle), length * (1 - 2 * phase) * np.sin(angle)
    y = height * np.cos(np.pi * (x + z) / (2 * length)) if length else 0
    return np.array([x, z, y])


def yaw_arc(feet, current):
    return (
        np.pi / 2
        + np.arctan2(feet[1], feet[0])
        + np.arctan2(np.linalg.norm(current[:2] - feet[:2]), np.linalg.norm(feet[:2]))
    )


def get_control_points(length, angle, height):
    x_polar, z_polar = np.cos(angle), np.sin(angle)

    x = length * length_multipliers * x_polar
    z = length * length_multipliers * z_polar
    y = height * height_profile
    return np.stack([x, z, y], axis=1)


def bezier_curve(length, angle, height, phase):
    ctrl = get_control_points(length, angle, height)
    n = len(ctrl) - 1
    coeffs = np.array([math.comb(n, i) * (phase**i) * ((1 - phase) ** (n - i)) for i in range(n + 1)])
    return np.sum(ctrl * coeffs[:, None], axis=0)


class GaitController:
    def __init__(self, default_position: np.ndarray):
        self.default_position = default_position
        self.phase = 0.0

    def _calculate_optimal_params(self, velocity_mm_s: float, gait_type: GaitType, dt: float):
        min_speed, max_speed = gait_speed_ranges[gait_type]
        duty_cycle = gait_duty_cycles[gait_type]

        effective_velocity = velocity_mm_s / duty_cycle

        desired_step_length = abs(effective_velocity)
        step_length = np.clip(desired_step_length, 0, MAX_STEP_LENGTH)

        if step_length > 0:
            required_speed = abs(effective_velocity) / step_length
        else:
            required_speed = min_speed

        speed = np.clip(required_speed, min_speed, max_speed)

        actual_step_length = abs(effective_velocity) / speed if speed > 0 else 0
        actual_step_length = np.clip(actual_step_length, 0, MAX_STEP_LENGTH)

        return actual_step_length, speed

    def step(self, gait: GaitStateT, body: BodyStateT, dt: float):
        velocity_mm_s = gait["velocity_mm_s"]
        direction_rad = gait["direction_rad"] + np.pi / 2
        turn_rate_rad_s = gait["turn_rate_rad_s"]
        gait_type = gait["gait_type"]

        if not any((velocity_mm_s, turn_rate_rad_s)):
            body["feet"] = body["feet"] + (self.default_position - body["feet"]) * dt * 10
            self.phase = 0.0
            return

        step_length, speed = self._calculate_optimal_params(velocity_mm_s, gait_type, dt)

        step_length = step_length * (-1 if abs(direction_rad) < np.pi / 2 else 1)
        turn_amplitude = direction_rad

        turn_speed_factor = np.abs(turn_rate_rad_s)
        speed = np.clip(speed + turn_speed_factor, 0.75, 2.5)

        self._advance_phase(dt, speed)
        stand_fraction = gait["stand_frac"]
        depth = gait["step_depth"]
        height = gait["step_height"]
        offsets = gait["offset"]

        new_feet = np.zeros_like(self.default_position)

        for i, (default_foot, current_foot) in enumerate(zip(self.default_position, body["feet"])):
            phase = (self.phase + offsets[i]) % 1
            ph_norm, curve_fn, amp = self._phase_params(phase, stand_fraction, depth, height)
            delta_pos = curve_fn(step_length / 2, turn_amplitude, amp, ph_norm)
            delta_rot = curve_fn(np.rad2deg(turn_rate_rad_s), yaw_arc(default_foot, current_foot), amp, ph_norm)
            new_feet[i][:3] = default_foot[:3] + delta_pos + delta_rot
            new_feet[i][3] = 1

        body["feet"] = new_feet

    def _advance_phase(self, dt: float, velocity: float):
        self.phase = (self.phase + dt * velocity) % 1

    def _phase_params(self, phase: float, stand_frac: float, depth: float, height: float):
        if phase < stand_frac:
            return phase / stand_frac, sine_curve, -depth
        return (phase - stand_frac) / (1 - stand_frac), bezier_curve, height
