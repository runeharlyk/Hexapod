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
    GaitType.TRI_GATE: [0, 0.5, 0, 0.5, 0, 0.5],
    GaitType.BI_GATE: [0, 1 / 3, 2 / 3, 2 / 3, 1 / 3, 0],
    GaitType.WAVE: [0, 1 / 6 * 1, 1 / 6 * 2, 1 / 6 * 5, 1 / 6 * 4, 1 / 6 * 3],
    GaitType.RIPPLE: [0, 1 / 6 * 1, 1 / 6 * 2, 1 / 6 * 5, 1 / 6 * 4, 1 / 6 * 3],
}

default_stand_frac = {
    GaitType.TRI_GATE: 3.1 / 6,
    GaitType.BI_GATE: 2.1 / 6,
    GaitType.WAVE: 5 / 6,
    GaitType.RIPPLE: 5 / 6,
}


class GaitStateT(TypedDict):
    step_height: float
    step_x: float
    step_z: float
    step_angle: float
    step_velocity: float
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
    def __init__(self, default_pos: np.ndarray):
        self.default_pos = default_pos
        self.num_legs = len(default_pos)
        self.t = 0.0

    def step(self, s: GaitStateT, b: BodyStateT, dt: float):
        L = np.hypot(s["step_x"], s["step_z"])
        self.step_length = -L if s["step_x"] < 0 else L
        self.t = (self.t + dt * s["step_velocity"]) % 1

        new_feet = np.zeros((self.num_legs, 3))

        for i, p in enumerate(self.default_pos):
            ph = (self.t + s["offset"][i]) % 1

            if s["step_x"] or s["step_z"] or s["step_angle"]:
                if ph < s["stand_frac"]:
                    ph0, curve, amp = ph / s["stand_frac"], sine_curve, -s["step_depth"]
                else:
                    ph0, curve, amp = (ph - s["stand_frac"]) / (1 - s["stand_frac"]), bezier_curve, s["step_height"]

                x = curve(self.step_length / 2, np.arctan2(s["step_z"], self.step_length) * 2, amp, ph0)

                r = curve(np.rad2deg(s["step_angle"]), yaw_arc(p, b["feet"][i]), amp, ph0)

                new_feet[i] = p + x + r
            else:
                new_feet[i] = p

        b["feet"] = new_feet
