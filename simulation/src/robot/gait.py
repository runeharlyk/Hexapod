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
    GaitType.BI_GATE: [0, 1/3, 2/3, 2/3, 1/3, 0],
    GaitType.WAVE: [0, 1/6*1, 1/6*2, 1/6*5, 1/6*4, 1/6*3],
    GaitType.RIPPLE: [0, 1/6*1, 1/6*2, 1/6*5, 1/6*4, 1/6*3]
}

default_stand_frac = {
    GaitType.TRI_GATE: 3.1 / 6,
    GaitType.BI_GATE: 2.1 / 6,
    GaitType.WAVE: 5 / 6,
    GaitType.RIPPLE: 5 / 6
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

def sine_curve(length, angle, height, phase):
    x_polar, z_polar = np.cos(angle), np.sin(angle)
    step = length * (1 - 2 * phase)
    x, z = step * x_polar, step * z_polar
    y = height * np.cos(np.pi * (x + z) / (2 * length)) if length != 0 else 0
    return np.array([x, z, y])


def yaw_arc(feet_pos, current_pos):
    foot_mag = np.hypot(feet_pos[0], feet_pos[1])
    foot_dir = np.arctan2(feet_pos[1], feet_pos[0])
    offsets = current_pos - feet_pos
    offset_mag = np.hypot(offsets[0], offsets[1])
    offset_mod = np.arctan2(offset_mag, foot_mag)
    return np.pi / 2 + foot_dir + offset_mod


def get_control_points(length, angle, height):
    x_polar, z_polar = np.cos(angle), np.sin(angle)
    step = np.array([-length, -length*1.4, -length*1.5, -length*1.5, -length*1.5,
                     0.0, 0.0, 0.0,
                     length*1.5, length*1.5, length*1.4, length])
    y_vals = np.array([0.0, 0.0, height*0.9, height*0.9, height*0.9,
                       height*0.9, height*0.9, height*1.1, height*1.1,
                       height*1.1, 0.0, 0.0])
    x = step * x_polar
    z = step * z_polar
    return np.stack([x, z, y_vals], axis=1)


def bezier_curve(length, angle, height, phase):
    ctrl = get_control_points(length, angle, height)
    n = len(ctrl) - 1
    coeffs = np.array([math.comb(n, i) * (phase**i) * ((1 - phase)**(n - i)) for i in range(n + 1)])
    return np.sum(ctrl * coeffs[:, None], axis=0)


class GaitController:
    def __init__(self, default_pos: np.ndarray):
        self.t = 0.0
        self.default_pos = default_pos
        self.num_legs = len(default_pos)

    def step(self, gait_state: GaitStateT, body_state: BodyStateT, dt: float):
        self.gait_state = gait_state
        self.body_state = body_state

        self.step_length = np.hypot(gait_state["step_x"], gait_state["step_z"])
        if gait_state["step_x"] < 0: self.step_length = -self.step_length

        self._update_phase(dt)
        self._update_feet()

    def _update_phase(self, dt):
        self.t += dt * self.gait_state["step_velocity"]
        self.t %= 1.0
    
    def _update_feet(self):
        new_feet = np.array([self._update_foot(i) for i in range(self.num_legs)])
        self.body_state["feet"] = new_feet

    def _update_foot(self, i):
        res = self.default_pos[i]
        if self.gait_state["step_x"] != 0 or self.gait_state["step_z"] != 0 or self.gait_state["step_angle"] != 0:
            phase = (self.t + self.gait_state["offset"][i]) % 1
            if phase < self.gait_state["stand_frac"]:
                local_phase = phase / self.gait_state["stand_frac"]
                res = self.default_pos[i] + self._stance_controller(i, self.gait_state, local_phase, self.gait_state["step_depth"])
            else:
                local_phase = (phase - self.gait_state["stand_frac"]) / (1 - self.gait_state["stand_frac"])
                res = self.default_pos[i] + self._swing_controller(i, self.gait_state, local_phase, self.gait_state["step_height"])
        return res

    def _stance_controller(self, i, gait_state: GaitStateT, phase, depth):
        return self._controller(i, gait_state, sine_curve, phase, -depth)

    def _swing_controller(self, i, gait_state: GaitStateT, phase, height):
        return self._controller(i, gait_state, bezier_curve, phase, height)

    def _controller(self, i, gait_state: GaitStateT, curve, phase, *args):
        angle = np.arctan2(gait_state["step_z"], self.step_length) * 2
        length = self.step_length / 2
        pos = curve(length, angle, *args, phase)

        length = np.rad2deg(gait_state["step_angle"])
        angle = yaw_arc(self.default_pos[i], self.body_state["feet"][i])

        rot = curve(length, angle, *args, phase)
        return pos + rot
