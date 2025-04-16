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

def stance_curve(length, angle, depth, phase):
    x_polar, z_polar = np.cos(angle), np.sin(angle)
    step = length * (1 - 2 * phase)
    x, z = step * x_polar, step * z_polar
    y = -depth * np.cos(np.pi * (x + z) / (2 * length)) if length != 0 else 0
    return np.array([x, z, y])


def yaw_arc(feet_pos, current_pos):
    feet_pos = np.asarray(feet_pos)
    current_pos = np.asarray(current_pos)
    foot_mag = np.hypot(feet_pos[0], feet_pos[2])
    foot_dir = np.arctan2(feet_pos[2], feet_pos[0])
    offsets = current_pos - feet_pos
    offset_mag = np.hypot(offsets[0], offsets[2])
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


t = 0

def update_gait(gait_state: GaitStateT, body_state:BodyStateT, default_pos: np.ndarray, dt: float):
    global t
    t += dt * gait_state["step_velocity"]
    t %= 1.0
    pose = np.zeros((6, 3))

    for i in range(6):
        pose[i] = default_pos[i]
        if gait_state["step_x"] != 0 or gait_state["step_z"] != 0:
            phase = (t + gait_state["offset"][i]) % 1
            pose[i] = gait_controller(gait_state, default_pos[i], phase)
    body_state["feet"] = pose


def gait_controller(gait_state: GaitStateT, default_pos: np.ndarray, t):
    step_length = np.hypot(gait_state["step_x"], gait_state["step_z"])
    if gait_state["step_x"] < 0: step_length = -step_length

    angle = np.arctan2(gait_state["step_z"], step_length) * 2
    
    length = step_length / 2

    if t < gait_state["stand_frac"]:
        delta_pos = stance_curve(length, angle, gait_state["step_depth"], t / gait_state["stand_frac"])
        length = gait_state["step_angle"] * 2
        angle = yaw_arc(default_pos, default_pos + delta_pos) # Use body_state current feet position
        delta_rot = stance_curve(length, angle, gait_state["step_depth"], t / gait_state["stand_frac"])
    else:
        delta_pos = bezier_curve(length, angle, gait_state["step_height"], (t - gait_state["stand_frac"]) / (1 - gait_state["stand_frac"]))
        length = gait_state["step_angle"] * 2
        angle = yaw_arc(default_pos, default_pos + delta_pos)
        delta_rot = bezier_curve(length, angle, gait_state["step_height"], t / gait_state["stand_frac"])
    return default_pos + delta_pos #+ delta_rot * 0.02

