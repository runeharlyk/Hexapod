import math
import numpy as np
from typing import TypedDict

class GaitStateT(TypedDict):
    step_height: float
    step_x: float
    step_z: float
    step_angle: float
    step_velocity: float
    step_depth: float

def stance_curve(length, angle, depth, phase):
    x_polar, z_polar = np.cos(angle), np.sin(angle)
    step = length * (1 - 2 * phase)
    x, z = step * x_polar, step * z_polar
    y = -depth * np.cos(np.pi * (x + z) / (2 * length)) if length != 0 else 0
    return np.array([x, z, y])


def yaw_arc(default_pos, current_pos):
    foot_mag = np.linalg.norm(default_pos[[0, 2]])
    foot_dir = np.arctan2(default_pos[2], default_pos[0])
    offset = current_pos - default_pos
    offset_mag = np.linalg.norm(offset[[0, 2]])
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

def gait_controller(length, angle, height, stand_frac, t):
    if t < stand_frac:
        return stance_curve(length, angle, 0, t / stand_frac)
    else:
        return bezier_curve(length, angle, height, (t - stand_frac) / (1 - stand_frac))

