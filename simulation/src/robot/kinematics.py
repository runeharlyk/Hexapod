import math
import numpy as np
from typing import TypedDict, List

import config

class BodyStateT(TypedDict):
  omega: float
  phi: float
  psi: float
  xm: float
  ym: float
  zm: float
  feet: List[List[float]]

def rot_x(theta): return np.array([[1, 0, 0],
                                   [0, np.cos(theta), -np.sin(theta)],
                                   [0, np.sin(theta), np.cos(theta)]])
def rot_y(theta): return np.array([[np.cos(theta), 0, np.sin(theta)],
                                   [0, 1, 0],
                                   [-np.sin(theta), 0, np.cos(theta)]])
def rot_z(theta): return np.array([[np.cos(theta), -np.sin(theta), 0],
                                   [np.sin(theta),  np.cos(theta), 0],
                                   [0, 0, 1]])

def compute_default_position():
    foot_positions = np.zeros((6, 3))
    l1 = config.kLegRootToJoint1
    l2 = config.kLegJoint1ToJoint2
    l3 = config.kLegJoint2ToJoint3
    l4 = config.kLegJoint3ToTip

    forward = l2 + l3 * config.SIN30 + l4 * config.SIN15
    down = l3 * config.COS30 + l4 * config.COS15

    spread_scale = 1
    local_tip = np.array([(l1 + forward) * spread_scale, 0, -down])

    for i in range(6):
        angle = np.radians(config.defaultAngle[i])
        rot = np.array([
            [np.cos(angle), -np.sin(angle), 0],
            [np.sin(angle),  np.cos(angle), 0],
            [0, 0, 1]
        ])
        rotated = rot @ local_tip
        pos = np.array(config.mountPosition[i]) + rotated
        foot_positions[i] = pos
    return foot_positions

def gen_posture(j2_angle, j3_angle, config=config):
    mount_x = config["legMountX"]
    mount_y = config["legMountY"]
    root_j1 = config["legRootToJoint1"]
    j1_j2 = config["legJoint1ToJoint2"]
    j2_j3 = config["legJoint2ToJoint3"]
    j3_tip = config["legJoint3ToTip"]
    mount_angle = np.array(config["legMountAngle"]) / 180 * np.pi

    j2_rad = j2_angle / 180 * np.pi
    j3_rad = j3_angle / 180 * np.pi
    posture = np.zeros((6, 3))

    posture[:, 0] = mount_x + (root_j1 + j1_j2 + (j2_j3 * np.sin(j2_rad)) + j3_tip * np.cos(j3_rad)) * np.cos(mount_angle)
    posture[:, 1] = mount_y + (root_j1 + j1_j2 + (j2_j3 * np.sin(j2_rad)) + j3_tip * np.cos(j3_rad)) * np.sin(mount_angle)
    posture[:, 2] = j2_j3 * np.cos(j2_rad) - j3_tip * np.sin(j3_rad)
    return posture

def inverse_kinematics(dest, body_state, config):
    mount_x = np.array(config["legMountX"])
    mount_y = np.array(config["legMountY"])
    root_j1 = config["legRootToJoint1"]
    j1_j2 = config["legJoint1ToJoint2"]
    j2_j3 = config["legJoint2ToJoint3"]
    j3_tip = config["legJoint3ToTip"]
    mount_angle = np.array(config["legMountAngle"]) / 180 * np.pi
    mount_position = np.zeros((6, 3))
    mount_position[:, 0] = mount_x
    mount_position[:, 1] = mount_y
    leg_scale = np.array(config["legScale"])

    roll = body_state["omega"]
    pitch = body_state["phi"]
    yaw = body_state["psi"]
    
    rot_matrix = rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)
    
    rotated_dest = np.zeros_like(dest)
    for i in range(6):
        rotated_dest[i] = rot_matrix @ dest[i]
    
    temp_dest = rotated_dest - mount_position
    local_dest = np.zeros_like(dest)
    local_dest[:, 0] = temp_dest[:, 0] * np.cos(mount_angle) + temp_dest[:, 1] * np.sin(mount_angle)
    local_dest[:, 1] = temp_dest[:, 0] * np.sin(mount_angle) - temp_dest[:, 1] * np.cos(mount_angle)
    local_dest[:, 2] = temp_dest[:, 2]

    angles = np.zeros((6, 3))
    x = local_dest[:, 0] - root_j1
    y = local_dest[:, 1]

    angles[:, 0] = -(np.arctan2(y, x) * 180 / np.pi)

    x = np.sqrt(x * x + y * y) - j1_j2
    y = local_dest[:, 2]
    ar = np.arctan2(y, x)
    lr2 = x * x + y * y
    lr = np.sqrt(lr2)
    
    valid_legs = lr > 1e-6
    a1 = np.zeros_like(lr)
    a2 = np.zeros_like(lr)
    
    if np.any(valid_legs):
        cos_a1 = (lr2[valid_legs] + j2_j3*j2_j3 - j3_tip*j3_tip) / (2 * j2_j3 * lr[valid_legs])
        cos_a2 = (lr2[valid_legs] - j2_j3*j2_j3 + j3_tip*j3_tip) / (2 * j3_tip * lr[valid_legs])
        
        cos_a1 = np.clip(cos_a1, -1.0, 1.0)
        cos_a2 = np.clip(cos_a2, -1.0, 1.0)
        
        a1[valid_legs] = np.arccos(cos_a1)
        a2[valid_legs] = np.arccos(cos_a2)

    angles[:, 1] = ((ar + a1) * 180 / np.pi) * leg_scale[:, 1]
    angles[:, 2] = (90 - ((a1 + a2) * 180 / np.pi)) * leg_scale[:, 1] - 90

    return angles