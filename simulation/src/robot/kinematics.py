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
    default_feet: List[List[float]]
    px: float
    py: float
    pz: float


def rot_x(theta):
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(theta):
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(theta):
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def get_transformation_matrix(body_state):
    roll, pitch, yaw = body_state["omega"], body_state["phi"], body_state["psi"]
    xm, ym, zm = body_state["xm"], body_state["ym"], body_state["zm"]
    px, py, pz = body_state["px"], body_state["py"], body_state["pz"]

    pivot = np.array([px, py, pz])
    translation = np.array([[1, 0, 0, xm], [0, 1, 0, ym], [0, 0, 1, zm], [0, 0, 0, 1]])

    rotation = np.eye(4)
    rotation[:3, :3] = rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)

    pivot_to_origin = np.eye(4)
    pivot_to_origin[:3, 3] = -pivot
    origin_to_pivot = np.eye(4)
    origin_to_pivot[:3, 3] = pivot

    return translation @ pivot_to_origin @ rotation @ origin_to_pivot


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
        rot = np.array([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
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

    posture[:, 0] = mount_x + (root_j1 + j1_j2 + (j2_j3 * np.sin(j2_rad)) + j3_tip * np.cos(j3_rad)) * np.cos(
        mount_angle
    )
    posture[:, 1] = mount_y + (root_j1 + j1_j2 + (j2_j3 * np.sin(j2_rad)) + j3_tip * np.cos(j3_rad)) * np.sin(
        mount_angle
    )
    posture[:, 2] = j2_j3 * np.cos(j2_rad) - j3_tip * np.sin(j3_rad)
    return posture


def inverse_kinematics(body_state, config):
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

    transformation = get_transformation_matrix(body_state)

    rotated_dest = np.zeros_like(body_state["feet"])
    for i in range(6):
        point = np.append(body_state["feet"][i], 1)
        transformed = transformation @ point
        rotated_dest[i] = transformed[:3]

    temp_dest = rotated_dest - mount_position
    local_dest = np.zeros_like(body_state["feet"])
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
        cos_a1 = (lr2[valid_legs] + j2_j3 * j2_j3 - j3_tip * j3_tip) / (2 * j2_j3 * lr[valid_legs])
        cos_a2 = (lr2[valid_legs] - j2_j3 * j2_j3 + j3_tip * j3_tip) / (2 * j3_tip * lr[valid_legs])

        cos_a1 = np.clip(cos_a1, -1.0, 1.0)
        cos_a2 = np.clip(cos_a2, -1.0, 1.0)

        a1[valid_legs] = np.arccos(cos_a1)
        a2[valid_legs] = np.arccos(cos_a2)

    angles[:, 1] = (ar + a1) * 180 / np.pi
    angles[:, 2] = (90 - ((a1 + a2) * 180 / np.pi)) - 90

    return angles
