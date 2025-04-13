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

def world_to_leg_local(foot_pos, base_pose, mount_pos, mount_angle_deg):
    roll, pitch, yaw, xm, ym, zm = base_pose
    base_rot = rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)
    T_base = np.eye(4)
    T_base[:3, :3] = base_rot
    T_base[:3, 3] = [xm, ym, zm]

    mount_rot = rot_z(np.radians(mount_angle_deg))
    T_mount = np.eye(4)
    T_mount[:3, :3] = mount_rot
    T_mount[:3, 3] = mount_pos

    T_world_to_leg = np.linalg.inv(T_base @ T_mount)
    foot_pos_h = np.append(foot_pos, 1)
    return (T_world_to_leg @ foot_pos_h)[:3]

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
    # mount = np.array(config.mountPosition)
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

def whole_body_kinematics(state: BodyStateT):
    joint_angles = np.zeros(18)
    for i in range(6):
        local_pos = world_to_leg_local(
            state['feet'][i],
            [state['omega'], state['phi'], state['psi'],state['xm'], state['ym'], state['zm']],
            config.mountPosition[i],
            config.defaultAngle[i]
        )
        angles = ik(local_pos)  # use your per-leg IK
        joint_angles[i*3:i*3+3] = angles
    return joint_angles

def safe_acos(x):
    return math.acos(min(1.0, max(-1.0, x)))

def ik(to):
    angles = []
    x = to[0] - config.kLegRootToJoint1
    y = to[1]

    angles.append(math.atan2(y, x) * 180 / math.pi)

    x = math.sqrt(x*x + y*y) - config.kLegJoint1ToJoint2
    y = to[2]
    lr2 = x*x + y*y
    lr = math.sqrt(lr2)

    L2 = config.kLegJoint2ToJoint3
    L3 = config.kLegJoint3ToTip

    if lr < 1e-6:
        return None

    a1 = math.pi - safe_acos((lr2 + L2*L2 - L3*L3) / (2 * L2 * lr))
    a2 = safe_acos((lr2 - L2*L2 + L3*L3) / (2 * L3 * lr))
    ar = math.atan2(y, x)

    angles.append((ar + a1) * 180 / math.pi)
    angles.append(90 - ((a1 + a2) * 180 / math.pi))

    return angles


def inverse_kinematics(dest, config):
    """Calculates the joint angles for each leg of the hexapod to reach a desired destination.

    Args:
        dest (numpy.ndarray): A 3D array representing the desired destination coordinates
            for each leg. The shape of the array is (6, 3), where:
                - 6 is the number of legs.
                - 3 is the number of coordinates (x, y, z).
        config (dict): A dictionary containing the hexapod's configuration parameters.
            The dictionary should contain the following keys:
                - "legMountX": A list of x-coordinates for the leg mounts.
                - "legMountY": A list of y-coordinates for the leg mounts.
                - "legRootToJoint1": The distance from the leg root to joint 1.
                - "legJoint1ToJoint2": The distance from joint 1 to joint 2.
                - "legJoint2ToJoint3": The distance from joint 2 to joint 3.
                - "legJoint3ToTip": The distance from joint 3 to the leg tip.
                - "legMountAngle": A list of angles for the leg mounts in degrees.
                - "legScale": A list of scaling factors for each leg's joint angles.

    Returns:
        numpy.ndarray: A 3D array representing the joint angles for each leg.
        The shape of the array is (6, 3), where:
            - 6 is the number of legs.
            - 3 is the number of joint angles (j1, j2, j3).
    """
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

    temp_dest = dest - mount_position
    local_dest = np.zeros_like(dest)
    local_dest[:, 0] = temp_dest[:, 0] * np.cos(mount_angle) + temp_dest[:, 1] * np.sin(
        mount_angle
    )
    local_dest[:, 1] = temp_dest[:, 0] * np.sin(mount_angle) - temp_dest[:, 1] * np.cos(
        mount_angle
    )
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
    a1 = np.arccos((lr2 + j2_j3 * j2_j3 - j3_tip * j3_tip) / (2 * j2_j3 * lr))
    a2 = np.arccos((lr2 - j2_j3 * j2_j3 + j3_tip * j3_tip) / (2 * j3_tip * lr))

    angles[:, 1] = ((ar + a1) * 180 / np.pi) * leg_scale[:, 1]
    angles[:, 2] = (90 - ((a1 + a2) * 180 / np.pi)) * leg_scale[:, 1] - 90

    return angles