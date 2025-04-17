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
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])


def rot_y(theta):
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]])


def rot_z(theta):
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])


def rot(omega, phi, psi):
    return rot_z(psi) @ rot_y(phi) @ rot_x(omega)


def translation(x, y, z):
    return np.array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])


def transformation(omega, phi, psi, x, y, z):
    return rot(omega, phi, psi) @ translation(x, y, z)


def get_transformation_matrix(body_state):
    omega, phi, psi = body_state["omega"], body_state["phi"], body_state["psi"]
    xm, ym, zm = body_state["xm"], body_state["ym"], body_state["zm"]

    return transformation(omega, phi, psi, xm, ym, zm)


class Kinematics:
    def __init__(self, config):
        self.mount_x = np.array(config["legMountX"])
        self.mount_y = np.array(config["legMountY"])
        self.root_j1 = config["legRootToJoint1"]
        self.j1_j2 = config["legJoint1ToJoint2"]
        self.j2_j3 = config["legJoint2ToJoint3"]
        self.j3_tip = config["legJoint3ToTip"]
        self.mount_angle = np.array(config["legMountAngle"]) / 180 * np.pi
        self.mount_position = np.zeros((6, 3))
        self.mount_position[:, 0] = self.mount_x
        self.mount_position[:, 1] = self.mount_y

    def gen_posture(self, j2_angle, j3_angle):
        posture = np.zeros((6, 3))

        posture[:, 0] = self.mount_x + (
            self.root_j1 + self.j1_j2 + (self.j2_j3 * np.sin(j2_angle)) + self.j3_tip * np.cos(j3_angle)
        ) * np.cos(self.mount_angle)
        posture[:, 1] = self.mount_y + (
            self.root_j1 + self.j1_j2 + (self.j2_j3 * np.sin(j2_angle)) + self.j3_tip * np.cos(j3_angle)
        ) * np.sin(self.mount_angle)
        posture[:, 2] = self.j2_j3 * np.cos(j2_angle) - self.j3_tip * np.sin(j3_angle)
        return posture

    def inverse_kinematics(self, body_state):
        T = get_transformation_matrix(body_state)

        feet = body_state["feet"]
        pts = np.hstack([feet, np.ones((6, 1))])
        world = (T @ pts.T).T[:, :3] - self.mount_position

        ca, sa = np.cos(self.mount_angle), np.sin(self.mount_angle)

        local_dest = np.zeros_like(body_state["feet"])
        local_dest[:, 0] = world[:, 0] * ca + world[:, 1] * sa
        local_dest[:, 1] = world[:, 0] * sa - world[:, 1] * ca
        local_dest[:, 2] = world[:, 2]

        angles = np.zeros((6, 3))
        x = local_dest[:, 0] - self.root_j1
        y = local_dest[:, 1]

        angles[:, 0] = -np.arctan2(y, x)

        x = np.sqrt(x * x + y * y) - self.j1_j2
        y = local_dest[:, 2]
        ar = np.arctan2(y, x)
        lr2 = x * x + y * y
        lr = np.sqrt(lr2)

        valid_legs = lr > 1e-6
        a1 = np.zeros_like(lr)
        a2 = np.zeros_like(lr)

        if np.any(valid_legs):
            cos_a1 = (lr2[valid_legs] + self.j2_j3 * self.j2_j3 - self.j3_tip * self.j3_tip) / (
                2 * self.j2_j3 * lr[valid_legs]
            )
            cos_a2 = (lr2[valid_legs] - self.j2_j3 * self.j2_j3 + self.j3_tip * self.j3_tip) / (
                2 * self.j3_tip * lr[valid_legs]
            )

            cos_a1 = np.clip(cos_a1, -1.0, 1.0)
            cos_a2 = np.clip(cos_a2, -1.0, 1.0)

            a1[valid_legs] = np.arccos(cos_a1)
            a2[valid_legs] = np.arccos(cos_a2)

        angles[:, 1] = ar + a1
        angles[:, 2] = -(a1 + a2)

        return angles
