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

    def gen_posture(self, j2_angle, j3_angle):
        j2_rad = j2_angle / 180 * np.pi
        j3_rad = j3_angle / 180 * np.pi
        posture = np.zeros((6, 3))

        posture[:, 0] = self.mount_x + (
            self.root_j1 + self.j1_j2 + (self.j2_j3 * np.sin(j2_rad)) + self.j3_tip * np.cos(j3_rad)
        ) * np.cos(self.mount_angle)
        posture[:, 1] = self.mount_y + (
            self.root_j1 + self.j1_j2 + (self.j2_j3 * np.sin(j2_rad)) + self.j3_tip * np.cos(j3_rad)
        ) * np.sin(self.mount_angle)
        posture[:, 2] = self.j2_j3 * np.cos(j2_rad) - self.j3_tip * np.sin(j3_rad)
        return posture

    def inverse_kinematics(self, body_state):
        mount_position = np.zeros((6, 3))
        mount_position[:, 0] = self.mount_x
        mount_position[:, 1] = self.mount_y

        transformation = get_transformation_matrix(body_state)

        rotated_dest = np.zeros_like(body_state["feet"])
        for i in range(6):
            point = np.append(body_state["feet"][i], 1)
            transformed = transformation @ point
            rotated_dest[i] = transformed[:3]

        temp_dest = rotated_dest - mount_position
        local_dest = np.zeros_like(body_state["feet"])
        local_dest[:, 0] = temp_dest[:, 0] * np.cos(self.mount_angle) + temp_dest[:, 1] * np.sin(self.mount_angle)
        local_dest[:, 1] = temp_dest[:, 0] * np.sin(self.mount_angle) - temp_dest[:, 1] * np.cos(self.mount_angle)
        local_dest[:, 2] = temp_dest[:, 2]

        angles = np.zeros((6, 3))
        x = local_dest[:, 0] - self.root_j1
        y = local_dest[:, 1]

        angles[:, 0] = -(np.arctan2(y, x) * 180 / np.pi)

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

        angles[:, 1] = (ar + a1) * 180 / np.pi
        angles[:, 2] = (90 - ((a1 + a2) * 180 / np.pi)) - 90

        return angles
