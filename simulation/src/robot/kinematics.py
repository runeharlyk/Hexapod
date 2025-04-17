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
        self.mount_angles = np.deg2rad(config["legMountAngle"])
        self.mount_position = np.column_stack([self.mount_x, self.mount_y, np.zeros_like(self.mount_x)])
        self.ca, self.sa = np.cos(self.mount_angles), np.sin(self.mount_angles)

    def gen_posture(self, j2, j3):
        ext = self.root_j1 + self.j1_j2 + self.j2_j3 * np.sin(j2) + self.j3_tip * np.cos(j3)
        x = self.mount_x + ext * self.ca
        y = self.mount_y + ext * self.sa
        z = self.j2_j3 * np.cos(j2) - self.j3_tip * np.sin(j3) * np.ones_like(x)
        return np.column_stack([x, y, z])

    def inverse_kinematics(self, body_state):
        T = get_transformation_matrix(body_state)

        feet = body_state["feet"]
        pts = np.hstack([feet, np.ones((6, 1))])
        world = (T @ pts.T).T[:, :3] - self.mount_position

        lx = world[:, 0] * self.ca + world[:, 1] * self.sa
        ly = world[:, 0] * self.sa - world[:, 1] * self.ca
        lz = world[:, 2]

        dx = lx - self.root_j1
        dy = ly
        angles = np.zeros((6, 3))
        angles[:, 0] = -np.arctan2(dy, dx)

        radial = np.hypot(dx, dy) - self.j1_j2
        vertical = lz
        base = np.arctan2(vertical, radial)
        lr2 = radial * radial + vertical * vertical
        lr = np.sqrt(lr2)

        cos_a1 = (lr2 + self.j2_j3 * self.j2_j3 - self.j3_tip * self.j3_tip) / (2 * self.j2_j3 * lr)
        cos_a2 = (lr2 - self.j2_j3 * self.j2_j3 + self.j3_tip * self.j3_tip) / (2 * self.j3_tip * lr)

        cos_a1 = np.clip(cos_a1, -1.0, 1.0)
        cos_a2 = np.clip(cos_a2, -1.0, 1.0)

        a1 = np.arccos(cos_a1)
        a2 = np.arccos(cos_a2)

        angles[:, 1] = base + a1
        angles[:, 2] = -(a1 + a2)

        return angles
