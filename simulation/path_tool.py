import numpy as np

from path_lib import semicircle_generator, semicircle2_generator
from path_lib import path_rotate_z
from path_lib import get_rotate_x_matrix, get_rotate_y_matrix, get_rotate_z_matrix


def gen_walk_path(standby_coordinate, g_steps=448, g_radius=30, direction=0):
    assert (g_steps % 4) == 0
    halfsteps = int(g_steps / 2)

    semi_circle = semicircle_generator(g_radius, g_steps)

    semi_circle = np.array(path_rotate_z(semi_circle, direction))
    mir_path = np.roll(semi_circle, halfsteps, axis=0)

    path = np.zeros((g_steps, 6, 3))
    path[:, [0, 2, 4], :] = np.tile(semi_circle[:, np.newaxis, :], (1, 3, 1))
    path[:, [1, 3, 5], :] = np.tile(mir_path[:, np.newaxis, :], (1, 3, 1))

    return path + np.tile(standby_coordinate, (g_steps, 1, 1))


def gen_fastwalk_path(
    standby_coordinate, g_steps=20, y_radius=50, z_radius=40, x_radius=15, reverse=False
):
    assert (g_steps % 2) == 0

    halfsteps = int(g_steps / 2)

    path = np.zeros((g_steps, 6, 3))
    semi_circle_r = semicircle2_generator(g_steps, y_radius, z_radius, x_radius, reverse=reverse)
    semi_circle_l = semicircle2_generator(g_steps, y_radius, z_radius, -x_radius, reverse=reverse)

    path[:, [0, 2], :] = np.tile(semi_circle_r[:, np.newaxis, :], (1, 2, 1))
    path[:, 1, :] = np.roll(semi_circle_r, halfsteps, axis=0)
    path[:, 4, :] = semi_circle_l
    path[:, [3, 5], :] = np.tile(np.roll(semi_circle_l[:, np.newaxis, :], halfsteps, axis=0), (1, 2, 1))

    return path + np.tile(standby_coordinate, (g_steps, 1, 1))


def gen_turn_path(standby_coordinate, g_steps=448, g_radius=35, direction="left"):
    assert (g_steps % 4) == 0
    halfsteps = int(g_steps / 2)

    path = np.zeros((g_steps, 6, 3))

    semi_circle = semicircle_generator(g_radius, g_steps)
    mir_path = np.roll(semi_circle, halfsteps, axis=0)

    if direction == "left":
        path[:, 0, :] = path_rotate_z(semi_circle, 45)
        path[:, 1, :] = path_rotate_z(mir_path, 0)
        path[:, 2, :] = path_rotate_z(semi_circle, 315)
        path[:, 5, :] = path_rotate_z(mir_path, 225)
        path[:, 4, :] = path_rotate_z(semi_circle, 180)
        path[:, 3, :] = path_rotate_z(mir_path, 135)
    elif direction == "right":
        path[:, 0, :] = path_rotate_z(semi_circle, 45 + 180)
        path[:, 1, :] = path_rotate_z(mir_path, 0 + 180)
        path[:, 2, :] = path_rotate_z(semi_circle, 315 + 180)
        path[:, 5, :] = path_rotate_z(mir_path, 225 + 180)
        path[:, 4, :] = path_rotate_z(semi_circle, 180 + 180)
        path[:, 3, :] = path_rotate_z(mir_path, 135 + 180)

    return path + np.tile(standby_coordinate, (g_steps, 1, 1))


def gen_climb_path(
    standby_coordinate,
    g_steps=448,
    y_radius=20,
    z_radius=80,
    x_radius=30,
    z_shift=-30,
    reverse=False,
):
    assert (g_steps % 4) == 0
    halfsteps = int(g_steps / 2)

    rpath = semicircle2_generator(g_steps, y_radius, z_radius, x_radius, reverse=reverse)
    rpath[:, 2] = rpath[:, 2] + z_shift

    lpath = semicircle2_generator(g_steps, y_radius, z_radius, -x_radius, reverse=reverse)
    lpath[:, 2] = lpath[:, 2] + z_shift

    mir_rpath = np.roll(rpath, halfsteps, axis=0)
    mir_lpath = np.roll(lpath, halfsteps, axis=0)

    path = np.zeros((g_steps, 6, 3))
    path[:, 0, :] = rpath
    path[:, 1, :] = mir_rpath
    path[:, 2, :] = rpath
    path[:, 3, :] = mir_lpath
    path[:, 4, :] = lpath
    path[:, 5, :] = mir_lpath

    return path + np.tile(standby_coordinate, (g_steps, 1, 1))


def gen_rotatex_path(standby_coordinate, g_steps=448, swing_angle=15, y_radius=15):
    assert (g_steps % 4) == 0
    quarter = int(g_steps / 4)

    path = np.zeros((g_steps, 6, 3))

    step_angle = swing_angle / quarter
    step_offset = y_radius / quarter

    scx = np.append(standby_coordinate, np.ones((6, 1)), axis=1)

    for i in range(quarter):
        m = get_rotate_x_matrix(swing_angle - i * step_angle)
        m[1, 3] = -i * step_offset

        path[i, :, :] = ((np.matmul(m, scx.T)).T)[:, :-1]

    for i in range(quarter):
        m = get_rotate_x_matrix(-i * step_angle)
        m[1, 3] = -y_radius + i * step_offset

        path[i + quarter, :, :] = ((np.matmul(m, scx.T)).T)[:, :-1]

    for i in range(quarter):
        m = get_rotate_x_matrix(i * step_angle - swing_angle)
        m[1, 3] = i * step_offset

        path[i + quarter * 2, :, :] = ((np.matmul(m, scx.T)).T)[:, :-1]

    for i in range(quarter):
        m = get_rotate_x_matrix(i * step_angle)
        m[1, 3] = y_radius - i * step_offset

        path[i + quarter * 3, :, :] = ((np.matmul(m, scx.T)).T)[:, :-1]

    return path


def gen_rotatey_path(standby_coordinate, g_steps=448, swing_angle=15, x_radius=15):
    assert (g_steps % 4) == 0
    quarter = int(g_steps / 4)

    path = np.zeros((g_steps, 6, 3))

    step_angle = swing_angle / quarter
    step_offset = x_radius / quarter

    scx = np.append(standby_coordinate, np.ones((6, 1)), axis=1)

    for i in range(quarter):
        m = get_rotate_y_matrix(swing_angle - i * step_angle)
        m[1, 3] = -i * step_offset

        path[i, :, :] = ((np.matmul(m, scx.T)).T)[:, :-1]

    for i in range(quarter):
        m = get_rotate_y_matrix(-i * step_angle)
        m[1, 3] = -x_radius + i * step_offset

        path[i + quarter, :, :] = ((np.matmul(m, scx.T)).T)[:, :-1]

    for i in range(quarter):
        m = get_rotate_y_matrix(i * step_angle - swing_angle)
        m[1, 3] = i * step_offset

        path[i + quarter * 2, :, :] = ((np.matmul(m, scx.T)).T)[:, :-1]

    for i in range(quarter):
        m = get_rotate_y_matrix(i * step_angle)
        m[1, 3] = x_radius - i * step_offset

        path[i + quarter * 3, :, :] = ((np.matmul(m, scx.T)).T)[:, :-1]

    return path


def gen_rotatez_path(standby_coordinate, g_steps=448, z_lift=4.5, xy_radius=1):
    assert (g_steps % 4) == 0

    path = np.zeros((g_steps, 6, 3))

    step_angle = 2 * np.pi / g_steps
    scx = np.append(standby_coordinate, np.ones((6, 1)), axis=1)

    for i in range(g_steps):
        x = xy_radius * np.cos(i * step_angle)
        y = xy_radius * np.sin(i * step_angle)

        m = get_rotate_y_matrix(
            np.arctan2(x, z_lift) * 180 / np.pi
        ) * get_rotate_x_matrix(np.arctan2(y, z_lift) * 180 / np.pi)

        path[i, :, :] = ((np.matmul(m, scx.T)).T)[:, :-1]

    return path


def gen_twist_path(
    standby_coordinate, g_steps=448, raise_angle=3, twist_x_angle=20, twise_y_angle=12
):
    assert (g_steps % 4) == 0

    quarter = int(g_steps / 4)
    step_x_angle = twist_x_angle / quarter
    step_y_angle = twise_y_angle / quarter
    scx = np.append(standby_coordinate, np.ones((6, 1)), axis=1)

    m = get_rotate_x_matrix(raise_angle)

    path = np.zeros((g_steps, 6, 3))

    for i in range(quarter):
        temp = (
            m
            * get_rotate_z_matrix(i * step_x_angle)
            * get_rotate_x_matrix(i * step_y_angle)
        )

        path[i, :, :] = ((np.matmul(temp, scx.T)).T)[:, :-1]

    for i in range(quarter):
        temp = (
            m
            * get_rotate_z_matrix((quarter - i) * step_x_angle)
            * get_rotate_x_matrix((quarter - i) * step_y_angle)
        )

        path[i + quarter * 1, :, :] = ((np.matmul(temp, scx.T)).T)[:, :-1]

    for i in range(quarter):
        temp = (
            m
            * get_rotate_z_matrix(-i * step_x_angle)
            * get_rotate_x_matrix(i * step_y_angle)
        )

        path[i + quarter * 2, :, :] = ((np.matmul(temp, scx.T)).T)[:, :-1]

    for i in range(quarter):
        temp = (
            m
            * get_rotate_z_matrix((-quarter + i) * step_x_angle)
            * get_rotate_x_matrix((quarter - i) * step_y_angle)
        )

        path[i + quarter * 3, :, :] = ((np.matmul(temp, scx.T)).T)[:, :-1]

    return path


def gen_standup_path(standby_coordinate, laydown_coordinate, steps=28):
    standing_up_lut_size = steps

    lift_up_size = 10
    adjust_leg_size = int((standing_up_lut_size - lift_up_size) / 2)

    lift_up_linspace = np.linspace(
        laydown_coordinate[0, 2], standby_coordinate[0, 2], lift_up_size
    )

    lut_standup = np.zeros((standing_up_lut_size, 6, 3))

    for idx, var in enumerate(lift_up_linspace):
        laydown_coordinate[:, 2] = var
        lut_standup[idx, :, :] = laydown_coordinate

    radius = (lut_standup[lift_up_size - 1, 1, 0] - standby_coordinate[1, 0]) / 2

    step_angle = np.pi / adjust_leg_size
    angle = np.arange(1, adjust_leg_size + 1) * step_angle

    z_offset = radius * np.sin(angle)
    x_offset = radius * np.cos(angle) - radius

    r_leg2_offset = np.zeros((adjust_leg_size, 3))
    r_leg2_offset[:, 0] = x_offset
    r_leg2_offset[:, 2] = z_offset

    r_leg1_offset = path_rotate_z(r_leg2_offset, 45)
    r_leg3_offset = path_rotate_z(r_leg2_offset, -45)

    l_leg1_offset = path_rotate_z(r_leg2_offset, 135)
    l_leg2_offset = path_rotate_z(r_leg2_offset, 180)
    l_leg3_offset = path_rotate_z(r_leg2_offset, -135)

    for idx in range(0, adjust_leg_size):
        lut_standup[idx + lift_up_size, :, :] = lut_standup[lift_up_size - 1, :, :]
        lut_standup[idx + lift_up_size, 3, :] = (
            lut_standup[idx + lift_up_size, 3, :] + l_leg1_offset[idx, :]
        )
        lut_standup[idx + lift_up_size, 1, :] = (
            lut_standup[idx + lift_up_size, 1, :] + r_leg2_offset[idx, :]
        )
        lut_standup[idx + lift_up_size, 5, :] = (
            lut_standup[idx + lift_up_size, 5, :] + l_leg3_offset[idx, :]
        )

    for idx in range(0, adjust_leg_size):
        lut_standup[idx + lift_up_size + adjust_leg_size, :, :] = lut_standup[
            lift_up_size + adjust_leg_size - 1, :, :
        ]

        lut_standup[idx + lift_up_size + adjust_leg_size, 0, :] = (
            lut_standup[idx + lift_up_size + adjust_leg_size, 0, :]
            + r_leg1_offset[idx, :]
        )
        lut_standup[idx + lift_up_size + adjust_leg_size, 4, :] = (
            lut_standup[idx + lift_up_size + adjust_leg_size, 4, :]
            + l_leg2_offset[idx, :]
        )
        lut_standup[idx + lift_up_size + adjust_leg_size, 2, :] = (
            lut_standup[idx + lift_up_size + adjust_leg_size, 2, :]
            + r_leg3_offset[idx, :]
        )

    return lut_standup


def gen_posture(j2_angle, j3_angle, config):
    mount_x = np.array(config["legMountX"])
    mount_y = np.array(config["legMountY"])
    root_j1 = config["legRootToJoint1"]
    j1_j2 = config["legJoint1ToJoint2"]
    j2_j3 = config["legJoint2ToJoint3"]
    j3_tip = config["legJoint3ToTip"]
    mount_angle = np.array(config["legMountAngle"]) / 180 * np.pi

    j2_rad = j2_angle / 180 * np.pi
    j3_rad = j3_angle / 180 * np.pi
    posture = np.zeros((6, 3))

    posture[:, 0] = mount_x + (
        root_j1 + j1_j2 + (j2_j3 * np.sin(j2_rad)) + j3_tip * np.cos(j3_rad)
    ) * np.cos(mount_angle)
    posture[:, 1] = mount_y + (
        root_j1 + j1_j2 + (j2_j3 * np.sin(j2_rad)) + j3_tip * np.cos(j3_rad)
    ) * np.sin(mount_angle)
    posture[:, 2] = j2_j3 * np.cos(j2_rad) - j3_tip * np.sin(j3_rad)
    return posture