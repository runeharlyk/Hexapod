import numpy as np
from path_lib import semicircle_generator


def get_rotate_z_matrix(angle):
    angle = angle * np.pi / 180
    return np.matrix(
        [
            [np.cos(angle), -np.sin(angle), 0, 0],
            [np.sin(angle), np.cos(angle), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ]
    )

def path_rotate_z(path, angle):
    ptx = np.append(path, np.ones((np.shape(path)[0], 1)), axis=1)
    return ((get_rotate_z_matrix(angle) * np.matrix(ptx).T).T)[:, :-1]

def gen_walk_path(standby_coordinate, g_steps=112, g_radius=30, direction=0):
    assert (g_steps % 4) == 0
    halfsteps = int(g_steps / 2)

    semi_circle = semicircle_generator(g_radius, g_steps)

    semi_circle = np.array(path_rotate_z(semi_circle, direction))
    mir_path = np.roll(semi_circle, halfsteps, axis=0)

    path = np.zeros((g_steps, 6, 3))
    path[:, [0, 2, 4], :] = np.tile(semi_circle[:, np.newaxis, :], (1, 3, 1))
    path[:, [1, 3, 5], :] = np.tile(mir_path[:, np.newaxis, :], (1, 3, 1))

    return path + np.tile(standby_coordinate, (g_steps, 1, 1))