import numpy as np


def semicircle_generator(radius, steps, reverse=False):
    assert (steps % 4) == 0
    halfsteps = int(steps / 2)

    step_angle = np.pi / halfsteps

    result = np.zeros((steps, 3))
    halfsteps_array = np.arange(halfsteps)

    # first half, move backward (only y change)
    result[:halfsteps, 1] = radius - halfsteps_array * radius * 2 / (halfsteps)

    # second half, move forward in semicircle shape (y, z change)
    angle = np.pi - step_angle * halfsteps_array
    result[halfsteps:, 1] = radius * np.cos(angle)
    result[halfsteps:, 2] = radius * np.sin(angle)

    result = np.roll(result, int(steps / 4), axis=0)

    if reverse:
        result = np.flip(result, axis=0)
        result = np.roll(result, 1, axis=0)

    return result

def semicircle2_generator(steps, y_radius, z_radius, x_radius, reverse=False):
    assert (steps % 4) == 0
    halfsteps = int(steps / 2)

    step_angle = np.pi / halfsteps

    result = np.zeros((steps, 3))
    halfsteps_array = np.arange(halfsteps)

    # first half, move backward (only y change)
    result[:halfsteps, 1] = y_radius - halfsteps_array * y_radius * 2 / (halfsteps)

    # second half, move forward in semicircle shape (x, y, z change)
    angle = np.pi - step_angle * halfsteps_array
    result[halfsteps:, 0] = x_radius * np.sin(angle)
    result[halfsteps:, 1] = y_radius * np.cos(angle)
    result[halfsteps:, 2] = z_radius * np.sin(angle)

    result = np.roll(result, int(steps / 4), axis=0)

    if reverse:
        result = np.flip(result, axis=0)
        result = np.roll(result, 1, axis=0)

    return result


def get_rotate_x_matrix(angle):
    return np.matrix(
        [
            [1, 0, 0, 0],
            [0, np.cos(angle), -np.sin(angle), 0],
            [0, np.sin(angle), np.cos(angle), 0],
            [0, 0, 0, 1],
        ]
    )


def get_rotate_y_matrix(angle):
    return np.matrix(
        [
            [np.cos(angle), 0, np.sin(angle), 0],
            [0, 1, 0, 0],
            [-np.sin(angle), 0, np.cos(angle), 0],
            [0, 0, 0, 1],
        ]
    )


def get_rotate_z_matrix(angle):
    return np.matrix(
        [
            [np.cos(angle), -np.sin(angle), 0, 0],
            [np.sin(angle), np.cos(angle), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ]
    )


def matrix_mul(m, pt):
    ptx = list(pt) + [1]
    return list((m * np.matrix(ptx).T).T.flat)[:-1]


def path_rotate_x(path, angle):
    ptx = np.append(path, np.ones((np.shape(path)[0], 1)), axis=1)
    return ((get_rotate_x_matrix(angle) * np.matrix(ptx).T).T)[:, :-1]


def path_rotate_y(path, angle):
    ptx = np.append(path, np.ones((np.shape(path)[0], 1)), axis=1)
    return ((get_rotate_y_matrix(angle) * np.matrix(ptx).T).T)[:, :-1]


def path_rotate_z(path, angle):
    c, s = np.cos(angle), np.sin(angle)
    rotation_matrix = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return np.dot(path, rotation_matrix.T)

