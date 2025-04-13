import numpy as np


def semicircle_generator(radius, steps, reverse=False):
    assert (steps % 4) == 0
    halfsteps = int(steps / 2)
    step_angle = np.pi / halfsteps
    result = np.zeros((steps, 3))
    halfsteps_array = np.arange(halfsteps)

    result[:halfsteps, 1] = radius - halfsteps_array * radius * 2 / (halfsteps)

    angle = np.pi - step_angle * halfsteps_array
    result[halfsteps:, 1] = radius * np.cos(angle)
    result[halfsteps:, 2] = radius * np.sin(angle)

    result = np.roll(result, int(steps / 4), axis=0)

    if reverse:
        result = np.flip(result, axis=0)
        result = np.roll(result, 1, axis=0)

    return result