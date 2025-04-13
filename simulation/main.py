import math
import os
import pybullet as p
import pybullet_data
import time
import json
import numpy as np
from kinematics import BodyStateT, compute_default_position, gen_posture, inverse_kinematics, whole_body_kinematics
from gait import BezierState, GaitStateT, compute_gait_positions

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
model_path = os.path.join(os.path.dirname(__file__), "model.urdf")
robot_id = p.loadURDF(model_path, useFixedBase=True)

with open("config.json", "r", encoding="utf-8") as read_file:
    config = json.load(read_file)

standby = gen_posture(60, 75, config)

# p.resetDebugVisualizerCamera( # bottom view
#     cameraDistance=3,
#     cameraYaw=90,
#     cameraPitch=-89,
#     cameraTargetPosition=[0, 0, 0]
# )

# p.resetDebugVisualizerCamera( # Front view
#     cameraDistance=3.0,
    #     cameraYaw=0,          
#     cameraPitch=-20,      
#     cameraTargetPosition=[0, 0, 0]  
# )

# p.resetDebugVisualizerCamera( # Side view
#     cameraDistance=3.0,
#     cameraYaw=90,
#     cameraPitch=0,
#     cameraTargetPosition=[0, 0, 0]
# )

def get_rotate_z_matrix(angle):
    """Returns a 4x4 rotation matrix for rotation around the z-axis.

    Args:
        angle (float): The angle of rotation in degrees.

    Returns:
        numpy.matrix: A 4x4 rotation matrix.
    """
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
    """Rotates a path around the z-axis.

    Args:
        path (numpy.ndarray): A 2D array representing the path.
            The shape of the array is (steps, 3), where:
                - steps is the number of steps in the path.
                - 3 is the number of coordinates (x, y, z).
        angle (float): The angle of rotation in degrees.

    Returns:
        numpy.ndarray: The rotated path.
    """
    ptx = np.append(path, np.ones((np.shape(path)[0], 1)), axis=1)
    return ((get_rotate_z_matrix(angle) * np.matrix(ptx).T).T)[:, :-1]

def semicircle_generator(radius, steps, reverse=False):
    """Generates a semicircle path with specified radius and steps.

    Args:
        radius (float): The radius of the semicircle.
        steps (int): The number of steps in the path. Must be divisible by 4.
        reverse (bool, optional): Whether to reverse the path. Defaults to False.

    Returns:
        numpy.ndarray: A 2D array representing the semicircle path.
        The shape of the array is (steps, 3), where:
            - steps is the number of steps in the path.
            - 3 is the number of coordinates (x, y, z).

    Raises:
        AssertionError: If steps is not divisible by 4.
    """
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

def gen_walk_path(standby_coordinate, g_steps=112, g_radius=30, direction=0):
    """Generates a walking path for the hexapod.

    Args:
        standby_coordinate (numpy.ndarray): The standby coordinate of the hexapod.
        g_steps (int, optional): The number of steps in the path. Defaults to 28.
        g_radius (int, optional): The radius of the walking circle. Defaults to 35.
        direction (int, optional): The direction of the walking path. Defaults to 0.

    Returns:
        numpy.ndarray: A 3D array representing the walking path.
        The shape of the array is (g_steps, 6, 3), where:
            - g_steps is the number of steps in the path.
            - 6 is the number of legs.
            - 3 is the number of coordinates (x, y, z).

    Raises:
        AssertionError: If g_steps is not divisible by 4.
    """
    assert (g_steps % 4) == 0
    halfsteps = int(g_steps / 2)

    semi_circle = semicircle_generator(g_radius, g_steps)

    semi_circle = np.array(path_rotate_z(semi_circle, direction))
    mir_path = np.roll(semi_circle, halfsteps, axis=0)

    path = np.zeros((g_steps, 6, 3))
    path[:, [0, 2, 4], :] = np.tile(semi_circle[:, np.newaxis, :], (1, 3, 1))
    path[:, [1, 3, 5], :] = np.tile(mir_path[:, np.newaxis, :], (1, 3, 1))

    return path + np.tile(standby_coordinate, (g_steps, 1, 1))

joint_indices = list(range(p.getNumJoints(robot_id)))

default_position = compute_default_position()

body_state = BodyStateT(omega=0, phi=0, psi=0, xm=0, ym=0, zm=100, feet=default_position)#gen_posture(60, 75))
gait_state = GaitStateT(step_height=0.04, step_x=0.2, step_z=0, step_angle=0, step_velocity=0.2, step_depth=0.002)

gait = BezierState(num_legs=6, num_phases=2, cycle_period=1.0)

lut_walk_0 = gen_walk_path(standby, direction=0)

dt = 1./240
t = 0
while True:
    # gait.step(body_state, gait_state, dt)
    # body_state["feet"][0][0] += 0.01 * math.sin(t)

    # body_state["feet"][0][0] = default_position[0][0] + 0.05 * math.sin(t*10)

    # joints = whole_body_kinematics(body_state)
    # print(np.rad2deg(joints))
    # break
    print(t * 800)
    pose = lut_walk_0[int(t * 800) % 112]
    angles = inverse_kinematics(pose, config).flatten().round(2)
    print(angles)
    joints = np.deg2rad(angles)
    leg_order = [3, 0, 4, 1, 5, 2]#[4, 5, 3, 1, 0, 2]
    joints = joints.reshape(6,3)[leg_order].flatten()
    # joints[15:] = 0

    for i, j in enumerate(joint_indices):
        p.resetJointState(robot_id, j, joints[i])
    
    for foot_pos in body_state["feet"]:
        p.addUserDebugPoints([foot_pos], [[1, 0, 0]], pointSize=10)
    
    p.stepSimulation()
    time.sleep(dt)
    t += dt
