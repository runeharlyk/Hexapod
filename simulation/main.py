import os
import pybullet as p
import pybullet_data
import time
import json
import numpy as np

from src.robot.kinematics import BodyStateT, gen_posture, inverse_kinematics
from src.robot.gait import GaitStateT, gait_controller
from src.envs.hexapod_env import HexapodEnv
env = HexapodEnv()

with open("config.json", "r", encoding="utf-8") as read_file:
    config = json.load(read_file)

leg_order = [3, 0, 4, 1, 5, 2]
standby = gen_posture(60, 75, config)
joint_indices = list(range(p.getNumJoints(env.robot.robot_id)))

body_state = BodyStateT(omega=0, phi=0, psi=0, x=0, y=0, z=0)
gait_state = GaitStateT(step_height=0.04, step_x=0.2, step_z=0, step_angle=0, step_velocity=0.2, step_depth=0.002)
# gait = BezierState(num_legs=6, num_phases=2, cycle_period=1.0)

dt = 1./240
t = 0

offset = [0, 0.5, 0, 0.5, 0, 0.5] # tri-gate
stand_frac = 3.1 / 6

# offset = [0, 1/3, 2/3, 2/3, 1/3, 0] # bi-gate
# stand_frac = 2.1 / 6

# offset = [0, 1/6*1, 1/6*2, 1/6*5, 1/6*4, 1/6*3] # wave
# stand_frac = 5 / 6
while True:
    position, orientation, direction, step_height, speed, step_length = env.gui.update()
    body_state["omega"] = orientation[0]
    body_state["phi"] = orientation[1]
    body_state["psi"] = orientation[2]
    pose = np.zeros((6, 3))

    for i in range(6):
        pose[i] = standby[i]
        if step_length != 0:
            phase = (t + offset[i]) % 1
            pose[i] += gait_controller(step_length, direction, step_height, stand_frac, phase)
    angles = inverse_kinematics(pose, body_state, config).flatten().round(2)
    joints = np.deg2rad(angles)
    
    joints = joints.reshape(6, 3)[leg_order].flatten()

    env.step(joints)

    time.sleep(dt)
    t += dt * speed
    t %= 1.0
    print(t)
