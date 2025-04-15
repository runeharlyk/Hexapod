import os
import pybullet as p
import pybullet_data
import time
import json
import numpy as np

from src.robot.kinematics import BodyStateT, gen_posture, inverse_kinematics
from src.robot.gait import GaitStateT, GaitType, gait_controller
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

while True:
    env.gui.update_gait_state(gait_state)
    env.gui.update_body_state(body_state)
    env.gui.update()

    pose = np.zeros((6, 3))

    for i in range(6):
        pose[i] = standby[i]
        if gait_state["step_x"] != 0:
            phase = (t + gait_state["offset"][i]) % 1
            pose[i] += gait_controller(gait_state, phase)
    angles = inverse_kinematics(pose, body_state, config).flatten().round(2)
    joints = np.deg2rad(angles)
    
    joints = joints.reshape(6, 3)[leg_order].flatten()

    env.step(joints)

    time.sleep(dt)
    t += dt * gait_state["step_velocity"]
    t %= 1.0
    print(t)
