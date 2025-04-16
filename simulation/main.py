import time
import json
import numpy as np

from src.robot.kinematics import BodyStateT, gen_posture, inverse_kinematics
from src.robot.gait import GaitController, GaitStateT
from src.envs.hexapod_env import HexapodEnv

with open("config.json", "r", encoding="utf-8") as read_file:
    config = json.load(read_file)

env = HexapodEnv()

leg_order = [3, 0, 4, 1, 5, 2]
standby = gen_posture(60, 75, config)

body_state = BodyStateT(omega=0, phi=0, psi=0, x=0, y=0, z=0, feet=standby, default_feet=standby)
gait_state = GaitStateT(step_height=0.04, step_x=0.2, step_z=0, step_angle=0, step_velocity=0.2, step_depth=0.002)

gait = GaitController(standby)

dt = 1./240
while True:
    env.gui.update_gait_state(gait_state)
    env.gui.update_body_state(body_state)
    env.gui.update()

    gait.step(gait_state, body_state, dt)
    angles = inverse_kinematics(body_state, config).flatten().round(2)
    joints = np.deg2rad(angles).reshape(6, 3)[leg_order].flatten()

    env.step(joints)

    time.sleep(dt)
