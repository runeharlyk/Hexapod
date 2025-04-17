import time
import json
import numpy as np

from src.robot.kinematics import Kinematics, BodyStateT
from src.robot.gait import GaitController, GaitStateT
from src.envs.hexapod_env import HexapodEnv

with open("config.json", "r", encoding="utf-8") as read_file:
    config = json.load(read_file)

env = HexapodEnv()

leg_order = [3, 0, 4, 1, 5, 2]

kinematics = Kinematics(config)

standby = kinematics.gen_posture(np.deg2rad(60), np.deg2rad(75))

body_state = BodyStateT(omega=0, phi=0, psi=0, xm=0, ym=0, zm=0, px=0, py=0, pz=0, feet=standby, default_feet=standby)
gait_state = GaitStateT(step_height=30, step_x=0, step_z=0, step_angle=0, step_velocity=1, step_depth=0.002)

gait = GaitController(standby)

dt = 1.0 / 240
while True:
    env.gui.update_gait_state(gait_state)
    env.gui.update_body_state(body_state)
    env.gui.update()

    gait.step(gait_state, body_state, dt)
    angles = kinematics.inverse_kinematics(body_state).flatten().round(2)
    joints = angles.reshape(6, 3)[leg_order].flatten()

    _, _, done, truncated, _ = env.step(joints)
    if done or truncated:
        env.reset()

    time.sleep(dt)
