import os
import pybullet as p
import pybullet_data
import time
import json
import numpy as np

from kinematics import gen_posture, inverse_kinematics
from gait import BezierState, GaitStateT
from path_tool import gen_walk_path

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
model_path = os.path.join(os.path.dirname(__file__), "model.urdf")
robot_id = p.loadURDF(model_path, useFixedBase=True)
direction_slider = p.addUserDebugParameter("Direction", 0, 360, 0)
step_height_slider = p.addUserDebugParameter("Step height", 0, 0.1, 0.04)
speed_slider = p.addUserDebugParameter("Speed", 0, 0.5, 0.2)

with open("config.json", "r", encoding="utf-8") as read_file:
    config = json.load(read_file)

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

leg_order = [3, 0, 4, 1, 5, 2]
standby = gen_posture(60, 75, config)
joint_indices = list(range(p.getNumJoints(robot_id)))

gait_state = GaitStateT(step_height=0.04, step_x=0.2, step_z=0, step_angle=0, step_velocity=0.2, step_depth=0.002)
gait = BezierState(num_legs=6, num_phases=2, cycle_period=1.0)


dt = 1./240
t = 0
while True:
    direction = p.readUserDebugParameter(direction_slider)
    lut_walk_0 = gen_walk_path(standby, direction=direction)
    pose = lut_walk_0[int(t * 400) % 112]
    angles = inverse_kinematics(pose, config).flatten().round(2)
    joints = np.deg2rad(angles)
    
    joints = joints.reshape(6,3)[leg_order].flatten()

    for i, j in enumerate(joint_indices):
        p.resetJointState(robot_id, j, joints[i])
    
    p.stepSimulation()
    time.sleep(dt)
    t += dt
