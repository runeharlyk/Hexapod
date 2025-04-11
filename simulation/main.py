import math
import os
import pybullet as p
import pybullet_data
import time

from kinematics import BodyStateT, compute_default_position, gen_posture, whole_body_kinematics
from gait import BezierState, GaitStateT, compute_gait_positions

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
model_path = os.path.join(os.path.dirname(__file__), "model.urdf")
robot_id = p.loadURDF(model_path, useFixedBase=True)

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

joint_indices = list(range(p.getNumJoints(robot_id)))

body_state = BodyStateT(omega=0, phi=0, psi=0, xm=0, ym=0, zm=100, feet=compute_default_position())#gen_posture(60, 75))
gait_state = GaitStateT(step_height=0.4, step_x=2, step_z=0, step_angle=0, step_velocity=0.2, step_depth=0.02)

gait = BezierState(num_legs=6, num_phases=2, cycle_period=1.0)

dt = 1./240
t = 0
while True:
    # gait.step(body_state, gait_state, dt)
    # body_state["feet"][0][0] += 0.01 * math.sin(t)

    joints = whole_body_kinematics(body_state)
    for i, j in enumerate(joint_indices):
        p.resetJointState(robot_id, j, joints[i])
    p.stepSimulation()
    time.sleep(dt)
    t += dt
