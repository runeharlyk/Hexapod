import pybullet as p
import numpy as np

class GUI:
    def __init__(self, bot):
        self.bot = bot
        self.c_yaw = 0
        self.c_pitch = -7
        self.c_distance = 5


        self.x_slider = p.addUserDebugParameter("x", -0.1, 0.1, 0)
        self.y_slider = p.addUserDebugParameter("y", -0.1, 0.1, 0)
        self.z_slider = p.addUserDebugParameter("z", -0.1, 0.1, 0)
        self.yaw_slider = p.addUserDebugParameter("yaw", -np.pi/4, np.pi/4, 0)
        self.pitch_slider = p.addUserDebugParameter("pitch", -np.pi/4, np.pi/4, 0)
        self.roll_slider = p.addUserDebugParameter("roll", -np.pi/4, np.pi/4, 0)

        self.direction_slider = p.addUserDebugParameter("Direction", -np.pi, np.pi, 0)
        self.step_height_slider = p.addUserDebugParameter("Step height", 0, 0.1, 0.04)
        self.speed_slider = p.addUserDebugParameter("Speed", 0, 2, 1)



    def update(self):
        quadruped_pos, _ = p.getBasePositionAndOrientation(self.bot)
        p.resetDebugVisualizerCamera(
            cameraDistance=self.c_distance,
            cameraYaw=self.c_yaw,
            cameraPitch=self.c_pitch,
            cameraTargetPosition=quadruped_pos
        )

        keys = p.getKeyboardEvents()
        if keys.get(ord('j')):
            self.c_yaw += 0.1
        if keys.get(ord('k')):
            self.c_yaw -= 0.1
        if keys.get(ord('m')):
            self.c_pitch += 0.1
        if keys.get(ord('i')):
            self.c_pitch -= 0.1
        
        if keys.get(ord('q')) or keys.get(27):
            p.disconnect()
            exit()

        position = np.array([
            p.readUserDebugParameter(self.x_slider),
            p.readUserDebugParameter(self.y_slider),
            p.readUserDebugParameter(self.z_slider)
        ])

        orientation = np.array([
            p.readUserDebugParameter(self.roll_slider),
            p.readUserDebugParameter(self.pitch_slider),
            p.readUserDebugParameter(self.yaw_slider),
        ])

        direction, step_height, speed, = (p.readUserDebugParameter(s) for s in (self.direction_slider, self.step_height_slider, self.speed_slider))

        return position, orientation, direction, step_height, speed


