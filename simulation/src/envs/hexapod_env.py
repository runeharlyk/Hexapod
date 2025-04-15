import gymnasium as gym
import pybullet as p
import pybullet_data
import numpy as np
from enum import Enum

from src.utils.gui import GUI

class TerrainType(Enum):
    FLAT = "flat"
    PLANAR_REFLECTION = "planar_reflection"
    TERRAIN = "terrain"
    MAZE = "maze"

class HexapodRobot:
    def __init__(
            self, 
            urdf_path, 
            position=[0, 0, 0.3], 
            orientation=[0, 0, 0], 
            use_fixed_base=False
        ):
        q_orientation = p.getQuaternionFromEuler(orientation)
        self.robot_id = p.loadURDF(urdf_path, position, q_orientation, useFixedBase=use_fixed_base)

    def get_observation(self):
        _, orientation = p.getBasePositionAndOrientation(self.robot_id)
        orientation = p.getEulerFromQuaternion(orientation)[:2]
        velocity, angular_velocity = p.getBaseVelocity(self.robot_id)
        joint_states = p.getJointStates(
            self.robot_id, range(p.getNumJoints(self.robot_id))
        )
        joint_positions = [state[0] for state in joint_states]
        joint_velocities = [state[1] for state in joint_states]
        return np.concatenate(
            [
                orientation,
                velocity,
                angular_velocity,
                joint_positions,
                joint_velocities,
            ]
        )
    
    def apply_action(self, action):
        for i, position in enumerate(action):
            p.setJointMotorControl2(
                bodyIndex=self.robot_id,
                jointIndex=i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=position,
                force=343, #  / 100 for newtons - Fix mass
                positionGain=0.5,
                maxVelocity=13.09,
            )

class HexapodEnv(gym.Env):
    def __init__(self, terrain_type: TerrainType = TerrainType.MAZE):
        super().__init__()
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.robot = HexapodRobot("src/resources/model.urdf")

        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(18,))
        self.action_space = gym.spaces.Box(low=-1, high=1, shape=(6,))

        self.setup_simulation()
        self.load_terrain(terrain_type)
        self.gui = GUI(self.robot.robot_id)

        self.env_start_state = p.saveState()

    def load_terrain(self, terrain_type: TerrainType):
        if terrain_type == TerrainType.FLAT:
            self.terrain = p.loadURDF("plane.urdf")
        elif terrain_type == TerrainType.PLANAR_REFLECTION:
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
            p.configureDebugVisualizer(p.COV_ENABLE_PLANAR_REFLECTION, 1)
            p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
            self.terrain = p.loadURDF("plane_transparent.urdf", useMaximalCoordinates=True)
        elif terrain_type == TerrainType.TERRAIN:
            terrainShape = p.createCollisionShape(shapeType = p.GEOM_HEIGHTFIELD, meshScale=[.1,.1,24],fileName = "heightmaps/wm_height_out.png")
            textureId = p.loadTexture("heightmaps/gimp_overlay_out.png")
            self.terrain = p.createMultiBody(0, terrainShape)
            p.changeVisualShape(self.terrain, -1, textureUniqueId = textureId)
        elif terrain_type == TerrainType.MAZE:
            terrainShape = p.createCollisionShape(shapeType = p.GEOM_HEIGHTFIELD, meshScale=[1,1,3],fileName = "heightmaps/Maze.png")
            textureId = p.loadTexture("heightmaps/Maze.png")
            maze = p.createMultiBody(0, terrainShape)
            self.terrain = [p.loadURDF("plane.urdf"), maze]
            p.changeVisualShape(self.terrain[1], -1, textureUniqueId = textureId)
    def setup_simulation(self):
        p.resetBasePositionAndOrientation(self.robot.robot_id, [0, 0, 1], [0, 0, 0, 1])
        p.setGravity(0, 0, -9.8)
        p.setTimeStep(1 / 240)

    def reset(self):
        p.restoreState(self.env_start_state)
        return self.robot.get_observation()
    
    def step(self, action):
        self.gui.update()
        self.robot.apply_action(action)
        p.stepSimulation()
        obs = self.robot.get_observation()
        reward = self.calculate_reward(obs)
        done = self.is_done(obs)
        return obs, reward, done
    
    def close(self):
        p.disconnect()

    def calculate_reward(self, obs):
        reward = 0
        return reward

    def is_done(self, obs):
        done = False
        return done