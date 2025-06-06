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
    def __init__(self, urdf_path, position=[0, 0, 0.3], orientation=[0, 0, 0], use_fixed_base=False):
        q_orientation = p.getQuaternionFromEuler(orientation)
        self.robot_id = p.loadURDF(urdf_path, position, q_orientation, useFixedBase=use_fixed_base)

    def get_observation(self):
        position, orientation = p.getBasePositionAndOrientation(self.robot_id)
        orientation = p.getEulerFromQuaternion(orientation)
        velocity, angular_velocity = p.getBaseVelocity(self.robot_id)
        joint_states = p.getJointStates(self.robot_id, range(p.getNumJoints(self.robot_id)))
        joint_positions = [state[0] for state in joint_states]
        joint_velocities = [state[1] for state in joint_states]
        return np.concatenate(
            [
                position,
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
                force=50,  # 343 #  / 100 for newtons - Fix mass
                positionGain=0.5,
                maxVelocity=13.09,
            )


class HexapodEnv(gym.Env):
    def __init__(self, terrain_type: TerrainType = TerrainType.FLAT, render_mode: str = "human"):
        super().__init__()
        if render_mode == "human":
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)

        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(48,))
        self.action_space = gym.spaces.Box(low=-1, high=1, shape=(18,))
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        self.terrain_type = terrain_type
        self.render_mode = render_mode
        self.target_velocity = 0.5
        self.max_steps = float("inf")
        self.current_step = 0

        self._setup_world()
        if render_mode == "human":
            self.env_start_state = p.saveState()

        # env parameters
        self._distance_limit = float("inf")

    def _setup_world(self):
        self.robot = HexapodRobot("src/resources/model.urdf")
        self._load_terrain(self.terrain_type)
        p.setGravity(0, 0, -9.8)
        p.setTimeStep(1 / 240)
        if self.render_mode == "human":
            self.gui = GUI(self.robot.robot_id)
        else:
            self.gui = None

    def _load_terrain(self, terrain_type: TerrainType):
        if terrain_type == TerrainType.FLAT:
            self.terrain = p.loadURDF("plane.urdf")
        elif terrain_type == TerrainType.PLANAR_REFLECTION:
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
            p.configureDebugVisualizer(p.COV_ENABLE_PLANAR_REFLECTION, 1)
            p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
            self.terrain = p.loadURDF("plane_transparent.urdf", useMaximalCoordinates=True)
        elif terrain_type == TerrainType.TERRAIN:
            terrainShape = p.createCollisionShape(
                shapeType=p.GEOM_HEIGHTFIELD, meshScale=[0.1, 0.1, 24], fileName="heightmaps/wm_height_out.png"
            )
            textureId = p.loadTexture("heightmaps/gimp_overlay_out.png")
            self.terrain = p.createMultiBody(0, terrainShape)
            p.changeVisualShape(self.terrain, -1, textureUniqueId=textureId)
        elif terrain_type == TerrainType.MAZE:
            terrainShape = p.createCollisionShape(
                shapeType=p.GEOM_HEIGHTFIELD, meshScale=[1, 1, 3], fileName="heightmaps/Maze.png"
            )
            textureId = p.loadTexture("heightmaps/Maze.png")
            maze = p.createMultiBody(0, terrainShape)
            self.terrain = [p.loadURDF("plane.urdf"), maze]
            p.changeVisualShape(self.terrain[1], -1, textureUniqueId=textureId)

    def reset(self, *, seed: int | None = None):
        super().reset(seed=seed)
        if self.render_mode == "human":
            p.restoreState(self.env_start_state)
        else:
            p.resetSimulation()
            self._setup_world()
        self.current_step = 0
        return self.robot.get_observation(), {}

    def step(self, action):
        self.current_step += 1
        if self.gui:
            self.gui.update()
        self.robot.apply_action(action)
        p.stepSimulation()

        obs = self.robot.get_observation()
        reward = self.calculate_reward(obs)
        done = self.is_done(obs)
        truncated = self.current_step >= self.max_steps

        return obs, reward, done, truncated, {}

    def close(self):
        pass
        # p.disconnect()

    def calculate_reward(self, obs):
        position = obs[:3]
        velocity = obs[6:9]
        angular_velocity = obs[9:12]

        forward_velocity = velocity[0]
        velocity_reward = -abs(forward_velocity - self.target_velocity)

        height_penalty = -abs(position[2] - 0.3)

        angular_penalty = -np.sum(np.square(angular_velocity))

        total_reward = velocity_reward + 0.1 * height_penalty + 0.01 * angular_penalty
        return total_reward

    def is_done(self, obs):
        position = obs[:3]
        orientation = obs[3:6]
        return self._is_fallen(orientation) or self._is_distance_limit_exceeded(position)

    def _is_distance_limit_exceeded(self, position):
        distance = np.hypot(position[0], position[1])
        return distance > self._distance_limit

    def _is_fallen(self, orientation):
        # orientation = self.spot.GetBaseOrientation()
        # rot_mat = self._pybullet_client.getMatrixFromQuaternion(orientation)
        # local_up = rot_mat[6:]
        # pos = self.spot.GetBasePosition()
        # return (np.dot(np.asarray([0, 0, 1]), np.asarray(local_up)) < 0.55)
        return abs(orientation[0]) > 0.85 or abs(orientation[1]) > 0.85
