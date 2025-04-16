from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.monitor import Monitor

from src.envs.hexapod_env import HexapodEnv

def make_env(render_mode=None):
    env = HexapodEnv(render_mode=render_mode)
    env = Monitor(env)
    return env

env = DummyVecEnv([lambda: make_env(render_mode=None)])
env = VecNormalize(env, norm_obs=True, norm_reward=True)

model = PPO(
    "MlpPolicy",
    env,
    learning_rate=3e-4,
    n_steps=2048,
    batch_size=64,
    n_epochs=10,
    gamma=0.99,
    gae_lambda=0.95,
    clip_range=0.2,
    ent_coef=0.01,
    verbose=1,
    tensorboard_log="./logs/"
)

eval_env = DummyVecEnv([lambda: make_env(render_mode=None)])
eval_env = VecNormalize(eval_env, norm_obs=True, norm_reward=True)

eval_callback = EvalCallback(
    eval_env,
    best_model_save_path="./best_model/",
    log_path="./logs/",
    eval_freq=10000,
    deterministic=True,
    render=False
)

model.learn(
    total_timesteps=1_000_000,
    callback=eval_callback,
    progress_bar=True
)

env.close()
eval_env.close()
