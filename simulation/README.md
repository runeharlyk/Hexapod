# Hexapod Simulation

MuJoCo simulation + RL training for the hexapod, designed for **sim-to-real** transfer to
the ESP32 robot. The policy observes only what the real robot can sense (IMU + gyro; open-loop
servos have no encoders) and acts through the firmware gait/IK.

## Setup (uv)

```sh
uv sync                 # core MuJoCo + SB3 + torch (cu118) stack
uv sync --extra legacy  # also install pybullet for the old main.py / hexapod_env.py
```

Run anything with `uv run`, e.g. `uv run python train_mj.py --smoke`.

## Workflow

```sh
# 1. (Re)generate the MuJoCo model from firmware geometry
uv run python src/resources/build_model.py            # model.xml (flat plane)
uv run python src/resources/build_model.py --terrain  # + model_terrain.xml (heightfield ground)

# 2. Classical firmware gait walking in MuJoCo (no RL)
uv run python replay_gait.py                 # viewer
uv run python replay_gait.py --headless --ly 0.5

# 3. (Optional) retune the analytic command->gait coefficients (writes resources/gait_coef.json)
uv run python optimize_gait.py

# 4. Train a walking policy (SB3 PPO, GPU policy + CPU SubprocVecEnv)
uv run python train_mj.py --control-mode residual_pure --randomize \
    --timesteps 8_000_000 --num-envs 16 --resample-steps 300 \
    --zero-final --init-std 0.3 --target-kl 0.02 --tag residual_pure_stab
uv run python train_mj.py --smoke            # quick end-to-end sanity check
#   tensorboard --logdir runs

# 5. Evaluate / watch a trained policy
uv run python eval_policy.py --run <name> --control-mode residual_pure            # viewer
uv run python eval_policy.py --run <name> --control-mode residual_pure --headless # metrics + gates
uv run python eval_policy.py --run <name> --headless --randomize --seeds 10       # robustness gate
uv run python eval_policy.py --run <name> --headless --push 5 --seeds 10          # push stress gate
uv run python eval_policy.py --run <name> --headless --terrain 0.02 --seeds 10    # 20mm bump terrain
uv run python eval_policy.py --zero-action --control-mode residual_pure --headless # analytic baseline
uv run python eval_policy.py --run <name> --cmd 0.12 0 0                          # fixed command
uv run python eval_policy.py --run <name> --video walk.mp4 --cmd 0.1 0 0.3        # save video

# 6. Export the trained actor as a dependency-free C++ header for the firmware
uv run python export_policy.py --run <name>
```

## Control modes

The command is a body-frame velocity vector `[vx, vy]` (m/s) + yaw rate (rad/s).

- **`residual_pure`** (deploy target) — gait settings come from the analytic command→gait map
  (`analytic_gait_action`, coefficients in `resources/gait_coef.json`, tuned by `optimize_gait.py`);
  the policy outputs only 6×3 foot residuals (±20 mm) on top.
  Zero action reproduces the classical gait exactly, so train with `--zero-final --init-std 0.3`
  to start at the analytic gait and learn pure stabilization.
- **`phase_gait`** — policy outputs gait settings `[step_x, step_z, step_angle, step_height,
  gait_blend, phase_rate]` and drives the firmware gait engine's phase → IK. Low-dim, constrained.
- **`residual`** — `phase_gait` settings + 6×3 foot residuals (policy owns both).
- **`foot`** — policy outputs 6×3 foot-position offsets → IK. Full per-leg authority; learns the gait.

## Layout

- `src/robot/firmware_gait.py` — NumPy port of the firmware gait + IK (the deploy target).
- `src/resources/build_model.py` → `model.xml` — MuJoCo model (0.68 kg; legs 0.077 kg each).
- `src/sim/mj_runtime.py` — MuJoCo runtime wrapper.
- `src/sim/domain_rand.py` — domain randomization (mass/friction/motor/latency/IMU/pushes).
- `src/sim/terrain.py` — per-episode random heightfield bumps (`--terrain` in train/eval; the policy
  is blind, so terrain robustness comes from the IMU + gait, not exteroception).
- `src/envs/hexapod_mj_env.py` — Gymnasium env (hardware-only observation).
- `train_mj.py` / `eval_policy.py` / `replay_gait.py` — train / eval / classical replay.
- `optimize_gait.py` / `export_policy.py` — tune the analytic gait map / export the actor to C++.
- Legacy (PyBullet): `main.py`, `src/envs/hexapod_env.py`, `train.py`.

## Create URDF from xacro

```sh
uv run python src/utils/xacro.py -o src/resources/model.urdf src/resources/model.xacro
```
