# Hexapod Simulation

MuJoCo simulation + RL training for the hexapod, designed for **sim-to-real** transfer to
the ESP32 robot. The policy observes only what the real robot can sense (IMU + gyro; open-loop
servos have no encoders) and acts through the firmware gait/IK.

## Setup (uv)

```sh
uv sync                 # core MuJoCo + SB3 + torch (cu118) stack, plus pytest (dev group)
```

Run anything with `uv run`, e.g. `uv run python train_mj.py --smoke`.
`uv run pytest -q` runs the sim-to-firmware parity tests (`test_firmware_gait_parity.py`).

## Workflow

```sh
# 1. (Re)generate the MuJoCo model from firmware geometry
uv run python src/resources/build_model.py            # model.xml (flat plane)
uv run python src/resources/build_model.py --terrain  # + model_terrain.xml (heightfield ground)

# 2. Classical firmware gait walking in MuJoCo (no RL)
uv run python replay_gait.py                 # viewer
uv run python replay_gait.py --headless --ly 0.5
uv run python replay_gait.py --headless --gait tuned   # the searched gait GaitType::TUNED runs

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

# 6. Rough-terrain / obstacle benchmark (progress-based; see below)
uv run python bench_terrain.py --runs <a> <b> --baseline --workers 8
uv run python bench_terrain.py --runs <a> <b> --baseline --curb   # how tall a step it can climb

# 7. Export the trained actor as a dependency-free C++ header for the firmware
uv run python export_policy.py --run <name>
```

## Rough terrain

The heightfield generator (`src/sim/terrain.py`, needs `model_terrain.xml`) provides four training
grounds plus one measurement fixture, all scaled by `--terrain <max height m>`:

| kind    | what it is                        | `--terrain-feature` controls |
| ------- | --------------------------------- | ---------------------------- |
| `bumps` | smooth random hills               | spatial frequency            |
| `rocks` | scattered flat-topped blocks      | density                      |
| `steps` | tiled plateaus, sharp edges       | tiles per metre              |
| `waves` | sinusoidal rolling ground         | waves per metre              |
| `curb`  | one full-width step 0.6 m ahead   | (fixture; eval only)         |

`--terrain-feature` is an `eval_policy.py` flag only.
Training with `--randomize` draws the feature per episode, and `bench_terrain.py` uses the fixed `TERRAIN_FEATURE` of `src/sim/rollout.py`.

`--terrain-kind mixed` samples a kind per episode, which is the rough-terrain training recipe:

```sh
uv run python train_mj.py --control-mode residual_gait --randomize --curriculum \
    --terrain 0.09 --terrain-adaptive --terrain-kind mixed \
    --obs-history 4 --contact-obs \
    --resample-steps 300 --zero-final --init-std 0.3 --target-kl 0.02 \
    --timesteps 12_000_000 --num-envs 12 --tag rough_contact
```

- `--obs-history N` stacks N sensor frames (gravity + gyro [+ contacts]) spaced 3 control steps
  apart (~180 ms at N=4). A single IMU frame cannot distinguish a slope from a bump mid-stance;
  the recent trajectory can, which is what makes blind rough-terrain walking work.
- `--terrain-adaptive` promotes/demotes roughness by measured tracking quality instead of ramping
  on a fixed schedule (`--terrain-curriculum`). A fixed ramp to a chosen maximum can spend its
  final stretch on ground that is physically impassable, which teaches stalling, not skill.
- `--contact-obs` adds 6 per-foot contact sensors (force-thresholded, with dropouts, false closes
  and dead-sensor faults under DR). This is a **hardware** question — foot switches or FSRs the
  robot does not have yet — so it is measured as an ablation, not assumed.
- Sampled command speeds shrink as roughness rises (`TERRAIN_SPEED_*`): commanding a speed the
  robot cannot make good parks the tracking reward in its flat tail and teaches thrashing.
- The leg links and chassis collide with the ground, so obstacles are solid against shins and belly
  (self-collision stays off). Before that they were phantom and "obstacle clearance" was fiction.

### What limits obstacle height

> **Corrected 2026-08:** the 54 mm figure below is wrong as an absolute cap. Foot lift is limited by
> the femur's ±90° range and that limit moves with ride height, as described — but measured against
> `jnt_range`, 62 mm is fully reachable at nominal stance and 68 mm needs ~+19 mm of body raise.
> `PG_HEIGHT`'s 80 mm ceiling is partly fictional: even with the body fully raised, 3.6 % of the
> commanded joint angles fall outside the servo range. Several searched gaits rely on those clamped
> commands (`ik_feasibility.py` audits this). See `docs/terrain-locomotion.md`.


Foot lift is capped by the femur servo reaching 90° from centre, and that cap **moves with body ride
height**: 54 mm of lift at the nominal 66 mm stance, 78 mm with the body raised 25 mm, only 28 mm
crouched. So clearing a tall step requires standing tall first — a coupling the policy has to learn,
since it controls both (`step_height` and `body_zm` actions). `PG_HEIGHT` therefore spans 10–80 mm,
and `GAIT_DELTA_GAIN` gives the step-height channel extra gain because the analytic base parks that
parameter near the bottom of the range (a unit delta could otherwise never reach the top).

**Judge rough terrain by progress, not by falls.** A statically stable hexapod almost never tips
over; it high-centers and stalls. `bench_terrain.py` therefore reports the fraction of commanded
speed actually made good, the fraction of time stalled, body-rate RMS, and shin/belly knock force,
over a paired matrix of terrain kind × height × command × seed (identical terrain per arm).

### Measured (2026-07-25, 12M-step runs, 6 seeds × 4 kinds × 3 commands)

> **Superseded as a policy-vs-gait comparison** — see `docs/terrain-locomotion.md`.
> The "analytic gait" row below is the *firmware* gait, which turns out to be far from the best
> open-loop gait this robot can walk. A CMA-ES search over stride, cadence, foot lift, duty factor,
> leg phasing and ride height (`optimize_gait_terrain.py`) finds an open-loop gait that beats both
> policies at every roughness level. The rows below are still correct about what they measured; they
> just measured against a weak opponent.

Progress at 0 / 40 / 80 / 120 mm roughness — 0 falls in all 1224 episodes, for every arm:

| arm | flat | 40 mm | 80 mm | 120 mm | curb climbed |
| --- | ---- | ----- | ----- | ------ | ------------ |
| analytic gait      | 0.59 | 0.32 | 0.13 | 0.07 | 30 mm |
| `rough_nocontact`  | 0.76 | 0.74 | 0.65 | 0.45 | **80 mm** |
| `rough_contact`    | 0.74 | 0.74 | 0.65 | 0.48 | **80 mm** |

**Foot contact sensors are near-worthless as a policy input, but worth the hardware as gait
reflexes.** — *both halves of this were re-measured in 2026-08 against a properly tuned gait and
both changed; see `docs/terrain-locomotion.md`. Reflexes help the mistuned gait below (+0.078) but
are net NEGATIVE on every well-tuned arm (−0.012 to −0.059, knock ×8), retaining value only at
≥120 mm roughness and at an arm's curb ceiling. Contacts as an observation are now a small but
significant WIN (+0.015 on held-out commands, p<0.0001; yaw error −0.011 rad/s).*

As an observation (paired over 234 episodes, Wilcoxon): progress +0.009 (p=0.36, no
effect); only body tilt −0.81° and yaw error −0.019 rad/s (both p<0.0001) improve, and stall time
gets *worse* (+0.013, p=0.0001). The same sensors driving reflexes inside the gait engine
(`ReflexConfig`, `HexapodMjEnv(reflex=True)`, `bench_terrain.py --reflex`) do far more:

| paired delta | flat | 40 mm | 80 mm | curb ceiling |
| ------------ | ---- | ----- | ----- | ------------ |
| classical gait + reflexes | 0.59 → 0.59 | 0.32 → **0.41** | 0.13 → **0.24** | 30 → **40 mm** |
| trained policy + reflexes | +0.028 (ns) | **+0.029** (p=0.003) | **+0.071** (p<0.001) | 80 → **100 mm** |

The reflex is reach-until-loaded plus ground-follow — a bang-bang regulator (reach down while a foot
is late, back off while loaded), gated to early stance after a grace period, with an explicit sensor
latency and asymmetric debounce. Three things that do *not* work and are off by default: braking the
shared clock when few feet are loaded (a stance foot reads open ~⅓ of the time on this robot, so it
misfires and was the entire flat-ground regression); holding a leg's phase until touchdown (fires
every swing, since the planned trajectory only kisses the ground); and the elevator reflex (a switch
under the foot cannot sense an obstacle in front of the shin — that needs a shin bumper).

Known open defect: `gait_coef.json`'s `pr_yaw = 0.109` under-cadences turns, so turn-rate tracking
misses its gate on rough ground. The firmware boosts turn cadence up to 1.5× (`gait.h`); the sim
should too (≈1.5), which needs a retrain. (`optimize_gait_terrain.py` searches `pr_yaw`, and the
gait it returns uses 0.547.)

### Searching the gait itself

```sh
# best single open-loop gait for the whole terrain suite -- the baseline a policy must beat
uv run python optimize_gait_terrain.py --mode single --budget 480 --workers 14
# per-terrain gaits (oracle upper bound) and the curb-climb fixture
uv run python optimize_gait_terrain.py --mode per-terrain --merge --workers 14
uv run python optimize_gait_terrain.py --mode curb --merge --workers 14
# compare policies against them
uv run python bench_terrain.py --runs <a> <b> --baseline \
    --gait-library src/resources/gait_library.json --workers 14
# train on top of a searched gait (zero action = that gait)
uv run python train_mj.py --control-mode residual_sched --gait-from src/resources/gait_library.json \
    --gait-key __single__ --zero-final ...
```

The search space (`src/robot/gait_schedule.py`) adds what the analytic map could not express: duty
factor, leg phase pattern (the metachronal family containing tripod/bipod/wave/ripple) and body ride
height. Full methodology, protocol and results: **`docs/terrain-locomotion.md`**.

## Control modes

The command is a body-frame velocity vector `[vx, vy]` (m/s) + yaw rate (rad/s).

- **`residual_sched`** — `residual_gait` plus authority over the coordination pattern itself: duty
  factor and the metachronal leg phasing (slew-limited, since re-timing a leg cannot be a step
  change). `residual_gait` can only interpolate tripod(duty 0.52)→bipod(0.35), so it cannot reach
  the duty ≈ 0.63–0.73 the gait search finds optimal on rough ground. Zero action = the base gait.
- **`residual_pure`** (deploy target) — gait settings come from the analytic command→gait map
  (`analytic_gait_action`, coefficients in `resources/gait_coef.json`, tuned by `optimize_gait.py`);
  the policy outputs only 6×3 foot residuals (±20 mm) on top.
  Zero action reproduces the classical gait exactly, so train with `--zero-final --init-std 0.3`
  to start at the analytic gait and learn pure stabilization.
- **`phase_gait`** — policy outputs gait settings `[step_x, step_y, step_angle, step_height,
  gait_blend, phase_rate]` and drives the firmware gait engine's phase → IK. Low-dim, constrained.
- **`residual`** — `phase_gait` settings + 6×3 foot residuals (policy owns both).
- **`foot`** — policy outputs 6×3 foot-position offsets → IK. Full per-leg authority; learns the gait.

## Layout

- `src/robot/firmware_gait.py` — NumPy port of the firmware gait + IK (the deploy target).
- `src/resources/build_model.py` → `model.xml` — MuJoCo model (0.68 kg; legs 0.077 kg each).
- `src/sim/mj_runtime.py` — MuJoCo runtime wrapper.
- `src/sim/domain_rand.py` - domain randomization (mass/friction/latency/IMU/pushes); servo strength is randomized by the env's `set_servo_scale`.
- `src/sim/terrain.py` — per-episode heightfield generation (bumps/rocks/steps/waves + the `curb`
  measurement fixture). The policy has no exteroception, so terrain skill comes from the IMU,
  the sensor history and (optionally) foot contacts — never from a height map.
- `src/envs/hexapod_mj_env.py` — Gymnasium env (hardware-only observation).
- `train_mj.py` / `eval_policy.py` / `replay_gait.py` — train / eval / classical replay.
- `bench_terrain.py` — rough-terrain / obstacle benchmark (progress, stall fraction, body tilt, knock
  force, gait adaptation) over a paired terrain matrix, plus the `--curb` climb sweep.
- `src/robot/gait_schedule.py` — the open-loop gait as an 18-parameter searchable vector
  (stride/cadence/foot trajectory/posture + duty and leg phasing).
- `src/sim/rollout.py` — the evaluation episode, terrain suite and scalar score, shared by
  `bench_terrain.py` and `optimize_gait_terrain.py` so both measure the same thing.
- `optimize_gait_terrain.py` — CMA-ES / TPE / DE search for the best gait per terrain, the best
  single gait, and the best curb-climbing gait → `src/resources/gait_library.json`.
- `optimize_gait.py` / `export_policy.py` — tune the analytic gait map / export the actor to C++.
- `export_gait.py` - write a `gait_library.json` entry as `firmware/include/gait_tuned.h`.
- `test_firmware_gait_parity.py` - pytest module (and CLI) checking `gait_tuned.h` against the library and `firmware_gait.py` against the firmware headers.
- `ik_feasibility.py` - fraction of a gait's commanded joint angles outside the servo range.
- `servo_feasibility.py` / `servo_margin.py` - whether the real servos can run a gait, and how much torque margin it leaves.
- `reward_alignment.py` / `reward_resolution.py` - whether the training reward ranks gaits like the benchmark, globally and near the optimum.
- `gait_sensitivity.py` - one-parameter-at-a-time sensitivity of a searched gait.
- `arc_stance_test.py` - arc stance versus the straight chord during turns.
- `check_self_level.py` - whether the firmware's self-levelling term reduces body tilt.
- `train_jump.py` / `eval_jump.py` - sim-only jump policies (vertical hop, directional charge).
- `sim_gui.py` - one window driving walk and jump policies live, optionally from the handheld controller.
- `sim_sandbox.py` - manual kinematics/gait sandbox and speed/stability tester, no RL.
- `controller_bridge.py` - reads the ESP-NOW handheld controller over USB for the sim.
- `lidar_slam.py` - standalone 2D LiDAR SLAM for the LD500 scanner, unrelated to the gait stack.

## Animations

`animations/*.json` at the repository root are the bundled animations; the schema is `platform_shared/animation.proto`.
`src/robot/animation.py` is the reference evaluator and player that the firmware and app ports mirror.
`uv run python check_animation.py` runs every animation through the servo model and reports clamped joints, peak joint speed, tilt, and falls.
`uv run python gen_animation_fixtures.py` regenerates `animations/fixtures/expected.json`; `uv run pytest` fails while it is stale.
`uv run python sim_sandbox.py` has an Animate mode for playing and scrubbing an animation.
`uv run python scripts/compile_protos.py` generates the gitignored `src/platform_shared/animation_pb2.py`; it runs automatically on first import, and can be run by hand after editing the proto.
