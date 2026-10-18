# Hexapod Simulation

MuJoCo simulation + RL training for the hexapod, designed for **sim-to-real** transfer to
the ESP32 robot. The policy observes only what the real robot can sense (IMU + gyro; open-loop
servos have no encoders) and acts through the firmware gait/IK.

## Setup (uv)

```sh
uv sync                 # core MuJoCo + SB3 + torch (cu126) stack, plus pytest (dev group)
```

Run anything with `uv run`, e.g. `uv run python train_mj.py --smoke`.
`uv run pytest -q` runs the sim-to-firmware parity tests (`test_firmware_gait_parity.py`).
`test_gait_trace_parity.py` replays `fixtures/gait_trace.json`, the feet the firmware gait commands through a scripted stand, walk and stop session (every tick run, every third recorded), through `firmware_gait.py` and holds every coordinate within 1e-3 mm.
That bound assumes the float32 firmware and the float64 port drift apart no further than over the fixture's 695 ticks (4.5e-4 mm measured); a law change moves the feet by millimetres.
After a change to `gait.h`, regenerate the fixture from the repository root with `GAIT_TRACE_OUT=simulation/fixtures/gait_trace.json pio test -e native -f test_gait`.

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

The firmware runs the policy exported from the run `residual_gait_fw` with `control_mode=residual_gait` and a four-frame sensor history (75 observations, 24 actions).
The first comment line of `firmware/include/policy/hexapod_policy.h` states this as `// run: <name> mode: <mode> hash: <commit>`, and the firmware build fails if the header's dimensions do not match what `PolicyRunner` assembles and applies.
`residual_gait` lets the policy adjust the gait itself (step, lift, blend, cadence, ride height) on top of 18 foot residuals.
`residual_pure` (18 outputs) keeps the gait fixed to the analytic command-to-gait map and learns only the foot residuals; `PolicyRunner` can drive such a header, but none is deployed.
Both modes train on the robot's own WALK_NN path (`src/sim/firmware_policy.py`): the firmware's `PolicyCore` builds the observation and filters and decodes the action, and its gait and inverse kinematics set the joints, so they refuse what the robot lacks (`--contact-obs`, reflexes, arc stance).

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
    --obs-history 4 \
    --resample-steps 300 --zero-final --init-std 0.3 --target-kl 0.02 \
    --timesteps 12_000_000 --num-envs 12 --tag rough
```

- `--obs-history N` stacks N sensor frames (gravity + gyro [+ contacts]) spaced 3 control steps
  apart (~180 ms at N=4). A single IMU frame cannot distinguish a slope from a bump mid-stance;
  the recent trajectory can, which is what makes blind rough-terrain walking work.
- `--terrain-adaptive` promotes/demotes roughness by measured tracking quality instead of ramping
  on a fixed schedule (`--terrain-curriculum`). A fixed ramp to a chosen maximum can spend its
  final stretch on ground that is physically impassable, which teaches stalling, not skill.
- `--contact-obs` adds 6 per-foot contact sensors (force-thresholded, with dropouts, false closes
  and dead-sensor faults under DR). This is a **hardware** question — foot switches or FSRs the
  robot does not have yet — so it is measured as an ablation, not assumed, in `residual_sched`:
  the robot's WALK_NN path has no contact input.
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
- **`residual_pure`** — gait settings come from the analytic command→gait map
  (`analytic_gait_action`, coefficients in `resources/gait_coef.json`, tuned by `optimize_gait.py`);
  the policy outputs only 6×3 foot residuals (±20 mm) on top.
  Zero action reproduces the classical gait exactly, so train with `--zero-final --init-std 0.3`
  to start at the analytic gait and learn pure stabilization.
- **`phase_gait`** — policy outputs gait settings `[step_x, step_y, step_angle, step_height,
  gait_blend, phase_rate]` and drives the firmware gait engine's phase → IK. Low-dim, constrained.
- **`residual`** — `phase_gait` settings + 6×3 foot residuals (policy owns both).
- **`foot`** — policy outputs 6×3 foot-position offsets → IK. Full per-leg authority; learns the gait.

## Layout

- `src/robot/firmware_gait.py` — NumPy port of the firmware gait + IK, which the research modes run on.
- `src/sim/firmware_policy.py` — the robot's WALK_NN path (`firmware/host/policy_api.cpp`), which `residual_pure` and `residual_gait` train on.
- `src/sim/virtual_robot.py` / `src/sim/firmware_loop.py` — the firmware's own Hexapod (`firmware/host/`) as a native library, compiled on first use with `$CXX` (C++20), driving MuJoCo for replay, the speed model and the `Robot` simulation backend.
- `src/resources/build_model.py` → `model.xml` — MuJoCo model (0.68 kg; legs 0.077 kg each).
- `src/sim/mj_runtime.py` — MuJoCo runtime wrapper.
- `docs/servo-model.md` - the non-back-drivable servo model (stall versus holding torque), its placeholder holding ratio and the bench measurement that replaces it.
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
- `export_envelope.py` - sweep the gait-independent stroke geometry per axis over a 9 x 9 x 9 grid of the lift, body height and feet distance sliders (the longest feasible stroke within joint travel with a 2 degree margin and reach, and the swing and stance joint-rate factors at ten strokes up to it), keep the servo speed (`--servo-rad-s`, default 13.1) and the 3 cycles/s cadence cap beside it, and write `firmware/include/gait_envelope.h` and `src/resources/gait_envelope.json` (generator `src/robot/gait_envelope.py`).
  The firmware's `gait_speed.h` reads the header and computes each gait's top cadence at the stroke the stride slider picks (0 is the speed-optimal stroke, +1 the longest, -1 0.3 of the optimum); regenerating the header changes the speed model, so rerun `measure_speed_model.py` afterwards.
- `test_firmware_gait_parity.py` - pytest module (and CLI) checking `gait_tuned.h` against the library and `firmware_gait.py` against the firmware headers.
- `test_gait_trace_parity.py` - replays the firmware's tick-by-tick gait trace (`fixtures/gait_trace.json`) through `firmware_gait.py`.
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
- `src/hexapod/backends/serial_link.py` - the robot's native USB serial link (COBS frames, request correlation, a wait for the board to answer), shared by `Robot("COM5")` (or `Robot("/dev/ttyACM0")`) and `robot_animate.py`; a USB-tethered run needs no WiFi.
- `calibrate_compass.py` - `calibrate_compass.py recording.hexrec [--write <address>]` fits the compass's hard-iron offset to a `record.py` recording of the robot turned through every orientation (`src/sim/compass_fit.py`).
  The recording holds the readings before the stored offset (`TickSample.mag`), so refitting a calibrated robot gives the same offset, not zero.
  It refuses too little coverage or a field outside Earth's 25-65 uT; `--write` stores `mag_offset_x/y/z` in the peripheral settings and keeps the I2C pins.

## Animations

`animations/*.json` at the repository root are the bundled animations; the schema is `platform_shared/animation.proto`.
`src/robot/animation.py` is the reference evaluator and player that the firmware and app ports mirror.
`uv run python check_animation.py` runs every animation through the servo model and reports clamped joints, peak joint speed, tilt, and falls.
`uv run python gen_animation_fixtures.py` regenerates `animations/fixtures/expected.json`; `uv run pytest` fails while it is stale.
`uv run python sim_sandbox.py` has an Animate mode for playing and scrubbing an animation.
`uv run python scripts/compile_protos.py` generates the gitignored `src/platform_shared/animation_pb2.py`; it runs automatically on first import, and can be run by hand after editing the proto.
`uv run python robot_animate.py --port <COMx> <command>` uploads, validates, plays, stops and watches animations over the robot's native USB port; run `--help` for the commands.

## Python Robot facade

`src/hexapod` drives the simulation and the real robot through one `Robot`:

```python
from src.hexapod import Robot

with Robot("simulation") as robot:   # or Robot("192.168.4.1"), the robot's WebSocket at ws://<host>/api/ws
    robot.stand()
    robot.move_forward(0.3)          # metres, a goal the firmware closes on its own odometry
    robot.rotate(90)                 # degrees counter-clockwise, a goal closed on the gyro-fused heading
```

The API speaks REP-103 (x forward, y left, yaw counter-clockwise); the backends convert to the robot's X-right, Y-forward body frame and to the firmware's sticks.
`Robot("simulation", viewer=True)` opens the MuJoCo viewer and runs in real time; the simulation backend runs `src/sim/firmware_loop.py`, the firmware's STAND/WALK tick that `replay_gait.py` also uses.
The firmware takes sticks, not velocities, so `src/resources/speed_model.json` holds the body velocity each stick produces, measured per gait in the simulation, at stride 0, by `uv run python measure_speed_model.py`; `test_speed_model.py` fails when it no longer matches the simulation.
It is the simulated robot's speed: check a distance on the floor before trusting it on the real one.
A move that is not a goal (TUNED, or a firmware from before goals) is timed from the PC: it holds its command for distance / speed and the gait phase its stop lands on scatters it by about 1 cm, so from a settled stand a move of 0.3 m or more lands within 4 % of the request, except TUNED forward (8 % short at 0.3 m, 5.5 % at 0.5 m).
A move of 0.1 m lands between 19 % short (TUNED forward) and 13 % long (RIPPLE sideways), measured per gait forward and to both sides.
The first move sent the moment `stand()` returns starts about 0.3 s late while the walk takes its posture, so it ends 0.02 to 0.045 m short at any distance.
A per-gait distance offset is future work.
On the real robot the input is re-sent every 200 ms inside the firmware's 600 ms dead-man, and a timed move raises if the link may have stalled past that or someone switched the robot out of WALK.
A mode change returns once the robot reports it applied, so a goal sent right after `stand()` is accepted; during an animation the robot applies it only after the clip's exit, and an exit longer than the 3 s reply timeout raises `RobotTimeout` although the robot still switches.
Leaving the `with` block, also on an exception or Ctrl-C, stops the robot, lowers it onto its belly and switches the servos off.

## Goals

A goal is a move the firmware makes on its own odometry (`docs/firmware.md`, "Odometry and goal moves"), so a WiFi stall cannot cut it short and the PC no longer times it.
A script sets a walking style and then moves; each call blocks until the move ends:

```python
robot.style(gait=Gait.TRI_GATE, speed=0.5, stride=0.0, step_height=20, body_height=0, feet_distance=0)
robot.reset_odometry()               # with the robot still
for _ in range(4):
    robot.walk(1.0)                  # metres; negative is backwards; sideways=True walks to the left (negative right)
    robot.turn(90)                   # degrees, counter-clockwise when positive
robot.arc(0.5, 360)                  # radius in m (> 0) and degrees; a positive angle curves left, a negative one right
robot.state().odometry               # the pose in the frame the robot faced at the reset
```

- `style()` sets the gait, the speed share in (0, 1] of the gait's top speed at the posture, the app's stride and feet-distance sliders in [-1, 1], the step height in mm and the body height in mm.
  An omitted field keeps its value.
- `walk()`, `turn()` and `arc()` take an optional `on_progress` callback, called with each new status.
  They raise `GoalTimedOut` on `TIMED_OUT`, and `RobotError` on `CANCELLED`, on `REFUSED` or when another client's goal replaced theirs; any other exception before the goal is known to have ended, a lost reply (`RobotTimeout`) and Ctrl-C included, cancels the goal as far as the link still allows.
- Units and signs are REP-103 at the facade's edge (x forward, y left, yaw counter-clockwise); the backends convert to the firmware's frame, where x is right.
- A walk holds the heading it starts on, and a turn or an arc is aimed at the heading the script intends: after a walk that drifted 2 deg to the left a 90 deg turn turns 88, so the corners of a square do not accumulate the drift.
  `reset_odometry()` sets the heading the goals then mean; call it with the robot still, as a walk under way would be counted into the new frame.
  The robot refuses it while a goal runs (409, a `RobotError`), as the pose is the goal's origin.
- Goals take TRI, BI, WAVE, RIPPLE and AUTO.
  `move_forward`, `move_sideways` and `rotate` fall back to the timed motions above for TUNED, which has no speed envelope, and for a firmware from before goals, which answers a goal request 400; `walk`, `turn` and `arc` refuse there.
  The speed model converts the speeds those three take (m/s and rad/s) to the share a goal asks for.
- An arc's drift correction changes its length as well as its end heading.
- The keep-alive that holds the link carries the last style's posture and stride (`s1`, `h`, `fd`, `stride`), so the robot keeps it after a goal.
  It therefore also carries a non-default style into later velocity walks, which the speed model (measured at the default posture and stride 0) does not cover.
  The style's speed is not carried: a goal ignores centred commands, so the speed would reach only the velocity walks.

Measured in MuJoCo on its true pose:

- The odometry reads about 5 % off the distance the robot truly walks, by foot slip; there is no calibration, by decision, and the floor tests decide whether one is needed.
- The heading is the DMP's yaw fused with the feet's: it ends within 0.2 deg of the true yaw over a 1 m walk and within 0.4 deg over a 90 deg turn, on TRI and BI.
- The 1 x 1 m square closes within 2.9 to 4.0 cm and 2.6 deg on TRI.
- On BI a 90 deg turn ends 2.6 to 3.2 deg short of the true yaw, and 2.5 to 2.8 deg short on its own odometry.
  The shortfall is the firmware's, not the simulation's: without physics, where the odometry is the truth, the stop prediction ends 90 deg turns between 2.2 deg short and 1.5 deg over (TRI and BI at speeds 0.25 to 1.0), and a turn is accepted within 3 deg.
  The tolerance stays at 3 deg, as a corrective pass of 1 to 3 deg is a tiny goal that can time out (below); the intended heading keeps the shortfall from adding up over a sequence of turns.
- BI's square still ends 0.8 to 11.6 cm and up to 9.8 deg off: each turn's corrective pass starts with a yaw of about 2 deg that the feet do not command, and the fused heading, which trusts the feet while they barely move, misses it.
  Whether the robot's feet slip so at a pass's start is unmeasured.
- Tiny goals, about 10 to 15 mm or a few degrees, at speeds 0.25 and 1.0 can end `TIMED_OUT`: the settled pose falls outside the tolerance and the passes run out.

## Floor benchmark

Three tools check the robot against the floor rather than against itself.

```sh
uv run python ground_truth.py course.json --write-markers markers/    # print the course's ArUco markers to size
uv run python ground_truth.py course.json --camera 0 --out run.csv    # the body's floor track from an overhead camera (or --video run.mp4)
uv run python score_track.py run.hexrec run.csv                       # the recording's motion against that track
uv run python measure_pwm_per_degree.py 192.168.4.1                   # the servos' PWM counts per degree from the IMU's roll (see below)
uv run python measure_pwm_per_degree.py simulation --sim-scale 2.28   # the same on MuJoCo servos of a known scale
```

`ground_truth.py` finds an ArUco marker on the robot's back over four floor markers at known positions, takes the marker's height out of the floor homography, and writes `t,x,y,yaw` in metres and radians in the robot's frame convention (`docs/conventions.md`: yaw 0 faces the floor's +y).
Its docstring describes the course file; on a rendered video the track comes within 1 mm RMS (`test_ground_truth.py`).
`score_track.py` aligns the track's clock with a `record.py` recording on the start motion (open a course with a turn in place from standing), then prints, per estimate, the distance made good, the position and yaw error, the yaw drift per metre walked and the ratio of the paths.
The estimate is the commanded motion, each tick's sticks through the speed model in WALK, so a recording that walked at a nonzero stride (the ninth command entry of a tick) is refused; the firmware's odometry (the ticks' `odom_x`, `odom_y` and `odom_yaw`) is scored beside it when the recording carries it.
`measure_pwm_per_degree.py` sets the nominal stand height, switches the robot to IDLE and steps the six femurs through raw PWMs, left side against right, and fits the counts per degree that make the firmware's leg geometry predict the body roll each step gave; `test_measure_pwm.py` recovers a scale injected into the simulated servos within 2 %.
On the robot it needs the real backend's roll, `servo_calibration` and `set_servo_pwm`, which are being added to `src/hexapod/backends/real.py`; until then it refuses and names what is missing.
The operator stands the robot on a flat, level floor before running it; the script refuses a robot that is not standing still.
