"""Evaluate / watch a trained hexapod policy. Loads the model ONCE (single process).

  python eval_policy.py --run phase_gait_dr_v1_forward            # viewer, command tour
  python eval_policy.py --run phase_gait_dr_v1_forward --headless # per-direction metrics
  python eval_policy.py --run <name> --cmd 0.15 0 0               # hold one command
  python eval_policy.py --run <name> --video walk.mp4             # save the tour
  python eval_policy.py --zero-action --control-mode residual_pure --headless
                                       # analytic-gait baseline in the identical harness
  python eval_policy.py --run <name> --headless --randomize --seeds 10   # robustness gate
  python eval_policy.py --run <name> --headless --push 5 --seeds 10      # push stress gate

Loads the model once and drives a fixed command schedule in one continuous episode.
"""

import argparse
import math
import os
import time
import numpy as np
import mujoco

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from src.envs.hexapod_mj_env import ACT_DIM, HexapodMjEnv, make_env
from src.sim.mj_runtime import CONTROL_DT

# acceptance gates defining "stable"
GATE_VEL_ERR = 0.04   # m/s, per-command mean velocity tracking error
GATE_YAW_ERR = 0.15   # rad/s, per-command mean yaw-rate tracking error
GATE_RES_MEAN = 0.5   # mean |action|: residuals stay corrective, not gait-replacing

# (label, (vx, vy, yaw), seconds)
DEFAULT_TOUR = [
    ("forward", (0.15, 0.0, 0.0), 4),
    ("turn left", (0.0, 0.0, 0.4), 3),
    ("forward", (0.15, 0.0, 0.0), 4),
    ("turn right", (0.0, 0.0, -0.4), 3),
    ("strafe left", (0.0, 0.10, 0.0), 4),
    ("strafe right", (0.0, -0.10, 0.0), 4),
    ("backward", (-0.10, 0.0, 0.0), 4),
    ("stop", (0.0, 0.0, 0.0), 2),
]


class _ZeroPolicy:
    """Analytic-gait baseline: the identical eval harness with zero residuals/params."""

    def __init__(self, act_dim):
        self._a = np.zeros(act_dim, dtype=np.float32)

    def predict(self, obs, deterministic=True):
        return self._a, None


class _Pusher:
    """Fixed-magnitude horizontal pushes on a fixed schedule (headless stress test).

    Pushes `force` N for 5 control steps (0.1 s) every 2.5 s in a seeded random
    horizontal direction, starting 1 s in. With --randomize, drives the
    DomainRandomizer's push fields so this schedule replaces the random 3 N one.
    """

    START, PERIOD, DUR = 50, 125, 5

    def __init__(self, env, force, seed):
        self.env, self.force = env, force
        self.rng = np.random.default_rng(seed)
        self.base_id = mujoco.mj_name2id(env.sim.model, mujoco.mjtObj.mjOBJ_BODY, "base")
        self.vec = np.zeros(3)

    def apply(self, step):
        phase = step - self.START
        if phase < 0:
            return
        if phase % self.PERIOD == 0:
            ang = self.rng.uniform(0.0, 2.0 * np.pi)
            self.vec = self.force * np.array([np.cos(ang), np.sin(ang), 0.0])
        active = phase % self.PERIOD < self.DUR
        dr = self.env.dr
        if dr is not None:
            dr.next_push = 10**9  # disable the random schedule (re-armed on every reset)
            dr.push_force = self.vec if active else np.zeros(3)
            dr.push_steps_left = 1 if active else 0
        else:
            self.env.sim.data.xfrc_applied[self.base_id, :3] = self.vec if active else 0.0


def load(args):
    if args.zero_action:
        return _ZeroPolicy(ACT_DIM[args.control_mode]), lambda o: o
    rundir = os.path.join(args.logdir, args.run)
    model_path = args.model or os.path.join(rundir, "final_model.zip")
    vn_path = args.vecnorm or os.path.join(rundir, "vecnormalize.pkl")
    # custom_objects avoids the FloatSchedule version-mismatch warning on load
    model = PPO.load(
        model_path,
        device="cpu",
        custom_objects={"lr_schedule": lambda _: 0.0, "clip_range": lambda _: 0.2},
    )
    vn = VecNormalize.load(vn_path, DummyVecEnv([make_env(args.control_mode)]))
    mean, var, clip, eps = vn.obs_rms.mean, vn.obs_rms.var, vn.clip_obs, vn.epsilon
    norm = lambda o: np.clip((o - mean) / np.sqrt(var + eps), -clip, clip).astype(
        np.float32
    )
    return model, norm


def schedule(args):
    if args.cmd is not None:
        return [("fixed", tuple(args.cmd), int(args.seconds))]
    return DEFAULT_TOUR


def run_headless(args):
    model, norm = load(args)
    rows = []  # (label, vel_err, yaw_err, fell, mean|action|)
    for i in range(args.seeds):
        seed = args.seed + i
        env = HexapodMjEnv(args.control_mode, randomize=args.randomize, seed=seed, terrain=args.terrain,
                           terrain_kind=args.terrain_kind, terrain_feature=args.terrain_feature)
        pusher = _Pusher(env, args.push, seed) if args.push > 0 else None
        for label, cmd, dur in schedule(args):
            env.fixed_command = np.asarray(cmd, dtype=np.float32)
            o, _ = env.reset()
            x0, y0 = env.sim.data.qpos[0], env.sim.data.qpos[1]
            bv, yaw_rate, acts, fell = [], [], [], False
            for step in range(int(dur / CONTROL_DT)):
                if pusher:
                    pusher.apply(step)
                a, _ = model.predict(norm(o), deterministic=True)
                acts.append(np.mean(np.abs(a)))
                o, _, t, _, info = env.step(a)
                bv.append([info["bvx"], info["bvy"]])
                yaw_rate.append(env.sim.data.qvel[5])  # body yaw rate (rad/s)
                if t:
                    fell = True
                    break
            bv = np.mean(bv, axis=0)
            vel_err = float(np.hypot(bv[0] - cmd[0], bv[1] - cmd[1]))
            yaw_err = float(abs(np.mean(yaw_rate) - cmd[2]))
            rows.append((label, vel_err, yaw_err, fell, float(np.mean(acts))))
            if args.seeds == 1:
                dist = np.hypot(env.sim.data.qpos[0] - x0, env.sim.data.qpos[1] - y0) * 1000
                print(
                    f"{label:12s} cmd(vx,vy,yaw)={cmd}  bvx={bv[0]:+.3f} bvy={bv[1]:+.3f} "
                    f"yaw={np.mean(yaw_rate):+.3f}  dist={dist:4.0f}mm"
                    + ("  FELL" if fell else "")
                )
    _print_gates(rows, args)


def _print_gates(rows, args):
    labels = list(dict.fromkeys(r[0] for r in rows))
    by_label = {lb: [r for r in rows if r[0] == lb] for lb in labels}
    if args.seeds > 1:
        print(f"per-command means over {args.seeds} seeds:")
        for lb, rs in by_label.items():
            print(
                f"{lb:12s} vel_err={np.mean([r[1] for r in rs]):.3f}  "
                f"yaw_err={np.mean([r[2] for r in rs]):.3f}  "
                f"falls={sum(r[3] for r in rs)}/{len(rs)}"
            )
    falls = sum(r[3] for r in rows)
    track_ok = all(
        np.mean([r[1] for r in rs]) <= GATE_VEL_ERR and np.mean([r[2] for r in rs]) <= GATE_YAW_ERR
        for rs in by_label.values()
    )
    res_mean = float(np.mean([r[4] for r in rows]))
    ok = lambda b: "PASS" if b else "FAIL"
    print(
        f"\nGATES  falls={falls}/{len(rows)} [{ok(falls == 0)}]  "
        f"tracking(vel<={GATE_VEL_ERR}, yaw<={GATE_YAW_ERR}) [{ok(track_ok)}]  "
        f"mean|action|={res_mean:.3f}(<{GATE_RES_MEAN}) [{ok(res_mean < GATE_RES_MEAN)}]"
    )
    print(f"OVERALL: {ok(falls == 0 and track_ok and res_mean < GATE_RES_MEAN)}")


def _drive(model, norm, env, sched, render_cb, loop=True):
    """Run the command tour; call render_cb() each control step. Resets on fall."""
    o, _ = env.reset()
    while True:
        for label, cmd, dur in sched:
            print(f"  command: {label} {cmd}")
            env.cmd[:] = cmd
            for _ in range(int(dur / CONTROL_DT)):
                a, _ = model.predict(norm(o), deterministic=True)
                o, _, t, _, _ = env.step(a)
                if not render_cb():
                    return
                if t:  # fell: reset, keep touring
                    o, _ = env.reset()
        if not loop:
            return


def _draw_arrows(viewer, env, cmd, avx, avy):
    """Red arrow = commanded velocity direction; green = (smoothed) actual velocity, world frame."""
    d = env.sim.data
    base = d.qpos[0:3].copy()
    base[2] += 0.12
    q = d.qpos[3:7]
    yaw = math.atan2(
        2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3])
    )
    c, s = math.cos(yaw), math.sin(yaw)
    cwx, cwy = c * cmd[0] - s * cmd[1], s * cmd[0] + c * cmd[1]  # body cmd -> world
    scn = viewer.user_scn
    scn.ngeom = 0

    def arrow(frm, vx, vy, rgba):
        spd = math.hypot(vx, vy)
        if spd < 1e-3 or scn.ngeom >= scn.maxgeom:
            return
        length = min(max(spd * 1.2, 0.08), 0.40)
        to = np.array([frm[0] + vx / spd * length, frm[1] + vy / spd * length, frm[2]])
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(
            g,
            mujoco.mjtGeom.mjGEOM_ARROW,
            np.zeros(3),
            np.zeros(3),
            np.eye(3).flatten(),
            np.array(rgba, np.float32),
        )
        mujoco.mjv_connector(
            g, int(mujoco.mjtGeom.mjGEOM_ARROW), 0.006, np.array(frm, float), to
        )
        scn.ngeom += 1

    arrow(base, cwx, cwy, [1.0, 0.3, 0.3, 1.0])
    arrow(
        base - np.array([0, 0, 0.02]), avx, avy, [0.3, 1.0, 0.3, 1.0]
    )


def run_viewer(args):
    """Interactive viewer: drive the robot live with the keyboard; arrows show command vs actual."""
    import mujoco.viewer

    if args.tour or args.cmd is not None:
        return run_tour_viewer(args)
    model, norm = load(args)
    env = HexapodMjEnv(args.control_mode, randomize=args.randomize, seed=args.seed, terrain=args.terrain)
    o, _ = env.reset()
    cmd = [0.0, 0.0, 0.0]
    DV = (0.03, 0.03, 0.1)
    LIM = ((-0.12, 0.12), (-0.20, 0.20), (-0.5, 0.5))
    clamp = lambda v, lim: max(lim[0], min(lim[1], v))

    def key_cb(key):
        if key in (265, 87):
            cmd[1] = clamp(cmd[1] + DV[1], LIM[1])  # Left / A
        elif key in (264, 83):
            cmd[1] = clamp(cmd[1] - DV[1], LIM[1])  # Right / D
        elif key in (263, 65):
            cmd[0] = clamp(cmd[0] + DV[0], LIM[0])  # Up / W
        elif key in (262, 68):
            cmd[0] = clamp(cmd[0] - DV[0], LIM[0])  # Down / S
        elif key == 81:
            cmd[2] = clamp(cmd[2] + DV[2], LIM[2])  # Q
        elif key == 69:
            cmd[2] = clamp(cmd[2] - DV[2], LIM[2])  # E
        elif key == 88:
            cmd[0] = cmd[1] = cmd[2] = 0.0  # X
        print(f"cmd  vx={cmd[0]:+.2f}  vy={cmd[1]:+.2f}  yaw={cmd[2]:+.2f}")

    print(
        "Controls: Up/Down or W/S = fwd/back | Left/Right or A/D = strafe | Q/E = yaw | X = stop"
    )
    sv = np.zeros(2)  # EMA-smoothed actual velocity for the green arrow
    with mujoco.viewer.launch_passive(
        env.sim.model, env.sim.data, key_callback=key_cb
    ) as viewer:
        while viewer.is_running():
            t0 = time.time()
            env.cmd[:] = cmd
            a, _ = model.predict(norm(o), deterministic=True)
            o, _, term, _, _ = env.step(a)
            if term:
                o, _ = env.reset()
            sv = 0.9 * sv + 0.1 * env.sim.data.qvel[0:2]
            _draw_arrows(viewer, env, cmd, sv[0], sv[1])
            viewer.sync()
            dt = CONTROL_DT - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)


def run_tour_viewer(args):
    import mujoco.viewer

    model, norm = load(args)
    env = HexapodMjEnv(args.control_mode, randomize=args.randomize, seed=args.seed, terrain=args.terrain)
    with mujoco.viewer.launch_passive(env.sim.model, env.sim.data) as viewer:

        def render():
            t0 = time.time()
            viewer.sync()
            dt = CONTROL_DT - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)
            return viewer.is_running()

        _drive(model, norm, env, schedule(args), render, loop=True)


def run_video(args):
    import mediapy, mujoco

    model, norm = load(args)
    env = HexapodMjEnv(args.control_mode, randomize=args.randomize, seed=args.seed, terrain=args.terrain)
    renderer = mujoco.Renderer(env.sim.model, height=480, width=640)
    frames = []

    def render():
        renderer.update_scene(env.sim.data, camera=-1)
        frames.append(renderer.render())
        return len(frames) < int(args.seconds / CONTROL_DT)

    _drive(model, norm, env, schedule(args), render, loop=False)
    mediapy.write_video(args.video, frames, fps=int(1 / CONTROL_DT))
    print(f"wrote {args.video} ({len(frames)} frames)")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--run", default="phase_gait_dr_v1_forward", help="run dir under --logdir"
    )
    ap.add_argument(
        "--control-mode",
        choices=["phase_gait", "foot", "residual", "residual_pure", "residual_gait"],
        default="phase_gait",
    )
    ap.add_argument("--model", default=None)
    ap.add_argument("--vecnorm", default=None)
    ap.add_argument("--logdir", default="./runs")
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument(
        "--cmd", type=float, nargs=3, default=None, metavar=("VX", "VY", "YAW")
    )
    ap.add_argument("--randomize", action="store_true")
    ap.add_argument("--seed", type=int, default=123)
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--zero-action", action="store_true",
                    help="no model: zero actions = the analytic-gait baseline (residual modes)")
    ap.add_argument("--seeds", type=int, default=1, help="headless: repeat the tour over N seeds and aggregate")
    ap.add_argument("--push", type=float, default=0.0,
                    help="headless: horizontal push force (N) on a fixed schedule (stress test)")
    ap.add_argument("--terrain", type=float, default=0.0,
                    help="max bump height (m) of per-episode random heightfield terrain (e.g. 0.02)")
    ap.add_argument("--terrain-kind", default="bumps", choices=["bumps", "rocks", "slope"],
                    help="terrain type for eval (fixed, so gait vs policy see the same ground)")
    ap.add_argument("--terrain-feature", type=float, default=1.0,
                    help="terrain bumpiness/feature (fixed for eval; e.g. 2.5)")
    ap.add_argument(
        "--tour", action="store_true", help="scripted command tour instead of keyboard"
    )
    ap.add_argument("--video", default=None)
    args = ap.parse_args()

    if args.video:
        run_video(args)
    elif args.headless:
        run_headless(args)
    else:
        run_viewer(args)
