"""One evaluation episode, the terrain suite, and the scalar score -- shared by the benchmark and
the gait search.

Both live here on purpose. If the per-terrain gait search optimized a different quantity from the
one the benchmark reports, the "our policy beats the best tuned gait" claim would be comparing a
tuned-for-A arm against a judged-on-B metric. Same rollout code, same score, same terrain seeds.
"""

from __future__ import annotations

import numpy as np

from src.envs.hexapod_mj_env import HexapodMjEnv
from src.sim.mj_runtime import CONTROL_DT

# (label, (vx, vy, yaw)); forward dominates -- that is where terrain actually bites
COMMANDS = [
    ("fwd_slow", (0.15, 0.0, 0.0)),
    ("fwd_fast", (0.30, 0.0, 0.0)),
    ("turn", (0.10, 0.0, 0.6)),
]

# A policy trains on the full omnidirectional command distribution; a gait searched over COMMANDS
# alone is tuned for three forward/turn commands and nothing else. Benchmarking both on COMMANDS is
# therefore biased toward the gait, and MEASURABLY so: on the commands below the searched gait's
# yaw error is 0.108-0.123 rad/s against a policy's 0.026-0.049, and it loses.
#
# So the search gets a command set spanning the same envelope the policy trains on (COMMANDS_WIDE),
# and evaluation happens on a THIRD, disjoint set (COMMANDS_HELDOUT) that neither has seen at these
# values. Anything else compares a specialist against a generalist and calls the difference skill.
COMMANDS_WIDE = COMMANDS + [
    ("back", (-0.15, 0.0, 0.0)),
    ("strafe", (0.10, 0.10, 0.0)),
    ("turn_hard", (0.05, 0.0, -1.0)),
    ("fwd_max", (0.42, 0.0, 0.0)),
]

COMMANDS_HELDOUT = [
    ("fwd_mid", (0.22, 0.0, 0.0)),
    ("back_fast", (-0.20, 0.0, 0.0)),
    ("strafe_rev", (0.15, -0.08, 0.0)),
    ("turn_mid", (0.12, 0.0, 0.85)),
    ("spin", (0.0, 0.0, -0.5)),
]

COMMAND_SETS = {"bench": COMMANDS, "wide": COMMANDS_WIDE, "heldout": COMMANDS_HELDOUT}

# Terrain suite: (name, kind, max height m). 'flat' is the plane, and at height 0 the kind is
# meaningless, so it appears once.
TERRAINS = [("flat", "bumps", 0.00)] + [
    (f"{k}_{int(h*1000)}", k, h)
    for k in ("bumps", "rocks", "steps", "waves")
    for h in (0.04, 0.08, 0.12)
]
TERRAIN_BY_NAME = {n: (k, h) for n, k, h in TERRAINS}

# Curb fixture: a single full-width step, swept by height. Judged on forward reach, not tracking.
CURB_HEIGHTS = (0.04, 0.06, 0.08, 0.10, 0.12, 0.14, 0.16, 0.18, 0.20)
CURB_CMD = (0.20, 0.0, 0.0)
CURB_SECONDS = 14.0
CURB_CLEARED = 0.15   # m past the edge that counts as "climbed it and kept going"

STUCK_FRAC = 0.3      # instantaneous progress below this fraction of the command counts as stalled
EPISODE_S = 10.0
TERRAIN_FEATURE = 2.0

# --- score weights (documented in docs/terrain-locomotion.md) ---
# Tracking kernels match the training reward (VEL_SIGMA / YAW_SIGMA) so a gait tuned here is tuned
# for the same notion of "walks as commanded" the policy is trained for.
SCORE_VEL_SIGMA = 0.04
SCORE_YAW_SIGMA = 0.08
from src.sim.rollout_weights import SCORE_W_STUCK, SCORE_W_KNOCK  # noqa: E402
# Body-rate RMS is NOT priced by default. It is the one axis on which the learned policies beat the
# searched open-loop gait, so weighting it is a deliberate choice about what "walks well" means,
# not a default -- and any search told to care about it must be re-run, not re-scored.
SCORE_W_RATE = 0.0
DEFAULT_WEIGHTS = {"stuck": SCORE_W_STUCK, "knock": SCORE_W_KNOCK, "rate": SCORE_W_RATE}


def score(m: dict, weights: dict | None = None) -> float:
    """Scalar objective for a rough-terrain episode, in roughly [0, 1].

    alive * (tracking - stalling - plowing):
      alive    fraction of the episode survived -- a fall truncates and is paid for in proportion
      track    Gaussian kernels on the episode-mean velocity and yaw-rate error (not on progress:
               progress rewards overshooting a slow command, which is a tracking failure)
      stuck    fraction of the episode spent below 30 % of the commanded speed -- high-centering,
               which is how a statically stable hexapod actually fails
      knock    shin/belly contact force as a fraction of body weight -- the cost of plowing
               through an obstacle instead of stepping over it
      rate     roll/pitch angular-rate RMS -- how violent the ride is (off by default)
    """
    w = DEFAULT_WEIGHTS if weights is None else {**DEFAULT_WEIGHTS, **weights}
    track = (np.exp(-m["vel_err"] ** 2 / SCORE_VEL_SIGMA)
             * np.exp(-m["yaw_err"] ** 2 / SCORE_YAW_SIGMA))
    return float(m["alive"] * (track - w["stuck"] * m["stuck"] - w["knock"] * m["knock"]
                               - w["rate"] * m["tilt_rate"]))


def curb_score(m: dict, weights: dict | None = None) -> float:
    """Objective for the curb fixture: how far past the edge it got, capped at 'cleared', minus the
    cost of getting there.

    The cap alone saturates -- every gait that clears the step scores exactly 1.0, so the search
    has no gradient left and returns whichever candidate cleared first. Subtracting the knock term
    restores a preference among clearing gaits for the one that steps over rather than scrambles,
    and keeps the fixture consistent with `score`.
    """
    from src.sim.terrain import CURB_AT
    w = DEFAULT_WEIGHTS if weights is None else {**DEFAULT_WEIGHTS, **weights}
    reach = float(np.clip(m["dy"] / (CURB_AT + CURB_CLEARED), 0.0, 1.0))
    return reach - w["knock"] * m["knock"] - w["rate"] * m["tilt_rate"]


def episode(predict, cfg, kind, height, cmd, seed, randomize, steps, reflex=False,
            gait_schedule=None, weights=None, arc_stance=False):
    """Roll one episode and return its metrics.

    `predict` maps observation -> action (a zero function gives the pure open-loop gait);
    `gait_schedule` replaces the analytic command->gait map for that open-loop base.
    """
    env = HexapodMjEnv(cfg["control_mode"], randomize=randomize, seed=seed,
                       terrain=height, terrain_kind=kind, terrain_feature=TERRAIN_FEATURE,
                       obs_contact=cfg["obs_contact"], obs_history=cfg["obs_history"],
                       reflex=reflex, gait_schedule=gait_schedule, arc_stance=arc_stance,
                       episode_seconds=steps * CONTROL_DT)
    env.fixed_command = np.asarray(cmd, dtype=np.float32)
    o, _ = env.reset()
    d = env.sim.data
    p0 = d.qpos[0:2].copy()
    fwd, lat, yawrate, tilt, knock, fell = [], [], [], [], [], False
    attitude = []   # deg off vertical: the direct "is the body staying level" readout
    gait = {"step_h": [], "cadence": [], "duty": [], "body_zm": [], "gait_blend": []}
    for _ in range(steps):
        o, _, term, _, info = env.step(predict(o))
        fwd.append(info["bvx"])
        lat.append(info["bvy"])
        yawrate.append(float(d.qvel[5]))
        tilt.append(float(np.hypot(*env.sim.gyro()[:2])))
        attitude.append(env.sim.body_tilt_deg())
        knock.append(info["knock"])
        for k in gait:
            gait[k].append(info[k])
        if term:
            fell = True
            break
    n = max(1, len(fwd))
    # progress = mean speed achieved along the commanded body axis / commanded speed. Body-frame
    # velocity already accounts for heading, so a robot that walks in a circle scores its arc.
    prog = float(np.mean(fwd) / cmd[0]) if abs(cmd[0]) > 1e-6 else 1.0
    yaw_prog = float(np.mean(yawrate) / cmd[2]) if abs(cmd[2]) > 1e-6 else 1.0
    inst = np.asarray(fwd) / cmd[0] if abs(cmd[0]) > 1e-6 else np.ones(n)
    m = {
        "progress": max(0.0, min(prog, 2.0)),
        "yaw_progress": max(0.0, min(yaw_prog, 2.0)),
        "vel_err": float(np.hypot(np.mean(fwd) - cmd[0], np.mean(lat) - cmd[1])),
        "yaw_err": float(abs(np.mean(yawrate) - cmd[2])),
        "stuck": float(np.mean(inst < STUCK_FRAC)),
        "tilt_rate": float(np.mean(tilt)),
        "tilt_deg": float(np.mean(attitude)),
        "tilt_max": float(np.max(attitude)) if attitude else 0.0,
        "knock": float(np.mean(knock)),
        "fell": float(fell),
        "alive": len(fwd) / float(steps),
        "dy": float(d.qpos[1] - p0[1]),  # forward displacement (+Y); the curb test's pass criterion
        **{k: float(np.mean(v)) for k, v in gait.items()},
    }
    m["score"] = score(m, weights)
    return m


ZERO_CFG = {"control_mode": "residual_gait", "obs_contact": False, "obs_history": 1}


def zero_predict(act_dim=24):
    z = np.zeros(act_dim, dtype=np.float32)
    return lambda _o: z
