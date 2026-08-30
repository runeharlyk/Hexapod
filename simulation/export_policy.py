"""Export a trained SB3 actor + VecNormalize stats to a self-contained C header.

Produces a header with the MLP weights, the observation-normalization constants, and an
inline `hexapod_policy::infer(obs_raw, act_out)` function (pure C++/math.h, no deps) that
the firmware calls. Float32, hand-rolled — a ~55k-param MLP needs no TFLite/esp-dl/ESP-NN.

The exporter self-checks numpy-vs-SB3 parity before writing (must match to ~1e-6).

  python export_policy.py --run residual_pure_stab          # default mode residual_pure
  python export_policy.py --control-mode phase_gait --model <zip> --vecnorm <pkl>

The header additionally carries the base gait the run was actually trained on (analytic_gait,
either the legacy map from resources/gait_coef.json or the searched schedule recorded in the run
config), the action-decoding constants (PG_*, FOOT_RESIDUAL_MM, CMD_* ranges, coordination
tables), and a golden TEST_OBS/TEST_ACT pair for a boot-time self-check.
"""

import argparse
import os
import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from src.envs.hexapod_mj_env import (
    make_env,
    load_env_config,
    GAIT_COEF,
    FOOT_RESIDUAL,
    BODY_ZM_MM,
    PG_STEP_XY,
    PG_STEP_ANGLE,
    PG_HEIGHT,
    PG_PHASE_RATE,
    CMD_VX,
    CMD_VY,
    CMD_YAW,
    HIST_STRIDE,
    CONTACT_FORCE_THRESH,
    GAIT_DELTA_GAIN,
)
from src.robot.firmware_gait import TRI_OFFSET, TRI_STAND_FRAC, BI_OFFSET, BI_STAND_FRAC
from src.robot.gait_schedule import GaitSchedule
from src.sim.mj_runtime import CONTROL_DT


def extract(model):
    """Return [(W,b), ...] for hidden layers (tanh after each) + final linear layer."""
    layers = []
    for m in model.policy.mlp_extractor.policy_net:
        if isinstance(m, torch.nn.Linear):
            layers.append((m.weight.detach().cpu().numpy(), m.bias.detach().cpu().numpy()))
    layers.append((model.policy.action_net.weight.detach().cpu().numpy(),
                   model.policy.action_net.bias.detach().cpu().numpy()))
    return layers


def np_infer(obs_raw, layers, mean, var, clip, eps, do_clip=True):
    x = (obs_raw - mean) / np.sqrt(var + eps)
    x = np.clip(x, -clip, clip)
    for i, (W, b) in enumerate(layers):
        x = W @ x + b
        if i < len(layers) - 1:
            x = np.tanh(x)
    return np.clip(x, -1.0, 1.0) if do_clip else x


def carr(name, arr):
    vals = ", ".join(f"{v:.8e}f" for v in np.asarray(arr).ravel())
    return f"static const float {name}[{arr.size}] = {{{vals}}};\n"


def obs_layout_comment(obs_history, obs_contact, act_dim):
    """Generate the layout comment from the env's actual configuration -- a hand-written one drifts,
    and a firmware that fills the wrong slot fails silently."""
    frame = 6 + (6 if obs_contact else 0)
    lines = [
        "// Observation layout (OBS_DIM floats, raw units; normalization is inside infer()):",
        f"//   [0:{obs_history * frame}] sensor-frame history: {obs_history} frames of "
        f"{frame} floats, newest first,",
        f"//           consecutive frames {HIST_STRIDE} control steps apart (keep a ring buffer).",
        "//           Each frame is:",
        "//             [0:3] gravity in body frame, world-down convention: level = (0, 0, -1).",
        "//                   NOTE: MPU6050 DMP dmpGetGravity() is world-UP -> negate all 3.",
        "//             [3:6] gyro (rad/s), body frame [wx, wy, wz]",
    ]
    if obs_contact:
        lines.append(f"//             [6:12] per-foot contact, 1.0 = loaded (>{CONTACT_FORCE_THRESH} N), "
                     "0.0 = open;")
        lines.append("//                   REQUIRES foot switches/FSRs. Order = leg 0..5.")
    o = obs_history * frame
    lines += [
        f"//   [{o}:{o+3}] rpy (rad) [roll, pitch, yaw]",
        f"//   [{o+3}:{o+21}] previous commanded joint angles (RADIANS, not servo degrees),",
        "//           order [coxa_i, femur_i, tibia_i for i in 0..5] = IK output order",
        f"//   [{o+21}:{o+23}] gait phase clock [sin(2*pi*phase), cos(2*pi*phase)]",
        f"//   [{o+23}] control dt (s) of THIS loop iteration -- feed the real measured dt, and",
        "//           advance the gait phase by the same dt (the policy was trained across 40-80 Hz)",
        f"//   [{o+24}:{o+27}] command [vx (m/s), vy (m/s), yaw_rate (rad/s)], see CMD_* ranges",
        f"//   [{o+27}:{o+27+act_dim}] previous action (this function's previous act_out)",
    ]
    return "\n".join(lines) + "\n"


class BaseGait:
    """The open-loop gait the exported residuals sit on top of, in the form the header needs.

    A run trained with `--gait-from <library>` sits on a searched `GaitSchedule`, not on the legacy
    analytic map, and baking the wrong one destroys the "zero action = base gait" property the
    firmware relies on. Both sources produce the same six normalized gait actions from the same
    coefficient names, so the header format is identical; only the numbers change. A schedule pins
    the coordination pattern instead of blending tripod->bipod, which is expressed by emitting both
    blend endpoints as the schedule's own offsets and duty.
    """

    def __init__(self, schedule: "GaitSchedule | None"):
        self.schedule = schedule
        if schedule is None:
            self.coef = GAIT_COEF
            self.tri_offset, self.bi_offset = TRI_OFFSET, BI_OFFSET
            self.tri_stand_frac, self.bi_stand_frac = TRI_STAND_FRAC, BI_STAND_FRAC
            self.ride_mm = 0.0
        else:
            offsets = schedule.offsets()
            self.coef = schedule.to_dict()
            self.tri_offset = self.bi_offset = offsets
            self.tri_stand_frac = self.bi_stand_frac = float(schedule.duty)
            self.ride_mm = float(schedule.ride_mm)

    @property
    def name(self):
        return "legacy analytic map (resources/gait_coef.json)" if self.schedule is None \
            else "searched gait schedule from the run config"


ACTION_LAYOUT = {
    "residual_pure": """\
// Action layout (residual_pure, 18): per-leg foot residuals [dx, dy, dz] * FOOT_RESIDUAL_MM,
// added to the feet produced by analytic_gait() -> IK. Zero action = the base gait baked below.
""",
    "residual_gait": """\
// Action layout (residual_gait, 24): deltas ADDED to analytic_gait()'s normalized params (each
// scaled by GAIT_DELTA_GAIN first), then clipped to [-1,1], plus foot residuals. Zero action = the
// base gait baked below, so a firmware that applies only part of this degrades gracefully -- but it
// will NOT reproduce the trained policy.
//   a[0] += step_x   (body-X, LATERAL stride)      a[3] += gait_blend (tripod 0 <-> bipod 1)
//   a[1] += step_y   (body-Y, FORWARD stride)      a[4] += phase_rate (cadence)
//   a[2] += step_height                            a[5]  -> body.zm = -a[5] * BODY_ZM_MM
//   a[6:24] per-leg foot residuals [dx, dy, dz] * FOOT_RESIDUAL_MM
""",
}


def analytic_gait_cpp(base):
    """C++ port of the run's command->gait map with its coefficients baked in."""
    c = base.coef
    return f"""\
// Analytic command -> 6 gait-param actions in [-1,1]^6 (residual_pure runs on top of this).
// Port of hexapod_mj_env.analytic_gait_action / GaitSchedule.gait_action; coefficients come from
// the {base.name} and are baked in here so firmware and sim cannot diverge.
// cmd = [vx (m/s) FORWARD, vy (m/s) LATERAL, yaw_rate (rad/s)]; out = [step_x, step_y, step_angle,
// step_height, gait_blend, phase_rate] as normalized actions (decode with PG_* constants).
// The robot FACES +Y: forward vx drives step_y (body-Y) and lateral vy drives step_x (body-X),
// matching firmware motion.h. Swapping these crab-walks the robot.
inline float clip1(float v) {{ return v > 1.f ? 1.f : (v < -1.f ? -1.f : v); }}

inline void analytic_gait(const float cmd[3], float out[6]) {{
    const float vx = cmd[0], vy = cmd[1], yaw = cmd[2];
    const float speed = sqrtf(vx * vx + vy * vy);
    float b = speed / {c['blend_speed']:.8f}f;
    if (b > 1.f) b = 1.f;
    const float gx = {c['gx0']:.8f}f + ({c['gx1']:.8f}f - {c['gx0']:.8f}f) * b;
    const float gy = {c['gy0']:.8f}f + ({c['gy1']:.8f}f - {c['gy0']:.8f}f) * b;
    const float gyaw = {c['gyaw0']:.8f}f + ({c['gyaw1']:.8f}f - {c['gyaw0']:.8f}f) * b;
    out[0] = clip1(vy / gy);   // step_x  = body-X = lateral
    out[1] = clip1(vx / gx);   // step_y  = body-Y = forward
    out[2] = clip1(yaw / gyaw + {c['yaw_comp']:.8f}f * vx);
    out[3] = {c['step_height']:.8f}f;
    out[4] = b * 2.f - 1.f;
    // cadence rises with translational speed AND with |yaw| (an in-place turn has speed ~0)
    out[5] = clip1({c['pr_base']:.8f}f + {c['pr_slope']:.8f}f * speed
                   + {c.get('pr_yaw', 0.0):.8f}f * fabsf(yaw));
}}
"""


def constants_cpp(base):
    coordination = "// gait_blend interpolation endpoints (tripod at blend 0 -> bipod at blend 1)" \
        if base.schedule is None else \
        "// The run's base gait pins the coordination pattern, so both blend endpoints are the\n" \
        "// schedule's own offsets and duty and the gait_blend interpolation is a no-op."
    return f"""\
// --- action decoding / application constants (mirror hexapod_mj_env.py) ---
constexpr float CONTROL_DT = {CONTROL_DT}f;          // s, policy rate (50 Hz)
constexpr float FOOT_RESIDUAL_MM = {FOOT_RESIDUAL * 1000.0:.1f}f;  // residual authority per foot axis
constexpr float PG_STEP_XY = {PG_STEP_XY}f;          // mm, step_x/step_y scale
constexpr float PG_STEP_ANGLE = {PG_STEP_ANGLE}f;    // rad, step_angle scale
constexpr float PG_HEIGHT_MIN = {PG_HEIGHT[0]}f, PG_HEIGHT_MAX = {PG_HEIGHT[1]}f;  // mm
constexpr float PG_PHASE_RATE_MIN = {PG_PHASE_RATE[0]}f, PG_PHASE_RATE_MAX = {PG_PHASE_RATE[1]}f;  // cycles/s
constexpr float STEP_DEPTH_MM = {base.coef['step_depth']:.6f}f;
constexpr float BODY_ZM_MM = {BODY_ZM_MM}f;          // body ride-height authority (residual_gait)
// Base-gait ride height (mm, + = body raised). Apply as body.zm = -(BODY_RIDE_MM + policy delta);
// the legacy analytic map leaves this at 0, a searched schedule generally does not.
constexpr float BODY_RIDE_MM = {base.ride_mm:.6f}f;
// residual_gait gait-delta gains, order [step_x, step_y, step_height, gait_blend, phase_rate]:
// multiply the action BEFORE adding it to analytic_gait()'s output, then clip to [-1,1].
static const float GAIT_DELTA_GAIN[5] = {{{', '.join(f'{g}f' for g in GAIT_DELTA_GAIN)}}};
constexpr int HIST_STRIDE = {HIST_STRIDE};             // control steps between stacked sensor frames
// command scaling: joystick [-1,1] -> trained command ranges
constexpr float CMD_VX_MIN = {CMD_VX[0]}f, CMD_VX_MAX = {CMD_VX[1]}f;  // m/s (backward, forward)
constexpr float CMD_VY_MAX = {CMD_VY[1]}f;   // m/s
constexpr float CMD_YAW_MAX = {CMD_YAW[1]}f;  // rad/s
{coordination}
static const float TRI_OFFSET[6] = {{{', '.join(f'{v}f' for v in base.tri_offset)}}};
static const float BI_OFFSET[6] = {{{', '.join(f'{v}f' for v in base.bi_offset)}}};
constexpr float TRI_STAND_FRAC = {base.tri_stand_frac}f;
constexpr float BI_STAND_FRAC = {base.bi_stand_frac}f;
"""


def write_header(path, control_mode, layers, mean, var, clip, eps, test_obs, test_act, cfg, base):
    obs_dim = layers[0][0].shape[1]
    act_dim = layers[-1][0].shape[0]
    layer_in = [W.shape[1] for W, _ in layers]
    layer_out = [W.shape[0] for W, _ in layers]
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", newline="\n") as f:
        f.write("#pragma once\n")
        f.write("// Auto-generated by simulation/export_policy.py - DO NOT EDIT.\n")
        f.write(f"// control_mode={control_mode} obs_dim={obs_dim} act_dim={act_dim} "
                f"obs_history={cfg['obs_history']} obs_contact={int(cfg['obs_contact'])}\n")
        f.write(obs_layout_comment(cfg["obs_history"], cfg["obs_contact"], act_dim))
        f.write(ACTION_LAYOUT[control_mode])
        f.write("#include <math.h>\n\nnamespace hexapod_policy {\n\n")
        f.write(f"constexpr int OBS_DIM = {obs_dim};\nconstexpr int ACT_DIM = {act_dim};\n")
        f.write(f"constexpr int N_LAYERS = {len(layers)};\n")
        f.write(f"constexpr float OBS_CLIP = {clip:.6f}f;\nconstexpr float OBS_EPS = {eps:.3e}f;\n\n")
        f.write(constants_cpp(base))
        f.write("\n")
        f.write(analytic_gait_cpp(base))
        f.write("\n")
        f.write(carr("OBS_MEAN", mean))
        f.write(carr("OBS_VAR", var))
        f.write(f"static const int LAYER_IN[{len(layers)}] = {{{', '.join(map(str, layer_in))}}};\n")
        f.write(f"static const int LAYER_OUT[{len(layers)}] = {{{', '.join(map(str, layer_out))}}};\n\n")
        # names must dodge Arduino binary.h macros (B0, B1, B10, ...)
        for i, (W, b) in enumerate(layers):
            f.write(carr(f"WEIGHT{i}", W))   # row-major [out][in]
            f.write(carr(f"BIAS{i}", b))
        ptrs_w = ", ".join(f"WEIGHT{i}" for i in range(len(layers)))
        ptrs_b = ", ".join(f"BIAS{i}" for i in range(len(layers)))
        f.write(f"\nstatic const float* const WEIGHTS[{len(layers)}] = {{{ptrs_w}}};\n")
        f.write(f"static const float* const BIASES[{len(layers)}] = {{{ptrs_b}}};\n\n")
        f.write(carr("TEST_OBS", test_obs))
        f.write(carr("TEST_ACT", test_act))
        f.write("\n")
        f.write(
            "// NOT reentrant (static buffers keep the 4 KB control-task stack safe);\n"
            "// call from the control task only.\n"
            "inline void infer(const float* obs_raw, float* act_out) {\n"
            "    static float cur[256], nxt[256];\n"
            "    for (int i = 0; i < OBS_DIM; ++i) {\n"
            "        float v = (obs_raw[i] - OBS_MEAN[i]) / sqrtf(OBS_VAR[i] + OBS_EPS);\n"
            "        if (v > OBS_CLIP) v = OBS_CLIP; else if (v < -OBS_CLIP) v = -OBS_CLIP;\n"
            "        cur[i] = v;\n    }\n"
            "    int in_dim = OBS_DIM;\n"
            "    for (int l = 0; l < N_LAYERS; ++l) {\n"
            "        int out_dim = LAYER_OUT[l];\n"
            "        const float* W = WEIGHTS[l]; const float* B = BIASES[l];\n"
            "        for (int o = 0; o < out_dim; ++o) {\n"
            "            float acc = B[o]; const float* wrow = W + o * in_dim;\n"
            "            for (int j = 0; j < in_dim; ++j) acc += wrow[j] * cur[j];\n"
            "            nxt[o] = (l < N_LAYERS - 1) ? tanhf(acc) : acc;\n        }\n"
            "        for (int o = 0; o < out_dim; ++o) cur[o] = nxt[o];\n"
            "        in_dim = out_dim;\n    }\n"
            "    for (int o = 0; o < ACT_DIM; ++o) {\n"
            "        float a = cur[o]; if (a > 1.f) a = 1.f; else if (a < -1.f) a = -1.f;\n"
            "        act_out[o] = a;\n    }\n}\n\n"
            "// Boot-time self-check: returns max |infer(TEST_OBS) - TEST_ACT| (expect < 1e-4).\n"
            "inline float selfCheck() {\n"
            "    float act[ACT_DIM];\n"
            "    infer(TEST_OBS, act);\n"
            "    float err = 0.f;\n"
            "    for (int i = 0; i < ACT_DIM; ++i) {\n"
            "        float d = fabsf(act[i] - TEST_ACT[i]);\n"
            "        if (d > err) err = d;\n    }\n"
            "    return err;\n}\n\n} // namespace hexapod_policy\n"
        )
    return obs_dim, act_dim


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--control-mode", choices=sorted(ACTION_LAYOUT), default="residual_pure")
    ap.add_argument("--run", default=None, help="run dir under ./runs (sets --model/--vecnorm)")
    ap.add_argument("--model", default=None)
    ap.add_argument("--vecnorm", default=None)
    ap.add_argument("--out", default="../firmware/include/policy/hexapod_policy.h")
    args = ap.parse_args()
    cfg = {"control_mode": args.control_mode, "obs_contact": False, "obs_history": 1,
           "gait_schedule": None}
    if args.run:
        args.model = args.model or os.path.join("runs", args.run, "final_model.zip")
        args.vecnorm = args.vecnorm or os.path.join("runs", args.run, "vecnormalize.pkl")
        cfg.update(load_env_config(os.path.join("runs", args.run)))
    assert args.model and args.vecnorm, "pass --run <name> or both --model and --vecnorm"
    # The run config can name a mode --control-mode never offered. Emitting a header without that
    # mode's action layout and decode constants is worse than refusing: the firmware would apply a
    # partial, silently wrong decoding.
    if cfg["control_mode"] not in ACTION_LAYOUT:
        raise SystemExit(f"cannot export control_mode={cfg['control_mode']}: the header carries no "
                         f"action layout or decode constants for it. Supported: "
                         f"{', '.join(sorted(ACTION_LAYOUT))}.")
    sched = cfg["gait_schedule"]
    base = BaseGait(GaitSchedule.from_dict(sched) if sched else None)

    model = PPO.load(args.model, device="cpu")
    venv = VecNormalize.load(args.vecnorm, DummyVecEnv([make_env(
        control_mode=cfg["control_mode"], obs_contact=cfg["obs_contact"],
        obs_history=cfg["obs_history"], gait_schedule=base.schedule)]))
    mean = venv.obs_rms.mean.astype(np.float64)
    var = venv.obs_rms.var.astype(np.float64)
    clip, eps = float(venv.clip_obs), float(venv.epsilon)
    layers = extract(model)

    # self-check: numpy vs SB3 over random raw observations
    rng = np.random.default_rng(0)
    obs_dim = layers[0][0].shape[1]
    max_err = 0.0
    for _ in range(200):
        raw = rng.normal(mean, np.sqrt(var) + 1e-3).astype(np.float32)
        norm = venv.normalize_obs(raw)
        with torch.no_grad():
            sb3 = model.policy.get_distribution(
                torch.as_tensor(norm).float().unsqueeze(0)).distribution.mean.numpy()[0]
        npy = np_infer(raw.astype(np.float64), layers, mean, var, clip, eps, do_clip=False)
        max_err = max(max_err, float(np.max(np.abs(npy - sb3))))
    print(f"numpy-vs-SB3 parity max error: {max_err:.2e} (must be ~1e-6)")
    assert max_err < 1e-4, "parity check FAILED"

    # golden test vector for the firmware boot self-check (clipped like the C++ infer())
    test_obs = rng.normal(mean, np.sqrt(var) + 1e-3).astype(np.float32)
    test_act = np_infer(test_obs.astype(np.float64), layers, mean, var, clip, eps).astype(np.float32)

    od, ad = write_header(args.out, cfg["control_mode"], layers, mean, var, clip, eps,
                          test_obs, test_act, cfg, base)
    n_params = sum(W.size + b.size for W, b in layers)
    print(f"wrote {args.out}  (obs={od} act={ad}, {n_params:,} params, ~{n_params*4/1024:.0f} KB fp32)")
    print(f"control_mode={cfg['control_mode']} obs_history={cfg['obs_history']} "
          f"obs_contact={int(cfg['obs_contact'])}")
    print(f"base gait: {base.name}")
    todo = []
    if base.ride_mm != 0.0:
        todo.append(f"apply the base gait's ride height: body.zm = -(BODY_RIDE_MM + policy delta), "
                    f"BODY_RIDE_MM = {base.ride_mm:.3f} mm (zero action does NOT reproduce the base "
                    f"gait without it)")
    if cfg["obs_history"] > 1:
        todo.append(f"keep a ring buffer of the last {(cfg['obs_history']-1)*HIST_STRIDE+1} sensor "
                    f"frames and stack {cfg['obs_history']} of them, {HIST_STRIDE} steps apart")
    if cfg["obs_contact"]:
        todo.append("read 6 foot contact switches/FSRs into each sensor frame (hardware required)")
    if cfg["control_mode"] == "residual_gait":
        todo.append("apply all 6 gait deltas (step_x/step_y/step_height/blend/cadence/body_zm) "
                    "before IK, not just the 18 foot residuals")
    if todo:
        print("\nPolicyRunner must be updated for this header:")
        for t in todo:
            print(f"  - {t}")


if __name__ == "__main__":
    main()
