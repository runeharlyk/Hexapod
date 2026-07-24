"""Interactive control panel for the hexapod sim: walk + jump in one window.

Opens a MuJoCo viewer plus a Tkinter panel. All modes drive ONE shared physics
instance, so you can switch between them live:

  - Walk          : residual_pure walking policy; sliders set vx / vy / yaw.
  - Vertical jump : press "Jump" to fire one in-place hop.
  - Charge jump   : aim with the direction dial, hold "Charge" (robot crouches and
                    leans back), press "Release" to launch; readout shows the leap.

  python sim_gui.py                                  # default runs (see MODES)
  python sim_gui.py --walk-run residual_pure_stab --charge-run jump_charge
  python sim_gui.py --serial                         # also drive it with the USB controller
  python sim_gui.py --serial COM11                   # explicit port

Runs whose model dir is missing are disabled in the panel (that mode is greyed out).

Physical controller (--serial): the ESP-NOW handheld controller drives the sim over
USB, mapped exactly like the robot's firmware (espnow_adapter.cpp / motion.h WALK):
  - left stick  -> walk velocity: vx <- -lx (fwd/back), vy <- ly (strafe)
  - right stick -> yaw (rx); in charge mode it aims the leap
  - left button -> cycle mode (walk -> vertical -> charge)
  - aux A       -> jump / reset ; aux B -> charge release (launch)
"""

import argparse
import os
import tkinter as tk
from tkinter import ttk
import numpy as np

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from src.sim.mj_runtime import HexapodSim, CONTROL_DT
from src.envs.hexapod_mj_env import HexapodMjEnv, make_env, CMD_VX, CMD_VY, CMD_YAW
from src.envs.hexapod_jump_env import HexapodJumpEnv, make_jump_env, STAND_Z
from src.envs.hexapod_charge_jump_env import HexapodChargeJumpEnv, make_charge_jump_env, make_body_jump_env

# Stick -> walk command scaling. Full deflection maps to the policy's trained command
# range (asymmetric fwd/back), so the sticks reach exactly the speeds the policy knows.
STICK_DEADZONE = 0.06  # ignore residual off-center noise past the controller's own deadzone
MODE_ORDER = ("walk", "vertical", "charge")


class Policy:
    """A loaded PPO actor + its VecNormalize obs statistics (deterministic inference)."""

    def __init__(self, run, norm_base_thunk, logdir):
        rundir = os.path.join(logdir, run)
        model_path = os.path.join(rundir, "best", "best_model.zip")
        if not os.path.exists(model_path):
            model_path = os.path.join(rundir, "final_model.zip")
        self.model = PPO.load(model_path, device="cpu",
                              custom_objects={"lr_schedule": lambda _: 0.0, "clip_range": lambda _: 0.2})
        vn = VecNormalize.load(os.path.join(rundir, "vecnormalize.pkl"),
                               DummyVecEnv([norm_base_thunk]))
        self.mean, self.var, self.clip, self.eps = vn.obs_rms.mean, vn.obs_rms.var, vn.clip_obs, vn.epsilon

    def act(self, obs):
        o = np.clip((obs - self.mean) / np.sqrt(self.var + self.eps), -self.clip, self.clip).astype(np.float32)
        return self.model.predict(o, deterministic=True)[0]


def _run_exists(logdir, run):
    d = os.path.join(logdir, run)
    has_model = os.path.exists(os.path.join(d, "final_model.zip")) or \
        os.path.exists(os.path.join(d, "best", "best_model.zip"))
    has_norm = os.path.exists(os.path.join(d, "vecnormalize.pkl"))  # written on completion
    return has_model and has_norm


class SimGUI:
    def __init__(self, args):
        self.shared = HexapodSim()  # one physics instance shared by every mode
        self.logdir = args.logdir

        # (label, run, env-instance, norm-base-thunk); envs get their sim swapped to the shared one
        specs = {
            "walk": ("Walk", args.walk_run, HexapodMjEnv(args.walk_mode), make_env(args.walk_mode)),
            "vertical": ("Vertical jump", args.vertical_run, HexapodJumpEnv(), make_jump_env()),
            "charge": ("Charge jump", args.charge_run,
                       HexapodChargeJumpEnv(auto_release=False, action_mode=args.charge_mode),
                       make_body_jump_env() if args.charge_mode == "body" else make_charge_jump_env()),
        }
        self.envs, self.policies, self.available = {}, {}, {}
        for key, (label, run, env, norm_base) in specs.items():
            env.sim = self.shared  # all modes act on the same MjModel/MjData
            self.envs[key] = env
            ok = _run_exists(self.logdir, run)
            self.available[key] = ok
            self.policies[key] = Policy(run, norm_base, self.logdir) if ok else None
            if not ok:
                print(f"[{label}] run '{run}' not found -> mode disabled")

        self.labels = {k: v[0] for k, v in specs.items()}
        self.cmd = [0.0, 0.0, 0.0]          # walk command (vx, vy, yaw)
        self.charge_dir_deg = 0.0           # charge aim (degrees, body frame)
        self.mode = None
        self.obs = None
        self.stepping = False

        # Optional USB controller. args.serial is None (disabled), "" (auto-detect),
        # or an explicit port string.
        self.bridge = None
        if args.serial is not None:
            from controller_bridge import ControllerBridge
            self.bridge = ControllerBridge(port=args.serial or None).start()

    # ------------------------------------------------------------------ modes
    def switch_mode(self, key):
        if not self.available.get(key):
            return
        self.mode = key
        self.obs, _ = self.envs[key].reset()
        self.stepping = (key == "walk")  # walk runs continuously; jumps are triggered
        self._status(f"mode: {self.labels[key]}")

    def trigger(self):
        """Fire the active jump (or reset walking)."""
        env = self.envs[self.mode]
        if self.mode == "charge":
            env.set_direction(np.radians(self.charge_dir_deg))
        self.obs, _ = env.reset()
        self.stepping = True

    def release(self):
        if self.mode == "charge" and self.stepping:
            self.envs["charge"].release()

    # ------------------------------------------------------------------ controller
    @staticmethod
    def _dz(v):
        return 0.0 if abs(v) < STICK_DEADZONE else v

    def poll_controller(self):
        """Apply one frame of USB-controller input: buttons (edges) then sticks."""
        b = self.bridge
        rising = b.take_rising()

        from controller_bridge import BTN_LEFT, BTN_A, BTN_B
        if rising & BTN_LEFT:  # cycle mode among the available ones (mirrors firmware left-button)
            avail = [k for k in MODE_ORDER if self.available.get(k)]
            if avail:
                cur = self.mode if self.mode in avail else avail[0]
                nxt = avail[(avail.index(cur) + 1) % len(avail)]
                self.mode_var.set(nxt)
                self.switch_mode(nxt)
        if rising & BTN_A:  # jump / reset the active mode
            self.trigger()
        if rising & BTN_B:  # launch a held charge
            self.release()

        lx, ly, rx, ry = (self._dz(v) for v in b.snapshot())
        # Mirror motion.h WALK: step_x = -lx, step_z = ly, step_angle = rx.
        vx = -lx
        self.cmd[0] = vx * (CMD_VX[1] if vx >= 0 else -CMD_VX[0])
        self.cmd[1] = ly * CMD_VY[1]
        self.cmd[2] = rx * CMD_YAW[1]
        for s, v in zip(getattr(self, "_walk_scales", []), self.cmd):
            s.set(v)  # reflect controller input on the sliders

        # In charge mode the right stick aims the leap when deflected.
        if self.mode == "charge" and (abs(rx) > STICK_DEADZONE or abs(ry) > STICK_DEADZONE):
            self.charge_dir_deg = float(np.degrees(np.arctan2(ry, rx))) % 360.0
            if hasattr(self, "dir_scale"):
                self.dir_scale.set(self.charge_dir_deg)

    # ------------------------------------------------------------------ loop
    def tick(self):
        if not self.viewer.is_running():
            self.root.destroy()
            return
        if self.bridge is not None:
            self.poll_controller()
        env = self.envs[self.mode]
        if self.stepping:
            if self.mode == "walk":
                env.cmd[:] = self.cmd
            elif self.mode == "charge" and env.charging:
                env.set_direction(np.radians(self.charge_dir_deg))
            a = self.policies[self.mode].act(self.obs)
            self.obs, _, term, trunc, _ = env.step(a)
            if term or trunc:
                if self.mode == "walk":
                    self.obs, _ = env.reset()  # keep walking
                else:
                    self.stepping = False       # hop finished; wait for next trigger
            self._telemetry()
        self.viewer.sync()
        self.root.after(int(CONTROL_DT * 1000), self.tick)

    def _telemetry(self):
        h = self.shared.base_height()
        if self.mode == "vertical":
            extra = f"apex {self.envs['vertical'].peak_air*1000:5.1f} mm"
        elif self.mode == "charge":
            e = self.envs["charge"]
            extra = f"{'CHARGING' if e.charging else 'LAUNCHED'}  leap {e.peak_dist*1000:5.0f} mm"
        else:
            v = self.shared.data.qvel[0:2]
            extra = f"vel {np.hypot(v[0], v[1]):.2f} m/s"
        self._status(f"{self.labels[self.mode]:14s}  height {h*1000:5.1f} mm   {extra}")

    def _status(self, text):
        if hasattr(self, "status"):
            self.status.set(text)

    # ------------------------------------------------------------------ ui
    def build_ui(self):
        self.root = tk.Tk()
        self.root.title("Hexapod sim control")
        self.root.geometry("340x430")
        pad = dict(padx=8, pady=3)

        self.mode_var = tk.StringVar(value="")
        mf = ttk.LabelFrame(self.root, text="Mode")
        mf.pack(fill="x", **pad)
        for key in ("walk", "vertical", "charge"):
            rb = ttk.Radiobutton(mf, text=self.labels[key], value=key, variable=self.mode_var,
                                 command=lambda k=key: self.switch_mode(k))
            if not self.available[key]:
                rb.state(["disabled"])
            rb.pack(anchor="w", padx=6)

        wf = ttk.LabelFrame(self.root, text="Walk command")
        wf.pack(fill="x", **pad)
        self._slider(wf, "vx (fwd)", -0.30, 0.45, 0)
        self._slider(wf, "vy (strafe)", -0.25, 0.25, 1)
        self._slider(wf, "yaw", -1.5, 1.5, 2)
        ttk.Button(wf, text="Stop", command=self._stop_walk).pack(**pad)

        jf = ttk.LabelFrame(self.root, text="Jump")
        jf.pack(fill="x", **pad)
        ttk.Button(jf, text="Jump (vertical / charge reset)", command=self.trigger).pack(fill="x", **pad)
        ttk.Label(jf, text="Charge aim (deg)").pack(anchor="w", padx=6)
        self.dir_scale = ttk.Scale(jf, from_=0, to=360, orient="horizontal",
                                   command=lambda v: setattr(self, "charge_dir_deg", float(v)))
        self.dir_scale.pack(fill="x", padx=6)
        ttk.Button(jf, text="Release (launch)", command=self.release).pack(fill="x", **pad)

        self.status = tk.StringVar(value="select a mode")
        ttk.Label(self.root, textvariable=self.status, relief="sunken", anchor="w").pack(
            fill="x", side="bottom", ipady=3)

    def _slider(self, parent, label, lo, hi, idx):
        row = ttk.Frame(parent)
        row.pack(fill="x", padx=6)
        ttk.Label(row, text=label, width=10).pack(side="left")
        s = ttk.Scale(row, from_=lo, to=hi, orient="horizontal",
                      command=lambda v, i=idx: self._set_cmd(i, float(v)))
        s.set(0.0)
        s.pack(side="left", fill="x", expand=True)
        if not hasattr(self, "_walk_scales"):
            self._walk_scales = []
        self._walk_scales.append(s)

    def _set_cmd(self, i, v):
        self.cmd[i] = v

    def _stop_walk(self):
        self.cmd[:] = [0.0, 0.0, 0.0]
        for s in getattr(self, "_walk_scales", []):
            s.set(0.0)

    # ------------------------------------------------------------------ run
    def run(self):
        import mujoco.viewer

        self.build_ui()
        first = next((k for k in ("walk", "vertical", "charge") if self.available[k]), None)
        if first is None:
            raise SystemExit("no trained runs found -- train a policy first")
        self.viewer = mujoco.viewer.launch_passive(self.shared.model, self.shared.data)
        self.mode_var.set(first)
        self.switch_mode(first)
        self.root.after(int(CONTROL_DT * 1000), self.tick)
        self.root.mainloop()
        self.viewer.close()
        if self.bridge is not None:
            self.bridge.close()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--logdir", default="./runs")
    ap.add_argument("--walk-run", default="residual_gait_terrain")
    ap.add_argument("--walk-mode", default="residual_gait",
                    help="walk policy action space: residual_gait (default) or residual_pure")
    ap.add_argument("--vertical-run", default="jump_v2")
    ap.add_argument("--charge-run", default="jump_sim2real5")
    ap.add_argument("--charge-mode", choices=["joint", "body"], default="body",
                    help="charge policy action space: 'body' (jump_body*) or 'joint' (jump_charge*)")
    ap.add_argument("--serial", nargs="?", const="", default=None,
                    help="drive the sim with the USB controller; bare flag auto-detects, "
                         "or pass a port (e.g. --serial COM11)")
    args = ap.parse_args()
    SimGUI(args).run()
