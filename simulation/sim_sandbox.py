"""Manual kinematics/gait sandbox + speed/stability tester for the hexapod -- no RL.

Drives the classical firmware kinematics + gait engine directly with sliders. Modes:

  - Stand : pose the body directly (height, roll/pitch/yaw, x/y shift).
  - Gait  : classical walking with every gait param exposed as a slider, plus the pattern
            (tripod/bipod/wave/ripple). Telemetry shows achieved body speed and holds the peak.
  - Jump  : a kinematic jump -- a scripted crouch-then-extend body-height trajectory.
  - Policy: run a trained policy through the env, driven by the command sliders (loaded lazily).

  python sim_sandbox.py                              # default policy = residual_gait_v9
  python sim_sandbox.py --policy <run>               # choose a different policy for Policy mode

Opens a MuJoCo viewer + a scrollable Tkinter panel. Uses the firmware math (firmware_gait.py) on
the calibrated servo model (mj_runtime.py), so results map onto the real robot's STAND / WALK.
"""

import tkinter as tk
from tkinter import ttk
import numpy as np
import mujoco

from src.sim.mj_runtime import HexapodSim, CONTROL_DT, TERRAIN_MODEL_PATH, SERVO_KP, SERVO_STALL, SERVO_NOLOAD
from src.sim.terrain import randomize_hfield
from src.robot.firmware_gait import (
    Kinematics, GaitController, BodyState, GaitState, set_gait, DEFAULT_FEET,
    STAND_HEIGHT_MM, TRI_GATE, BI_GATE, WAVE, RIPPLE,
)

GAITS = {"tripod": TRI_GATE, "bipod": BI_GATE, "wave": WAVE, "ripple": RIPPLE}
# default duty factor (stand_frac) per pattern, from firmware_gait.set_gait
STAND_FRAC_PRESET = {"tripod": 3.1 / 6, "bipod": 2.1 / 6, "wave": 5.0 / 6, "ripple": 5.0 / 6}


class Sandbox:
    def __init__(self, policy_run="residual_gait_v9", walk_mode="residual_gait"):
        from src.envs.hexapod_mj_env import HexapodMjEnv
        # One env/sim shared by all modes; the terrain model is loaded so terrain stays toggleable.
        self.env = HexapodMjEnv(walk_mode, randomize=False, terrain=0.06)
        self.env.terrain = 0.0     # the sandbox owns the hfield; env.reset won't overwrite it
        self.sim = self.env.sim
        self.walk_mode = walk_mode
        self.policy_run = policy_run   # loaded lazily the first time Policy mode is selected
        self.model = None
        self._norm = None
        self.policy_obs = None
        self.hfield_id = mujoco.mj_name2id(self.sim.model, mujoco.mjtObj.mjOBJ_HFIELD, "terrain")
        self.terrain_seed = 0
        self.kin = Kinematics()
        self.gc = GaitController()
        self.body = BodyState()
        self.mode = "stand"
        self.phase = 0.0
        self.jump_frames = []   # queued zm (mm) values for a scripted kinematic jump
        self.peak = 0.0         # peak body height (jump)
        self.vel_ema = np.zeros(2)  # smoothed body-frame [fwd, lat] velocity (m/s)
        self.peak_speed = 0.0   # max smoothed total speed seen since last recenter
        self.vals = {}          # slider name -> tk.DoubleVar
        self.base_id = mujoco.mj_name2id(self.sim.model, mujoco.mjtObj.mjOBJ_BODY, "base")
        self.sim.reset_to_stand()

    # ------------------------------------------------------------------ helpers
    def v(self, name):
        return self.vals[name].get()

    def _height_to_zm(self, height_mm):
        return STAND_HEIGHT_MM - height_mm  # zm<0 extends legs (taller); see body_height ~ 66 - zm

    def _load_policy(self):
        """Lazily load the trained policy (torch) the first time Policy mode is used."""
        if self.model is not None or not self.policy_run:
            return
        from stable_baselines3 import PPO
        from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
        from src.envs.hexapod_mj_env import make_env
        try:
            self.model = PPO.load(f"runs/{self.policy_run}/final_model.zip", device="cpu",
                                  custom_objects={"lr_schedule": lambda _: 0.0, "clip_range": lambda _: 0.2})
            vn = VecNormalize.load(f"runs/{self.policy_run}/vecnormalize.pkl",
                                   DummyVecEnv([make_env(self.walk_mode)]))
            m, var, clip, eps = vn.obs_rms.mean, vn.obs_rms.var, vn.clip_obs, vn.epsilon
            self._norm = lambda o: np.clip((o - m) / np.sqrt(var + eps), -clip, clip).astype(np.float32)
            print(f"[policy] loaded {self.policy_run}")
        except Exception as e:
            print(f"[policy] failed to load '{self.policy_run}': {e}")
            self.model = None

    def switch_mode(self, m):
        if m == "policy":
            self._load_policy()
        self.mode = m
        self.phase = 0.0
        self.jump_frames = []
        self.recenter()

    def recenter(self):
        """Teleport back to the origin at rest and reset the peak-speed hold (repeatable runs)."""
        if self.mode == "policy" and self.model is not None:
            self.policy_obs, _ = self.env.reset()   # env.terrain frozen -> keeps the current hfield
        else:
            self.sim.reset_to_stand()
            self.body = BodyState()
        self.peak = 0.0
        self.vel_ema[:] = 0.0
        self.peak_speed = 0.0

    def regen_terrain(self):
        """Regenerate the heightfield for the chosen type/height/bumpiness and refresh the viewer."""
        self.terrain_seed += 1
        rng = np.random.default_rng(self.terrain_seed)
        kind = self.terrain_type.get()
        height = 0.0 if kind == "flat" else float(self.v("height_m"))
        gen_kind = "bumps" if kind == "flat" else kind
        randomize_hfield(self.sim.model, rng, height, kind=gen_kind, feature=float(self.v("bumpiness")))
        if getattr(self, "viewer", None) is not None:
            try:
                self.viewer.update_hfield(self.hfield_id)  # push new terrain to the GPU
            except Exception:
                pass
        self.recenter()

    def _on_gait_type(self, name):
        """Changing the pattern resets the duty-factor slider to that pattern's default."""
        self.vals["stand_frac"].set(round(STAND_FRAC_PRESET[name], 3))

    def do_jump(self):
        """Queue a crouch -> fast-extend body-height trajectory (the kinematic jump)."""
        crouch_zm = self._height_to_zm(STAND_HEIGHT_MM - self.v("crouch depth"))
        launch_zm = self._height_to_zm(self.v("launch height"))
        ct, lt = int(self.v("crouch ticks")), int(self.v("launch ticks"))
        self.jump_frames = [crouch_zm] * ct + [launch_zm] * lt
        self.peak = 0.0

    # ------------------------------------------------------------------ per-tick control
    def _apply_stand(self, zm_override=None):
        self.body.feet = DEFAULT_FEET.copy()  # planted stance; body pose is what moves
        self.body.zm = self._height_to_zm(self.v("height")) if zm_override is None else zm_override
        self.body.omega = np.radians(self.v("roll"))
        self.body.phi = np.radians(self.v("pitch"))
        self.body.psi = np.radians(self.v("yaw"))
        self.body.xm = self.v("x")
        self.body.ym = self.v("y")

    def _apply_gait(self):
        gait = GaitState()
        gait.gait_type = GAITS[self.gait_type.get()]
        set_gait(gait)                            # baseline per-leg offset for the pattern
        gait.step_x = self.v("step_lat")
        gait.step_z = self.v("step_fwd")   # firmware step_z is the forward (long) axis
        gait.step_angle = np.radians(self.v("step_angle"))
        gait.step_height = self.v("step_height")
        gait.stand_frac = self.v("stand_frac")    # duty factor override (fraction of cycle grounded)
        gait.step_depth = self.v("step_depth")    # stance-phase downward push (traction)
        self.body.zm = self._height_to_zm(self.v("height"))
        self.phase = (self.phase + CONTROL_DT * self.v("cadence")) % 1.0
        self.gc.set_phase(self.phase)
        self.gc.generate_feet(gait, self.body)    # writes body.feet for this phase

    def tick(self):
        if not self.viewer.is_running():
            self.root.destroy()
            return
        # live servo-model params (feel out strength/stiffness vs stability)
        self.sim.kp = self.v("kp")
        self.sim.stall = self.v("stall")
        self.sim.noload = self.v("noload")

        if self.mode == "policy" and self.model is not None:
            self.env.cmd[:] = [self.v("fwd"), self.v("lat"), self.v("yaw")]
            if self.policy_obs is None:
                self.policy_obs, _ = self.env.reset()
            a, _ = self.model.predict(self._norm(self.policy_obs), deterministic=True)
            self.policy_obs, _, term, _, _ = self.env.step(a)  # env.step advances physics itself
            if term:
                self.policy_obs, _ = self.env.reset()
        else:
            if self.jump_frames:                  # scripted kinematic jump in progress
                self._apply_stand(zm_override=self.jump_frames.pop(0))
            elif self.mode == "gait":
                self._apply_gait()
            else:                                 # stand (and post-jump)
                self._apply_stand()
            self.sim.set_joint_targets(self.kin.inverse_kinematics(self.body, degrees=False))
            self.sim.step_physics()
        self.peak = max(self.peak, self.sim.base_height())
        self._measure_speed()
        self._status()
        self.viewer.sync()
        self.root.after(int(CONTROL_DT * 1000), self.tick)

    def _measure_speed(self):
        """Smoothed body-frame (heading-relative) velocity + peak-speed hold."""
        d = self.sim.data
        qw, qx, qy, qz = d.qpos[3:7]
        yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        wvx, wvy = d.qvel[0], d.qvel[1]
        bvx = np.cos(yaw) * wvx + np.sin(yaw) * wvy
        bvy = -np.sin(yaw) * wvx + np.cos(yaw) * wvy
        self.vel_ema = 0.97 * self.vel_ema + 0.03 * np.array([bvx, bvy])
        if self.sim.feet_in_contact().any():  # ignore airborne startup transient
            self.peak_speed = max(self.peak_speed, float(np.hypot(*self.vel_ema)))

    def _status(self):
        h = self.sim.base_height() * 1000
        air = "" if self.sim.feet_in_contact().any() else "  AIRBORNE"
        peak = f"  peak {self.peak*1000:.0f}mm" if self.jump_frames or self.peak > 0.11 else ""
        self.status.set(f"{self.mode:5s}  height {h:5.1f}mm{air}{peak}")
        fwd, lat = self.vel_ema
        self.telemetry.set(
            f"fwd {fwd:+.3f}   lat {lat:+.3f}   speed {np.hypot(fwd, lat):.3f} m/s\n"
            f"PEAK speed {self.peak_speed:.3f} m/s      body {h:.0f} mm (stand 66)"
        )

    # ------------------------------------------------------------------ ui
    def _slider(self, parent, name, lo, hi, default, fmt="{:.0f}"):
        row = ttk.Frame(parent)
        row.pack(fill="x", padx=6, pady=1)
        ttk.Label(row, text=name, width=11).pack(side="left")
        var = tk.DoubleVar(value=default)
        self.vals[name] = var
        val_lbl = ttk.Label(row, width=5, anchor="e")
        val_lbl.pack(side="right")
        var.trace_add("write", lambda *_: val_lbl.config(text=fmt.format(var.get())))
        val_lbl.config(text=fmt.format(default))
        ttk.Scale(row, from_=lo, to=hi, orient="horizontal", variable=var).pack(
            side="left", fill="x", expand=True)

    def build_ui(self):
        self.root = tk.Tk()
        self.root.title("Hexapod gait / speed / stability tester")
        self.root.geometry("430x760")
        pad = dict(padx=8, pady=3)

        # --- fixed top: mode + live telemetry (never scrolled away) ---
        mf = ttk.LabelFrame(self.root, text="Mode")
        mf.pack(fill="x", **pad)
        self.mode_var = tk.StringVar(value=self.mode)
        for m in ("stand", "gait", "policy"):
            ttk.Radiobutton(mf, text=m.capitalize(), value=m, variable=self.mode_var,
                            command=lambda k=m: self.switch_mode(k)).pack(side="left", padx=6)
        ttk.Button(mf, text="Recenter", command=self.recenter).pack(side="right", padx=8)

        tf = ttk.LabelFrame(self.root, text="Measured speed (body frame)")
        tf.pack(fill="x", **pad)
        self.telemetry = tk.StringVar(value="")
        ttk.Label(tf, textvariable=self.telemetry, font=("TkFixedFont", 11),
                  anchor="w", justify="left").pack(fill="x", padx=6, pady=4)

        # velocity command for Policy mode
        cf = ttk.LabelFrame(self.root, text="Policy command (velocity)")
        cf.pack(fill="x", **pad)
        self._slider(cf, "fwd", -0.25, 0.45, 0.0, fmt="{:.2f}")   # vx, forward m/s (the fast axis)
        self._slider(cf, "lat", -0.12, 0.12, 0.0, fmt="{:.2f}")   # vy, lateral/strafe m/s (slow axis)
        self._slider(cf, "yaw", -1.0, 1.0, 0.0, fmt="{:.2f}")     # yaw rate rad/s

        # terrain: pick a type + height, hit "New terrain" to compare the gait on uneven ground
        terf = ttk.LabelFrame(self.root, text="Terrain -- pick type + height, then New terrain")
        terf.pack(fill="x", **pad)
        self.terrain_type = tk.StringVar(value="flat")
        row = ttk.Frame(terf); row.pack(fill="x", padx=6, pady=1)
        ttk.Label(row, text="type", width=11).pack(side="left")
        ttk.OptionMenu(row, self.terrain_type, "flat", "flat", "bumps", "rocks", "slope").pack(side="left")
        self._slider(terf, "height_m", 0.0, 0.15, 0.06, fmt="{:.3f}")   # bump/rock/slope height (m)
        self._slider(terf, "bumpiness", 0.5, 3.0, 1.0, fmt="{:.1f}")    # feature density / frequency
        ttk.Button(terf, text="New terrain", command=self.regen_terrain).pack(fill="x", padx=8, pady=2)

        # --- scrollable body: all the sliders ---
        outer = ttk.Frame(self.root)
        outer.pack(fill="both", expand=True)
        canvas = tk.Canvas(outer, borderwidth=0, highlightthickness=0)
        vsb = ttk.Scrollbar(outer, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=vsb.set)
        vsb.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)
        panel = ttk.Frame(canvas)
        canvas.create_window((0, 0), window=panel, anchor="nw", width=400)
        panel.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind_all("<MouseWheel>", lambda e: canvas.yview_scroll(int(-e.delta / 120), "units"))

        bf = ttk.LabelFrame(panel, text="Body pose (Stand + Jump base)")
        bf.pack(fill="x", **pad)
        self._slider(bf, "height", 40, 130, STAND_HEIGHT_MM)
        self._slider(bf, "roll", -25, 25, 0)
        self._slider(bf, "pitch", -25, 25, 0)
        self._slider(bf, "yaw", -25, 25, 0)
        self._slider(bf, "x", -40, 40, 0)
        self._slider(bf, "y", -40, 40, 0)

        gf = ttk.LabelFrame(panel, text="Gait (Walk) -- all firmware gait params")
        gf.pack(fill="x", **pad)
        self.gait_type = tk.StringVar(value="tripod")
        row = ttk.Frame(gf); row.pack(fill="x", padx=6)
        ttk.Label(row, text="type", width=11).pack(side="left")
        ttk.OptionMenu(row, self.gait_type, "tripod", *GAITS.keys(),
                       command=self._on_gait_type).pack(side="left")
        self._slider(gf, "step_fwd", -120, 120, 60)        # body-Y forward stride amplitude (mm)
        self._slider(gf, "step_lat", -120, 120, 0)         # body-X lateral stride amplitude (mm)
        self._slider(gf, "step_angle", -20, 20, 0)         # turn per step (deg); small + cadence = fast turn
        self._slider(gf, "step_height", 0, 60, 20)         # swing arc height (mm)
        self._slider(gf, "stand_frac", 0.20, 0.90, STAND_FRAC_PRESET["tripod"], fmt="{:.2f}")  # duty
        self._slider(gf, "step_depth", 0, 6, 0, fmt="{:.1f}")  # stance downward push (mm, traction)
        self._slider(gf, "cadence", 0, 8, 1.5, fmt="{:.2f}")   # step speed (cycles/s)

        sf = ttk.LabelFrame(panel, text="Servo model (LIVE) -- strength / stiffness")
        sf.pack(fill="x", **pad)
        self._slider(sf, "kp", 50, 400, SERVO_KP)               # position->velocity gain (stiffness)
        self._slider(sf, "stall", 0.2, 1.5, SERVO_STALL, fmt="{:.2f}")   # torque envelope (N.m)
        self._slider(sf, "noload", 5, 25, SERVO_NOLOAD)        # no-load speed cap (rad/s)

        jf = ttk.LabelFrame(panel, text="Kinematic jump")
        jf.pack(fill="x", **pad)
        self._slider(jf, "crouch depth", 0, 30, 20)      # mm below stand
        self._slider(jf, "launch height", 66, 140, 110)  # mm commanded at launch
        self._slider(jf, "crouch ticks", 1, 40, 20)
        self._slider(jf, "launch ticks", 1, 20, 6)
        ttk.Button(jf, text="JUMP", command=self.do_jump).pack(fill="x", **pad)

        self.status = tk.StringVar(value="stand")
        ttk.Label(self.root, textvariable=self.status, relief="sunken", anchor="w").pack(
            fill="x", side="bottom", ipady=3)

    def run(self):
        import mujoco.viewer
        self.build_ui()
        self.viewer = mujoco.viewer.launch_passive(self.sim.model, self.sim.data)
        # camera follows the base so the robot stays in view while it walks off
        self.viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        self.viewer.cam.trackbodyid = self.base_id
        self.viewer.cam.distance = 0.9
        self.viewer.cam.elevation = -20.0
        self.viewer.cam.azimuth = 180.0   # look along +X (forward); the default +Y view makes it look sideways
        self.root.after(int(CONTROL_DT * 1000), self.tick)
        self.root.mainloop()
        self.viewer.close()


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--policy", default="residual_gait_v9",
                    help="run name under runs/ used by Policy mode")
    ap.add_argument("--walk-mode", default="residual_gait", help="control mode of the policy")
    args = ap.parse_args()
    Sandbox(policy_run=args.policy, walk_mode=args.walk_mode).run()
