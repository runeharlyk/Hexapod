"""Fits the simulation's servo model, actuator lag and IMU noise to a telemetry recording (record.py) and writes them
as the centre of the domain randomization:

    uv run python identify.py recording.hexrec [--out recording.sysid.json]
    uv run python train_mj.py --randomize --dr-centre recording.sysid.json ...

Record the robot standing still for at least two seconds, then walking on flat ground.

What a recording can determine, and from what:
  - Gyro bias and IMU noise: from still stretches, where no written joint angle changes, so the gyro's mean is its
    bias and the spread of gyro, roll/pitch and gravity their noise. The roll/pitch bias is not identifiable: on a
    still robot a mounting tilt and a tilted floor look alike.
  - Sensor-to-servo latency: the ticks' compute_us, from the IMU read at the tick's start to the end of its servo write.
  - The servo factors set_servo_scale takes (stall, hold, kp, noload), the deadband, and the actuator lag (from the
    servo write until the servo acts on it, the wait for the next PWM frame included): by replaying the written joint
    angles through HexapodSim and matching the recorded gyro and gravity direction, the only measurements of the
    body's motion, since the servos report nothing back. The telemetry carries no IMU sample time, so the sample's age
    falls into the lag. Masses, ground friction and the IMU mounting stay nominal, so the fit absorbs their errors.

The replay starts from the robot standing still and powered, so a recording must begin with one; it stops at the first
lost or deactivated tick, after which the servos' targets are unknown. The cost is the noise-scaled misfit of gyro and
gravity direction, each channel's mean removed (a gyro bias or a mounting tilt), under a Huber loss, since a footfall
sampled on its impact spike would otherwise outweigh the rest of the recording.

The cost is rugged: a joint that sticks in its gear friction one physics step later, or a foot that lands a step
earlier, moves the body's motion in jumps, so no gradient or simplex step means anything at small scales. The fit is
therefore a response surface: the cost on a quasi-random design over the search box, a quadratic fitted to it, and its
vertex as the estimate; a second design around that vertex refines it. Each sigma combines the statistical one (the
quadratic's curvature, scaled by the residual variance; a lower bound on real data, whose residuals are not white)
with the vertex's scatter over a bootstrap of the quadratic's residuals, which measures the ruggedness. A direction the
quadratic does not curve along counts as spanning the whole search box. A parameter whose 4-sigma interval would span
more than half of today's randomization range is reported as not identifiable instead of with a value.
"""
from __future__ import annotations

import argparse
import json
from dataclasses import dataclass, replace
from pathlib import Path

import mujoco
import numpy as np
from scipy.stats import qmc

from src.sim import telemetry_stats
from src.sim.domain_rand import CONTROL_DT, Ranges
from src.sim.mj_runtime import HexapodSim
from src.sim.recording import Recording, load

TICK_S = telemetry_stats.CONTROL_PERIOD_US / 1e6
DEACTIVATED = 0           # MOTION_STATE position: the servos are off
STILL_DEG = 1e-3          # a written angle changing by more than this in a tick is a move
STILL_MARGIN_S = 0.5      # the body settles for this long after a move
MIN_STILL_SAMPLES = 40    # fresh IMU samples a noise estimate needs (1 s at the firmware's 40 Hz)
SETTLE_S = 1.0            # the replay's first second, while the model settles onto its feet, is not compared
HUBER = 3.0               # noise sigmas beyond which a residual counts linearly
DESIGN_POINTS = 128       # cost evaluations per round of the search
LOCATE_BEST = 8           # the design points whose mean starts the response surface
SURFACE_HALF_WIDTH = 0.15  # of the search range, around the sweeps' best point
MIN_HALF_WIDTH = 0.05     # the second round's box, 4 sigma of the first round's vertex, is at least this
BOOTSTRAP = 200
MISFIT_RMS = 2.0          # a fit residual this many noise sigmas means the model misses the recording


@dataclass(frozen=True)
class Parameter:
    name: str
    unit: str
    bounds: tuple[float, float]  # where the fit searches
    prior: tuple[float, float]   # today's randomization range, which an identified value must narrow

    def value(self, unit_value):
        return self.bounds[0] + unit_value * (self.bounds[1] - self.bounds[0])


_DEFAULT = Ranges()
DYNAMICS = (
    Parameter("stall", "", (0.6, 1.4), _DEFAULT.stall),
    Parameter("hold", "", (0.6, 1.4), _DEFAULT.hold),
    Parameter("kp", "", (0.4, 1.6), _DEFAULT.kp),
    Parameter("noload", "", (0.7, 1.3), _DEFAULT.noload),
    Parameter("deadband", "rad", (0.0, 0.04), _DEFAULT.deadband),
    # Twice the 20 ms frame of the servos' 50 Hz PWM, which alone delays a written angle by up to one frame.
    Parameter("lag_ms", "ms", (0.0, 40.0),
              (_DEFAULT.action_latency_steps[0] * CONTROL_DT * 1e3,
               (_DEFAULT.action_latency_steps[1] - 1) * CONTROL_DT * 1e3)),
)


@dataclass(frozen=True)
class Estimate:
    value: float | None
    std: float | None = None  # one sigma; None where the value is an empirical percentile
    unit: str = ""
    note: str = ""            # why it is not identifiable, or what qualifies it

    @property
    def identified(self) -> bool:
        return self.value is not None


def _not_identifiable(reason: str, unit: str = "") -> Estimate:
    return Estimate(None, unit=unit, note=reason)


def _moves(rec: Recording) -> np.ndarray:
    return np.r_[False, np.any(np.abs(np.diff(rec.ticks["angles"], axis=0)) > STILL_DEG, axis=1)]


def still_ticks(rec: Recording) -> np.ndarray:
    """Ticks with no written angle changing within STILL_MARGIN_S either side."""
    margin = round(STILL_MARGIN_S / TICK_S)
    moves = _moves(rec).astype(float)
    # "same" would return the window's length for a recording shorter than the window.
    return np.convolve(moves, np.ones(2 * margin + 1))[margin:margin + len(moves)] == 0


def imu_estimates(rec: Recording) -> dict[str, Estimate]:
    still = still_ticks(rec)
    rest = telemetry_stats.imu_at_rest(replace(rec, ticks={name: values[still] for name, values in rec.ticks.items()}))
    n = rest.samples
    names = [f"gyro_bias_{axis}" for axis in "xyz"] + ["gyro_noise_std", "rpy_noise_deg", "gravity_noise_std"]
    if n < MIN_STILL_SAMPLES:
        return {name: _not_identifiable(f"{n} still IMU samples, {MIN_STILL_SAMPLES} needed") for name in names}

    def pooled(spread: np.ndarray, unit: str, note: str = "") -> Estimate:
        """One standard deviation over the axes; its sigma is that of a variance from len(spread) * (n - 1) dof."""
        std = float(np.sqrt(np.mean(np.square(spread))))
        return Estimate(std, std / np.sqrt(2 * len(spread) * (n - 1)), unit, note)

    estimates = {f"gyro_bias_{axis}": Estimate(float(bias), float(noise / np.sqrt(n)), "rad/s")
                 for axis, bias, noise in zip("xyz", rest.gyro_bias, rest.gyro_noise)}
    estimates["gyro_noise_std"] = pooled(rest.gyro_noise, "rad/s")
    estimates["rpy_noise_deg"] = pooled(rest.rpy_noise_deg[:2], "deg", "roll and pitch; the DMP's yaw drifts")
    estimates["gravity_noise_std"] = pooled(rest.gravity_noise, "")
    estimates["rpy_bias"] = _not_identifiable("a mounting tilt and a tilted floor look alike on a still robot")
    return estimates


def latency_estimates(rec: Recording) -> dict[str, Estimate]:
    compute_ms = rec.ticks["compute_us"].astype(float) / 1e3
    if not len(compute_ms):
        return {"sensor_to_servo_ms": _not_identifiable("the recording holds no tick")}
    note = "empirical percentile of compute_us; the IMU sample's own age is not in the telemetry and falls into lag_ms"
    return {f"sensor_to_servo_ms_p{q}": Estimate(float(np.percentile(compute_ms, q)), unit="ms", note=note)
            for q in (50, 99)}


class Replay:
    """The recording's written joint angles replayed through HexapodSim; `imu(...)` gives the model's gyro and the
    x, y of its world-up direction in the body frame at the recorded IMU sample times."""

    def __init__(self, rec: Recording, ticks: int):
        t = {name: values[:ticks] for name, values in rec.ticks.items()}
        origin = t["t_us"][0]
        self.sim = HexapodSim()
        self.h = self.sim.model.opt.timestep
        self.base = mujoco.mj_name2id(self.sim.model, mujoco.mjtObj.mjOBJ_BODY, "base")
        self.ctrl = np.radians(t["angles"])
        written_s = (t["t_us"].astype(np.int64) - np.int64(origin) + t["compute_us"]) / 1e6
        # The servos hold each written angle until the next. The running integral of that is exact under linear
        # interpolation, and gives each physics step the mean of the delayed command over its span, which is exact
        # whenever the command does not change inside a step.
        knots = np.r_[-1.0, written_s]
        held = np.vstack([self.ctrl[:1], self.ctrl])
        self.knots = np.r_[knots, knots[-1] + 1.0]
        self.integral = np.vstack([np.zeros(18), np.cumsum(held * np.diff(self.knots)[:, None], axis=0)])
        sample_s = (t["t_us"].astype(np.int64) - np.int64(origin)) / 1e6
        self.used = t["imu_fresh"].astype(bool) & (sample_s >= SETTLE_S)
        self.sample_s = sample_s[self.used]
        self.steps = int(np.ceil(self.sample_s.max() / self.h)) + 1 if self.used.any() else 0
        self.measured = np.hstack([t["gyro"][self.used], t["gravity"][self.used][:, :2]])

    def _start(self, stall, hold, kp, noload, deadband) -> None:
        sim = self.sim
        sim.reset_to_stand()
        sim.set_servo_scale(kp=kp, stall=stall, noload=noload, hold=hold)
        sim.deadband = deadband
        sim.data.qpos[sim.qpos_adr] = self.ctrl[0]
        mujoco.mj_kinematics(sim.model, sim.data)
        foot_radius = sim.model.geom_size[sim.foot_geom_ids[0], 0]
        sim.data.qpos[2] += foot_radius - sim.data.site_xpos[sim.foot_site_ids, 2].min()
        mujoco.mj_forward(sim.model, sim.data)

    def imu(self, stall, hold, kp, noload, deadband, lag_ms) -> np.ndarray:
        sim = self.sim
        self._start(stall, hold, kp, noload, deadband)
        edges = np.arange(self.steps + 1) * self.h - lag_ms / 1e3
        integral = np.column_stack([np.interp(edges, self.knots, self.integral[:, j]) for j in range(18)])
        commands = np.diff(integral, axis=0) / self.h
        signals = np.empty((self.steps + 1, 5))
        signals[0] = 0.0
        for step, command in enumerate(commands, start=1):
            sim.set_joint_targets(command)
            sim.step_physics(1)
            signals[step, :3] = sim.gyro()
            signals[step, 3:] = sim.data.xmat[self.base, 6:8]  # world up in the body frame: R^T z
        times = np.arange(self.steps + 1) * self.h
        return np.column_stack([np.interp(self.sample_s, times, signals[:, c]) for c in range(5)])


class _Cost:
    """The Huber cost of the noise-scaled residuals, as a function of the parameters' unit values."""

    def __init__(self, replay: Replay, scale: np.ndarray):
        self.replay, self.scale = replay, scale

    def residuals(self, unit: np.ndarray) -> np.ndarray:
        values = [p.value(u) for p, u in zip(DYNAMICS, np.clip(unit, 0.0, 1.0))]
        error = (self.replay.imu(*values) - self.replay.measured) / self.scale
        return (error - error.mean(axis=0)).ravel()  # each channel's mean is a gyro bias or a mounting tilt

    def __call__(self, unit: np.ndarray) -> float:
        r = np.abs(self.residuals(unit))
        return float(np.sum(np.where(r < HUBER, r**2, 2 * HUBER * r - HUBER**2)))


def _features(x: np.ndarray) -> np.ndarray:
    """1, x_i, then x_i^2 / 2 and x_i x_j (i < j): a quadratic's coefficients are then its gradient and Hessian."""
    d = x.shape[1]
    pairs = [(i, j) for i in range(d) for j in range(i, d)]
    return np.column_stack([np.ones(len(x)), x] + [x[:, i] * x[:, j] * (0.5 if i == j else 1.0) for i, j in pairs])


def _gradient_hessian(coef: np.ndarray, d: int) -> tuple[np.ndarray, np.ndarray]:
    g, hessian, k = coef[1 : d + 1], np.zeros((d, d)), d + 1
    for i in range(d):
        for j in range(i, d):
            hessian[i, j] = hessian[j, i] = coef[k]
            k += 1
    return g, hessian


def _locate(cost: _Cost) -> np.ndarray:
    """A start for the response surface: the mean of the best few points of a quasi-random design over the whole
    search box. The single best point is a lucky draw of the ruggedness; the mean of several finds the basin."""
    unit = qmc.Sobol(len(DYNAMICS), seed=0).random(DESIGN_POINTS)
    costs = np.array([cost(u) for u in unit])
    return unit[np.argsort(costs)[:LOCATE_BEST]].mean(axis=0)


def _free(hessian: np.ndarray, flat_curvature: float) -> dict[int, set[int]]:
    """The parameters the quadratic does not curve along, each with the parameters it shares that direction with:
    while the Hessian over the rest has a direction flatter than `flat_curvature`, every parameter making up at least
    a quarter of it (or the largest share) is free. A smaller share is the direction leaking into a parameter the cost does pin."""
    free: dict[int, set[int]] = {}
    while True:
        kept = [i for i in range(len(hessian)) if i not in free]
        if not kept:
            return free
        curvature, vectors = np.linalg.eigh(hessian[np.ix_(kept, kept)])
        if curvature[0] > flat_curvature:
            return free
        share = vectors[:, 0] ** 2
        valley = {kept[k] for k in np.flatnonzero(share >= 0.25)} | {kept[int(np.argmax(share))]}
        for i in valley:
            free[i] = valley - {i}


def _vertex(g: np.ndarray, hessian: np.ndarray, kept: list[int]) -> np.ndarray:
    """The quadratic's minimum over the kept parameters, the free ones held at the design centre."""
    step = np.zeros(len(g))
    step[kept] = -np.linalg.solve(hessian[np.ix_(kept, kept)], g[kept])
    return step


@dataclass
class _Surface:
    vertex: np.ndarray            # unit values
    variance: np.ndarray          # unit values squared; inf for a free parameter
    free: dict[int, set[int]]     # see _free


def _surface(cost: _Cost, centre: np.ndarray, half_width: np.ndarray, residual_variance: float, seed: int) -> _Surface:
    d = len(centre)
    low, high = np.clip(centre - half_width, 0.0, 1.0), np.clip(centre + half_width, 0.0, 1.0)
    unit = qmc.scale(qmc.Sobol(d, seed=seed).random(DESIGN_POINTS), low, high)
    mid = (low + high) / 2
    costs = np.array([cost(u) for u in unit])
    features = _features(unit - mid)
    coef, *_ = np.linalg.lstsq(features, costs, rcond=None)
    fitted = features @ coef
    g, hessian = _gradient_hessian(coef, d)
    # A sigma (sqrt(2 s^2 / curvature)) wider than the whole search range is no curvature at all.
    free = _free(hessian, 2.0 * residual_variance)
    kept = [i for i in range(d) if i not in free]
    vertex = np.clip(mid + _vertex(g, hessian, kept), low, high)
    variance = np.full(d, np.inf)
    if kept:
        variance[kept] = 2.0 * residual_variance * np.diag(np.linalg.inv(hessian[np.ix_(kept, kept)]))
        # The ruggedness: the vertex's scatter over a bootstrap of the quadratic's residuals.
        rng = np.random.default_rng(seed)
        residuals = costs - fitted
        replicas = []
        for _ in range(BOOTSTRAP):
            resampled, *_ = np.linalg.lstsq(features, fitted + rng.choice(residuals, len(residuals)), rcond=None)
            replicas.append(np.clip(mid + _vertex(*_gradient_hessian(resampled, d), kept), low, high))
        variance[kept] += np.var(replicas, axis=0)[kept]
    return _Surface(vertex, variance, free)


def _estimate(p: Parameter, value: float, std: float, at_bound: bool, free_with: set[str] | None,
              misfit: str) -> Estimate:
    unit = f" {p.unit}" if p.unit else ""
    if free_with is not None:
        valley = f", free along the same valley as {', '.join(sorted(free_with))}" if free_with else ""
        return _not_identifiable(f"the cost does not curve along it{valley}: the recording cannot pin it", p.unit)
    if 4.0 * std > (p.prior[1] - p.prior[0]) / 2:
        return _not_identifiable(f"sigma {std:.3g}{unit} at {value:.3g}{unit}: the recording cannot pin it within half "
                                 f"of today's range ({p.prior[0]:g}-{p.prior[1]:g}{unit})", p.unit)
    notes = [f"at the search bound {p.bounds}, so the true value may lie beyond"] if at_bound else []
    return Estimate(float(value), float(std), p.unit, "; ".join(notes + ([misfit] if misfit else [])))


def replay_window(rec: Recording) -> int:
    """Ticks up to the first lost or deactivated one: beyond it the servos' targets are unknown."""
    seq = rec.ticks["seq"].astype(np.int64)
    ends = np.r_[np.flatnonzero(np.diff(seq) != 1) + 1, np.flatnonzero(rec.ticks["mode"] == DEACTIVATED), len(seq)]
    return int(ends.min())


def dynamics_estimates(rec: Recording, imu: dict[str, Estimate]) -> dict[str, Estimate]:
    refuse = lambda reason: {p.name: _not_identifiable(reason, p.unit) for p in DYNAMICS}
    ticks = replay_window(rec)
    settle = round(SETTLE_S / TICK_S)
    if ticks <= settle or not still_ticks(rec)[:settle].all():
        return refuse(f"the recording does not begin with the robot standing still and powered for {SETTLE_S:g} s")
    if not _moves(rec)[settle:ticks].any():
        return refuse("no written joint angle changes after the settle, so the servos are never exercised")
    replay = Replay(rec, ticks)
    if not replay.used.any():
        return refuse("no fresh IMU sample after the settle")
    gyro_noise = imu["gyro_noise_std"].value or _DEFAULT.gyro_noise_std
    gravity_noise = imu["gravity_noise_std"].value or _DEFAULT.gravity_noise_std
    cost = _Cost(replay, np.array([gyro_noise] * 3 + [gravity_noise] * 2))

    d = len(DYNAMICS)
    dof = replay.measured.size - d - 5
    start = _locate(cost)
    residual_variance = cost(start) / dof
    first = _surface(cost, start, np.full(d, SURFACE_HALF_WIDTH), residual_variance, seed=1)
    half_width = np.clip(4.0 * np.sqrt(first.variance), MIN_HALF_WIDTH, SURFACE_HALF_WIDTH)
    final = _surface(cost, first.vertex, half_width, residual_variance, seed=2)

    residual = cost.residuals(final.vertex)
    residual_rms = float(np.sqrt(np.mean(residual**2)))
    misfit = (f"the fit residual is {residual_rms:.1f} noise sigmas: the model misses the recording, so the sigma "
              f"understates the uncertainty") if residual_rms > MISFIT_RMS else ""
    estimates = {
        p.name: _estimate(p, p.value(u), np.sqrt(var) * (p.bounds[1] - p.bounds[0]), min(u, 1.0 - u) < 0.01,
                          {DYNAMICS[j].name for j in final.free[i]} if i in final.free else None, misfit)
        for i, (p, u, var) in enumerate(zip(DYNAMICS, final.vertex, final.variance))
    }
    estimates["fit_residual_rms"] = Estimate(residual_rms, note="in noise sigmas; near 1 when the model explains the "
                                                                "recording down to the IMU noise")
    return estimates


def identify(rec: Recording) -> dict[str, Estimate]:
    imu = imu_estimates(rec)
    return {**imu, **latency_estimates(rec), **dynamics_estimates(rec, imu)}


def centre(estimates: dict[str, Estimate], source: str) -> dict:
    """The JSON domain_rand.Ranges.identified reads: the identified values, and why the rest are missing."""
    return {
        "source": source,
        "estimates": {name: {"value": e.value, "std": e.std, "unit": e.unit}
                      for name, e in estimates.items() if e.identified and name != "fit_residual_rms"},
        "not_identifiable": {name: e.note for name, e in estimates.items() if not e.identified},
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("path")
    parser.add_argument("--out", help="the JSON file domain_rand.Ranges.identified reads (default: <recording>.sysid.json)")
    args = parser.parse_args()
    rec = load(args.path)
    print(f"firmware {rec.features.get('firmwareVersion', '?')}, {len(rec.ticks['seq'])} ticks")
    estimates = identify(rec)
    for name, e in estimates.items():
        if e.identified:
            spread = f" +- {e.std:.3g}" if e.std is not None else ""
            print(f"{name:24s} {e.value:10.4g}{spread} {e.unit}" + (f"  ({e.note})" if e.note else ""))
        else:
            print(f"{name:24s} not identifiable: {e.note}")
    out = Path(args.out) if args.out else Path(args.path).with_suffix(".sysid.json")
    out.write_text(json.dumps(centre(estimates, str(args.path)), indent=2))
    print(f"\nwrote {out}")


if __name__ == "__main__":
    main()
