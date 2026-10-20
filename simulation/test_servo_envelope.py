"""servo_envelope.py on the simulation, where the MuJoCo joint shows whether a command moved it, and its analysis on a
recording shaped as the robot's, which carries the PWM and the boards' write results the host build has not."""
import argparse
import dataclasses
import math
import re

import numpy as np
import pytest

import servo_envelope as envelope
from session_prompts import Scripted, Terminal
from src.hexapod import Mode, Robot, servos
from src.hexapod.backends.sim import SimBackend
from src.hexapod.servos import ServoCalibration
from src.platform_shared import message_pb2 as pb
from src.sim import recording
from src.sim.recording import RecordingWriter

FEMUR = 1
# The simulated robot stands on the floor, not on a stand: a swing that lifts the foot (+30 to +90 deg) is unloaded, as
# the bench's is, where one that pushes the foot down would lift the body.
FROM_DEG = 30.0
SWEEP_MS = (160.0, 100.0, 80.0, 60.0)
REACHES_FROM_MS = 100.0  # the scripted operator sees both marks reached at this dwell and longer
# The scripted operator's end stops in the telemetry's angles: (+ side, - side, what stops the - side).
STOPS = {"coxa": (20.0, -25.0, "b"), "femur": (None, None, ""), "tibia": (-10.0, -140.0, "s")}


def arguments(**changes) -> argparse.Namespace:
    values = dict(servo=FEMUR, from_deg=FROM_DEG, repeats=2, dwell_s=0.5, sweep_ms=list(SWEEP_MS),
                  load_g=120.0, leg=0, joints=list(envelope.JOINTS), scale=None)
    values.update(changes)
    return argparse.Namespace(**values)


def operator() -> Scripted:
    def choose(prompt: str) -> str:
        if (dwell := re.match(r"at a dwell of ([\d.]+) ms", prompt)) is not None:
            return "y" if float(dwell[1]) >= REACHES_FROM_MS else "n"
        name, angle = re.match(r"(\w+) \(servo \d+\) at ([+-][\d.]+) deg", prompt).groups()
        upper, lower, blocked = STOPS[name]
        angle = float(angle)
        if upper is not None and angle > upper:
            return "s"
        if lower is not None and angle < lower:
            return blocked
        return envelope.FREE

    return Scripted(numbers={"servo rail voltage": 6.02, "move time": 92.0, "lowest servo rail": 5.71}, choose=choose)


class FemurWatch:
    """The MuJoCo femur's angle (degrees, in the telemetry's convention) after every tick, on the simulation's clock."""

    def __init__(self, backend: SimBackend):
        self.backend, self.t, self.degrees = backend, [], []
        backend.on_tick(self)

    def __call__(self):
        self.t.append(self.backend.now())
        self.degrees.append(math.degrees(self.backend.sim.data.qpos[self.backend.sim.qpos_adr[FEMUR]]))

    def at(self, t: float) -> float:
        return float(np.interp(t, self.t, self.degrees))


@pytest.fixture(scope="module")
def moved(tmp_path_factory):
    directory = tmp_path_factory.mktemp("move")
    backend = SimBackend()
    femur = FemurWatch(backend)
    results = envelope.session(Robot(backend=backend), operator(), directory, ["move"], arguments())
    assert backend.fresh_mode() is Mode.DEACTIVATED
    return directory, results, femur


def test_the_swing_alternates_its_ends_as_commanded_and_the_boards_take_every_command(moved):
    directory, results, _ = moved
    sent = envelope.commands(recording.load(directory / "move.hexrec"), FEMUR)
    assert len(sent) == 2 * 2 + 2 * envelope.CYCLES * len(SWEEP_MS)
    assert [c.degrees for c in sent] == [FROM_DEG + 60.0, FROM_DEG] * (len(sent) // 2)
    assert all(c.acknowledged for c in sent) and all(c.pwm is None for c in sent)  # the host build writes no PWM
    # Within each phase, to a tick: the simulation has no link to delay a command.
    index = 0
    for dwell, count in [(500.0, 4)] + [(d, 2 * envelope.CYCLES) for d in SWEEP_MS]:
        held = np.diff([c.t_s for c in sent[index:index + count]]) * 1000.0
        assert held == pytest.approx([dwell] * (count - 1), abs=5.0)
        index += count


def test_each_command_moves_the_femur_to_its_end_within_the_long_dwell(moved):
    directory, _, femur = moved
    sent = envelope.commands(recording.load(directory / "move.hexrec"), FEMUR)
    for command, following in zip(sent[:4], sent[1:5]):
        assert femur.at(following.t_s - 0.01) == pytest.approx(command.degrees, abs=2.0)


def test_the_sweep_bounds_the_move_time_and_the_operators_figures_are_kept(moved):
    directory, results, _ = moved
    move = results["move"]
    assert [step["reached"] for step in move["sweep"]] == [True, True, False, False]
    assert envelope.sweep_bounds(move["sweep"]) == (100.0, 80.0)
    assert (move["video_ms"], move["rail_v_holding"], move["rail_v_moving"], move["load_g"]) == (92.0, 6.02, 5.71,
                                                                                                120.0)
    text = (directory / "envelope.md").read_text()
    assert "From the sweep: over 80 ms and under 100 ms." in text and "Move time from the video: 92 ms." in text
    assert "| sweep 80 ms | 80 | 8 | 8 | 80 |" in text
    assert "Firmware simulation 0.1.1." in text


def test_the_travel_stops_where_the_operator_saw_each_joint_stop_and_at_the_firmwares_limit(tmp_path):
    backend = SimBackend()
    results = envelope.session(Robot(backend=backend), operator(), tmp_path, ["travel"], arguments(scale=2.5))
    travel = {t["joint"]: t for t in results["travel"]}
    assert travel["coxa"]["reached"] == {"+": 20.0, "-": -25.0}
    assert travel["coxa"]["stopped_by"] == {"+": "stopped on its end stop", "-": "blocked by the body or a leg"}
    assert travel["femur"]["reached"] == {"+": 90.0, "-": -90.0}
    assert travel["femur"]["stopped_by"] == {"+": "the firmware's limit", "-": "the firmware's limit"}
    assert travel["tibia"]["reached"] == {"+": -10.0, "-": -140.0}
    angles = recording.load(tmp_path / "travel.hexrec").ticks["angles"]
    assert angles[:, 0].max() == pytest.approx(25.0) and angles[:, 0].min() == pytest.approx(-30.0)
    assert np.abs(angles[:, 1]).max() == pytest.approx(90.0)  # never past the limit it was stepped to
    text = (tmp_path / "envelope.md").read_text()
    # 2 counts per degree commanded and 2.5 turned: 0.8 of each angle.
    coxa = ("| coxa | 0 | -31.5 to +31.5 | 20.0 | stopped on its end stop | -25.0 | blocked by the body or a leg | "
            "16.0 / -20.0")
    assert coxa + " |" in text
    assert backend.fresh_mode() is Mode.DEACTIVATED


class Calibrated:
    """A simulated backend whose robot reports `calibration` as its servo settings."""

    def __init__(self, backend: SimBackend, calibration: ServoCalibration):
        self._backend, self._calibration = backend, calibration

    def __getattr__(self, name):
        return getattr(self._backend, name)

    def servo_calibration(self):
        return self._calibration


def always_free() -> Scripted:
    return Scripted(choose=lambda prompt: envelope.FREE)


def commanded(path, servo: int) -> np.ndarray:
    """The joint's angles the robot took over the recording, as the telemetry reports them."""
    return recording.load(path).ticks["angles"][:, servo]


def unclipped_pwm(calibration: ServoCalibration, servo: int, degrees: np.ndarray) -> np.ndarray:
    """toPwm before its clip to the board's 0-4095, which would hide a pulse past the servo's range."""
    servo_deg = degrees + servos.TIBIA_OFFSET_DEG[servo] + calibration.center_angle[servo]
    return servo_deg * calibration.direction[servo] * calibration.conversion[servo] + calibration.center_pwm[servo]


# Telemetry +149 deg, the firmware's tibia limit, is 239 deg past the servo's centre and maps to a pulse of -172
# counts at the factory calibration, clipped to 0: far below the servo's 126 to 486.
@pytest.mark.parametrize("center_angle, plus_end, centre", [(0.0, 0.0, -90.0), (20.0, -20.0, -110.0)])
def test_the_travel_never_steps_a_tibia_past_its_servos_calibrated_pulse_range(tmp_path, center_angle, plus_end,
                                                                                 centre):
    tibia = 2
    calibration = dataclasses.replace(
        servos.FACTORY, center_angle=tuple(center_angle if i == tibia else 0.0 for i in range(18)))
    span = envelope.SERVO_HALF_RANGE_DEG * calibration.conversion[tibia]
    results = envelope.session(Robot(backend=Calibrated(SimBackend(), calibration)), always_free(), tmp_path,
                               ["travel"], arguments(joints=["tibia"]))
    (travel,) = results["travel"]
    assert travel["reached"] == {"+": plus_end, "-": -149.0}
    assert travel["stopped_by"] == {"+": "the servo's calibrated range", "-": "the firmware's limit"}
    angles = commanded(tmp_path / "travel.hexrec", tibia)
    assert angles.max() == pytest.approx(plus_end) and angles.min() == pytest.approx(-149.0)
    pwm = unclipped_pwm(calibration, tibia, angles)
    assert pwm.min() >= calibration.center_pwm[tibia] - span - 1e-3
    assert pwm.max() <= calibration.center_pwm[tibia] + span + 1e-3
    # Each side starts from the servo's centre pulse.
    assert envelope.joint_range(calibration, tibia).centre == pytest.approx(centre)


def test_enter_stops_the_step_out_instead_of_stepping_on(tmp_path, monkeypatch):
    asked = []
    monkeypatch.setattr("builtins.input", lambda prompt: asked.append(prompt) or "")
    results = envelope.session(Robot(backend=SimBackend()), Terminal(), tmp_path, ["travel"],
                               arguments(joints=["tibia"]))
    (travel,) = results["travel"]
    assert travel["reached"] == {"+": -90.0, "-": -90.0}
    assert travel["stopped_by"] == {"+": "stopped by the operator", "-": "stopped by the operator"}
    angles = commanded(tmp_path / "travel.hexrec", 2)
    # One step each way, then the ramp back to the stance's -106 deg, which passes no second step at -100.
    assert angles.max() == pytest.approx(-85.0) and np.isclose(angles, -95.0).any()
    assert not np.isclose(angles, -100.0).any()
    steps = [prompt for prompt in asked if "tibia (servo 2) at" in prompt]
    assert len(steps) == 2 and "[Enter stopped by the operator, f moved freely" in steps[0]


def test_quit_ends_the_joint_both_ways(tmp_path):
    choices = iter([envelope.FREE, envelope.QUIT])
    results = envelope.session(Robot(backend=SimBackend()), Scripted(choose=lambda prompt: next(choices)), tmp_path,
                               ["travel"], arguments(joints=["coxa"]))
    (travel,) = results["travel"]
    assert travel["reached"] == {"+": 5.0} and travel["stopped_by"] == {"+": "quit this joint"}
    assert commanded(tmp_path / "travel.hexrec", 0).min() >= -1e-6  # the - side was never stepped


def test_a_robot_switched_out_of_idle_stops_the_routine(tmp_path):
    backend = SimBackend()

    def someone_presses_stand(prompt):
        if "at -75.0 deg" in prompt:
            backend.set_mode(Mode.STAND)
        return envelope.FREE

    with pytest.raises(envelope.Refused, match="left IDLE for STAND"):
        envelope.session(Robot(backend=backend), Scripted(choose=someone_presses_stand), tmp_path, ["travel"],
                         arguments(joints=["tibia"]))
    assert backend.fresh_mode() is Mode.DEACTIVATED
    assert "travel" not in envelope.json.loads((tmp_path / "envelope.json").read_text())


def test_a_leg_that_does_not_exist_is_refused_before_anything_connects(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr("sys.argv", ["servo_envelope.py", "--sim", "--out", str(tmp_path / "e"), "--leg", "6",
                                     "travel"])
    with pytest.raises(SystemExit) as exit_:
        envelope.main()
    assert exit_.value.code == 2 and "--leg" in capsys.readouterr().err
    assert not (tmp_path / "e").exists()
    with pytest.raises(ValueError, match="no leg -1"):
        envelope.run_travel(Robot(backend=SimBackend()), always_free(), tmp_path, -1, ["coxa"])


def test_a_robot_that_is_not_switched_off_is_refused_and_left_as_it_is(tmp_path):
    backend = SimBackend()
    someone_else = Robot(backend=backend)
    someone_else.connect()
    someone_else.stand()
    someone_else.close()
    with pytest.raises(envelope.Refused, match="STAND"):
        envelope.session(Robot(backend=backend), operator(), tmp_path / "e", ["move"], arguments())
    assert backend.fresh_mode() is Mode.STAND and not (tmp_path / "e").exists()


def robot_recording(path, servo: int, commands: list[tuple[float, float]], failed_at: float, burst_ms: float):
    """Five seconds of a robot's telemetry: the joint commanded to each (time, degrees), its board's write failing on
    the command at `failed_at`, every write's PWM, and the gyro above rest for `burst_ms` after each command, read
    every 25 ms as the robot polls its IMU."""
    writer = RecordingWriter(path, {"firmwareVersion": "0.1.1", "firmwareElfSha256": "ab" * 32})
    angle = 0.0
    ticks = []
    for k in range(1000):
        t = k * 0.005
        started = [c for c in commands if c[0] <= t + 1e-9]
        if started and started[-1][1] != angle:
            angle = started[-1][1]
        command_tick = any(abs(t - c[0]) < 1e-9 for c in commands)
        moving = any(0 <= t - c[0] < burst_ms / 1000.0 for c in commands)
        tick = pb.TickSample(seq=k, t_us=round(t * 1e6), period_us=5000, imu_fresh=k % 5 == 0, mode=int(Mode.IDLE),
                             angles=[angle if i == servo else 0.0 for i in range(18)],
                             gyro=[0.6 if moving else 0.01 * (k % 3), 0.0, 0.0],
                             pwm=[300 + round(angle) if i == servo and command_tick else 0 for i in range(18)])
        tick.servo_write_right = (pb.SERVO_WRITE_FAILED if abs(t - failed_at) < 1e-9 else
                                  pb.SERVO_WRITE_OK if command_tick else pb.SERVO_WRITE_NONE)
        ticks.append(tick)
        if len(ticks) == 5:
            writer.write(pb.Message(telemetry_batch=pb.TelemetryBatch(batch_seq=k // 5, ticks=ticks))
                         .SerializeToString(), round(t * 1e6))
            ticks = []
    writer.close()


def test_a_robots_recording_gives_each_commands_write_its_pwm_and_the_imus_response(tmp_path):
    commands = [(1.0, 30.0), (2.0, -30.0), (3.0, 30.0)]
    robot_recording(tmp_path / "r.hexrec", FEMUR, commands, failed_at=2.0, burst_ms=100.0)
    rec = recording.load(tmp_path / "r.hexrec")
    sent = envelope.commands(rec, FEMUR)
    assert [(c.t_s, c.degrees, c.acknowledged, c.pwm) for c in sent] == [
        (1.0, 30.0, True, 330), (2.0, -30.0, False, 270), (3.0, 30.0, True, 330)]
    starts = [c.t_s for c in sent]
    response = envelope.imu_response_ms(rec, starts, starts[1:] + [4.0])
    # The gyro is read every 25 ms, so a 100 ms burst shows as its last sample, 75 ms after the command.
    assert response == pytest.approx([75.0] * 3)


def test_a_move_without_telemetry_still_reports_the_operators_figures(moved, tmp_path):
    _, results, _ = moved
    RecordingWriter(tmp_path / "move.hexrec", {"firmwareVersion": "0.1.1"}).close()
    text = envelope.report(tmp_path, results)
    assert "No telemetry was recorded." in text and "Move time from the video" not in text
    assert "rail at a servo connector: 6.02 V holding" in text
