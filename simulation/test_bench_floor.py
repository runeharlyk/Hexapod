"""bench_floor.py's routines on the simulation, with an overhead camera's track made from the model's true pose: what
each run records, what the summary makes of it, and how the session leaves the robot when it refuses, fails or is
interrupted."""
import dataclasses
import json
import math

import numpy as np
import pytest

import bench_floor
import ground_truth
import score_track
from session_prompts import Scripted
from src.hexapod import GoalState, Mode, Robot
from src.hexapod.backends.sim import SimBackend
from src.sim import recording

FLOOR = {"distance walked": 1.9, "heading change": 350.0, "distance from the start": 4.0, "heading error": 2.0,
         "battery": 7.8}
DISTANCE_M = 0.6  # each tri walk, shorter than the session's 2 m to keep the suite fast
TURN_DEG = 180.0


class TrueTrack:
    """An overhead camera that sees the model's true pose: a ground_truth.py track file, a row every 6 ticks (33 fps),
    on the simulation's clock, which score_track.py aligns with each recording, those that repeat a start motion
    near the session's offset."""

    def __init__(self, backend: SimBackend, path):
        self._backend, self._file, self._ticks = backend, open(path, "w"), 0
        self._file.write(f"# the simulation's true pose\n{ground_truth.TRACK_COLUMNS}\n")
        backend.on_tick(self)

    def __call__(self):
        self._ticks += 1
        if self._ticks % 6 == 0:
            sim = self._backend.sim
            x, y = sim.data.qpos[:2]
            self._file.write(f"{sim.data.time:.6f},{x:.5f},{y:.5f},{sim.base_yaw():.5f}\n")

    def close(self):
        self._file.close()


@pytest.fixture(scope="module")
def session_dir(tmp_path_factory):
    """The whole session once, every routine, its floor questions answered."""
    directory = tmp_path_factory.mktemp("floor")
    backend = SimBackend()
    track = TrueTrack(backend, directory / "track.csv")
    operator = Scripted(numbers=FLOOR)
    runs = bench_floor.plan(bench_floor.ROUTINES, DISTANCE_M, TURN_DEG)
    session = bench_floor.run_session(Robot(backend=backend), operator, directory, runs, "simulation")
    track.close()
    assert backend.fresh_mode() is Mode.DEACTIVATED
    return directory, session, operator


def test_every_routine_runs_and_records_its_own_contiguous_telemetry(session_dir):
    directory, session, _ = session_dir
    assert [run.routine for run in session.runs] == ["rest", "turn", *["tri"] * 3, *["metre"] * 3, "square", "circle"]
    assert [run.outcome for run in session.runs] == ["done"] * 10
    for run in session.runs:
        rec = recording.load(directory / run.file)
        t = rec.ticks
        assert rec.missing_ticks == 0 and rec.link_gaps == 0
        assert np.all(np.diff(t["t_us"]) == 5000)
        # Each recording opens and closes standing still: STILL_S in STAND or a stopped WALK, then the move.
        still = round(bench_floor.STILL_S / bench_floor.TICK_S) - 5
        assert not bench_floor.held(t)[:still].any() and not bench_floor.held(t)[-still:].any()
        assert np.ptp(t["odom_distance"][:still]) < 1e-3  # a goal, whose sticks the command does not show, too
        assert t["mode"][0] == Mode.STAND
        # A velocity run's sticks are the telemetry's command; a goal's the firmware writes itself, so only its
        # odometry moves.
        assert bench_floor.held(t).any() == (run.routine in ("turn", "tri"))
        assert (bench_floor.odometry_distance(t) > 0.2) == (run.routine in ("tri", "metre", "square", "circle"))


def test_the_tri_walks_send_their_sticks_and_walk_the_speed_model_within_its_slip(session_dir):
    directory, session, _ = session_dir
    tri = [run for run in session.runs if run.routine == "tri"]
    assert [run.stick["ly"] for run in tri] == list(bench_floor.TRI_STICKS)
    for run in tri:
        t = recording.load(directory / run.file).ticks
        stick, seconds = bench_floor.held_stick(t, 1), bench_floor.held_seconds(t)
        assert stick == pytest.approx(run.stick["ly"], abs=0.005)
        model = bench_floor.velocity_of(bench_floor.Gait.TRI_GATE, bench_floor.Sticks(0, stick, 0)).vx
        assert seconds == pytest.approx(DISTANCE_M / model, abs=0.05)
        # The odometry counts the feet's strokes; the simulated robot slips about 5 % (README, Goals).
        assert bench_floor.odometry_distance(t) / seconds == pytest.approx(model, rel=0.1)


def test_the_turn_reads_the_dmp_yaw_rate_counter_clockwise(session_dir):
    directory, session, _ = session_dir
    (turn,) = [run for run in session.runs if run.routine == "turn"]
    t = recording.load(directory / turn.file).ticks
    model = math.degrees(bench_floor.velocity_of(bench_floor.Gait.TRI_GATE, bench_floor.Sticks(0, 0, -0.5)).yaw_rate)
    assert model > 0
    assert bench_floor.dmp_yaw_rate(t) == pytest.approx(model, rel=0.1)
    assert bench_floor.odometry_yaw_rate(t) == pytest.approx(model, rel=0.1)


def test_the_goal_routines_close_on_the_odometry_and_report_each_goal(session_dir):
    directory, session, _ = session_dir
    square, circle = (next(run for run in session.runs if run.routine == name) for name in ("square", "circle"))
    assert square.goals == ["FORWARD REACHED", "TURN REACHED"] * 4 and bench_floor.goals(square) == "8 REACHED"
    assert circle.goals == ["ARC REACHED"]
    for run in (square, circle):
        distance, heading = bench_floor.odometry_closure(recording.load(directory / run.file).ticks)
        assert distance < 0.10 and abs(heading) < 5.0  # README, Goals: the TRI square closes within 4 cm, 2.6 deg


def test_the_camera_scores_every_run_against_the_floor(session_dir):
    directory, session, _ = session_dir
    track = score_track.Track.load(directory / "track.csv")
    moving = [run for run in session.runs if run.routine != "rest"]
    recs = {run.file: recording.load(directory / run.file) for run in moving}
    clock = bench_floor.session_clock([(run.file, run.routine, recs[run.file]) for run in moving], track)
    # The camera and the simulation share one clock; the turn and the three tri walks give it, and agree.
    assert clock.offset == pytest.approx(0.0, abs=0.1)
    assert sorted(clock.voters) == sorted(run.file for run in moving if run.routine in ("turn", "tri"))
    assert not clock.outliers and not clock.refused
    for run in moving:
        rec = recs[run.file]
        if run.routine == "metre":  # each repeats the others' start motion
            with pytest.raises(score_track.AlignmentRefused, match="repeats"):
                score_track.clock_offset(track, score_track.Telemetry.from_recording(rec))
        scored = bench_floor.camera_in_session(run.routine, rec, track, clock)
        assert isinstance(scored, score_track.Stretch), scored
        if run.routine == "tri":
            # The track is the true pose, so the camera sees what the odometry does, less the feet's slip.
            assert scored.track_made_good == pytest.approx(bench_floor.odometry_distance(rec.ticks), rel=0.1)
        elif run.routine == "metre":
            assert scored.track_made_good == pytest.approx(1.0, abs=0.08)  # the odometry's 1 m, less the slip
            assert scored.position_end < 0.08
        elif run.routine in ("square", "circle"):
            assert scored.track_made_good < 0.15 and scored.position_end < 0.15


def _later(rec: recording.Recording, seconds: float) -> recording.Recording:
    """The recording as a robot whose clock ran `seconds` ahead would have made it."""
    ticks = dict(rec.ticks, t_us=rec.ticks["t_us"] + round(seconds * 1e6))
    return dataclasses.replace(rec, ticks=ticks)


def test_a_session_offset_needs_two_runs_with_a_unique_start_that_agree(session_dir):
    directory, session, _ = session_dir
    track = score_track.Track.load(directory / "track.csv")
    runs = {run.file: (run.file, run.routine, recording.load(directory / run.file)) for run in session.runs
            if run.routine != "rest"}
    turn, *tri = (runs[run.file] for run in session.runs if run.routine in ("turn", "tri"))
    # The goal runs repeat their start motions, so however many there are, none gives the offset.
    goals = [runs[run.file] for run in session.runs if run.routine in ("metre", "square", "circle")]
    alone = bench_floor.session_clock(goals + [turn], track)
    assert alone.offset is None and "only 1" in alone.reason and not alone.voters
    # A run whose clock is off is left out, and the others still agree on the offset.
    shifted = (tri[1][0], "tri", _later(tri[1][2], 10.0))
    clock = bench_floor.session_clock([turn, tri[0], shifted, tri[2]], track)
    assert clock.offset == pytest.approx(0.0, abs=0.1)
    assert list(clock.outliers) == [tri[1][0]] and clock.outliers[tri[1][0]] == pytest.approx(-10.0, abs=0.1)
    # Two runs that disagree give none: their mean would match neither.
    split = bench_floor.session_clock([turn, shifted], track)
    assert split.offset is None and "agree" in split.reason and len(split.outliers) == 2
    # The summary says which runs gave the offset, which were left out, and that the left-out run was scored on its
    # own start motion.
    lines = "\n".join(bench_floor.clock_lines(session, clock))
    labels = {run.file: run.label for run in session.runs}
    assert "the median of" in lines and labels[turn[0]] in lines
    assert f"Left out, too far from the others: {labels[tri[1][0]]}" in lines
    assert "Scored on their own start motion, away from the session's offset" in lines
    # Without an offset the repeating runs are not scored, and say why.
    square = runs[next(run.file for run in session.runs if run.routine == "square")]
    assert "no session offset" in bench_floor.camera_in_session("square", square[2], track, split)


def test_a_given_offset_that_does_not_place_the_start_motion_is_refused(session_dir):
    directory, session, _ = session_dir
    track = score_track.Track.load(directory / "track.csv")
    square = recording.load(directory / next(run.file for run in session.runs if run.routine == "square"))
    telemetry = score_track.Telemetry.from_recording(square)
    # 6 s in, the square is halfway along a side: no turn, no start from standing, to match the window.
    with pytest.raises(score_track.AlignmentRefused, match="does not place it"):
        score_track.clock_offset(track, telemetry, near=6.0)
    assert score_track.clock_offset(track, telemetry, near=0.5)[0] == pytest.approx(0.0, abs=0.1)


def test_the_summary_carries_the_operators_figures_the_firmware_and_every_recording(session_dir):
    directory, session, operator = session_dir
    track = score_track.Track.load(directory / "track.csv")
    text = bench_floor.summary(directory, bench_floor.Session.load(directory), track)
    assert "firmware simulation 0.1.1" in text
    assert "## Camera clock" in text and "the median of" in text and "Left out" not in text
    tri = next(run for run in session.runs if run.routine == "tri")
    assert tri.floor == {"metres": FLOOR["distance walked"]} and tri.battery_v == FLOOR["battery"]
    floor_speed = FLOOR["distance walked"] / bench_floor.held_seconds(recording.load(directory / tri.file).ticks)
    row = next(line for line in text.splitlines() if line.startswith(f"| {tri.label} |"))
    cells = [cell.strip() for cell in row.strip("|").split("|")]
    assert cells[2] == "7.80" and cells[7] == f"{floor_speed:.3f}" and cells[8] != "-"
    assert "Not scored against the camera" not in text and "INVALID" not in text
    for run in session.runs:
        assert f"### {run.label}" in text and f"`{run.file}`, done" in text
    assert "## At rest" in text and "| Gyro noise (rad/s) |" in text
    # Every run asked for the battery and for its floor figures, and a confirmation before it moved.
    assert sum(prompt.startswith("battery") for prompt in operator.asked) == 10
    metre = next(run for run in session.runs if run.routine == "metre")
    row = next(line for line in text.splitlines() if line.startswith(f"| {metre.label} |"))
    cells = [cell.strip() for cell in row.strip("|").split("|")]
    assert cells[3] == "1 REACHED" and float(cells[4]) == pytest.approx(1.0, abs=0.011) and cells[5] == "1.900"
    assert json.loads((directory / "session.json").read_text())["runs"][0]["routine"] == "rest"


def test_a_robot_that_is_not_switched_off_is_refused_and_left_as_it_is(tmp_path):
    backend = SimBackend()
    someone_else = Robot(backend=backend)
    someone_else.connect()
    someone_else.stand()
    someone_else.close()
    with pytest.raises(bench_floor.Refused, match="STAND"):
        bench_floor.run_session(Robot(backend=backend), Scripted(), tmp_path / "s", bench_floor.plan(["rest"]),
                                "simulation")
    assert backend.fresh_mode() is Mode.STAND
    assert not (tmp_path / "s").exists()


class Tilted:
    """A simulated backend whose robot reports a body rolled `roll` degrees, as one standing on an obstacle does."""

    def __init__(self, backend: SimBackend, roll: float):
        self._backend, self._roll = backend, roll

    def __getattr__(self, name):
        return getattr(self._backend, name)

    def state(self):
        return dataclasses.replace(self._backend.state(), roll=self._roll)


def test_a_robot_that_does_not_stand_level_is_not_walked_and_is_switched_off(tmp_path):
    backend = SimBackend()
    with pytest.raises(bench_floor.Refused, match="rolls"):
        bench_floor.run_session(Robot(backend=Tilted(backend, 25.0)), Scripted(), tmp_path,
                                bench_floor.plan(["tri"], DISTANCE_M), "simulation")
    session = bench_floor.Session.load(tmp_path)
    assert session.runs[0].outcome.startswith("refused: the body rolls +25.0")
    assert not (tmp_path / session.runs[0].file).exists()
    assert backend.fresh_mode() is Mode.DEACTIVATED and (tmp_path / "summary.md").exists()


def test_ctrl_c_during_a_goal_cancels_it_switches_the_robot_off_and_keeps_what_was_recorded(tmp_path):
    backend = SimBackend()
    pressed = []

    def ctrl_c():
        if not pressed and backend.goal_status().state is GoalState.RUNNING and backend.odometry().x > 0.3:
            pressed.append(backend.now())
            raise KeyboardInterrupt

    backend.on_tick(ctrl_c)
    with pytest.raises(KeyboardInterrupt):
        bench_floor.run_session(Robot(backend=backend), Scripted(), tmp_path, bench_floor.plan(["square", "circle"]),
                                "simulation")
    assert pressed
    assert backend.goal_status().state is GoalState.CANCELLED
    assert backend.fresh_mode() is Mode.DEACTIVATED
    session = bench_floor.Session.load(tmp_path)
    assert [run.outcome for run in session.runs] == ["interrupted"]  # the circle never started
    rec = recording.load(tmp_path / "square.hexrec")
    assert rec.ticks["t_us"][-1] / 1e6 <= pressed[0] and rec.ticks["odom_y"].max() > 0.25
    assert "| square 1 m" in (tmp_path / "summary.md").read_text()


class SecondClient:
    """An app left open beside the script: every 200 ms while the robot stands or walks it sends its last command, the
    centred sticks of an app nobody touches (app/src/lib/stick-link.ts), which the firmware takes as the last writer
    until the script's next frame. On the robot the script's keep-alive also sends every 200 ms, so each client's
    frame stands about 100 ms; the simulation re-sends the script's sticks every 20 ms, so the app's frame is sent
    again on every tick of its 100 ms."""

    def __init__(self, backend: SimBackend):
        self._backend, self._ticks, self.sent = backend, 0, 0
        backend.on_tick(self)

    def __call__(self):
        self._ticks += 1
        if self._ticks % 40 < 20 and self._backend.fresh_mode() in (Mode.STAND, Mode.WALK):
            self._backend._loop.robot.command()
            self.sent += self._ticks % 40 == 0


def test_a_velocity_run_another_client_drove_too_is_invalid_and_reports_no_speed_or_rate(tmp_path):
    backend = SimBackend()
    app = SecondClient(backend)
    runs = bench_floor.plan(["turn"], turn_deg=90.0) + bench_floor.plan(["tri"], DISTANCE_M)[1:2]
    session = bench_floor.run_session(Robot(backend=backend), Scripted(numbers=FLOOR), tmp_path, runs, "simulation")
    assert app.sent > 10 and [run.outcome for run in session.runs] == ["done", "done"]
    text = (tmp_path / "summary.md").read_text()
    for run, measured in ((session.runs[0], slice(6, 9)), (session.runs[1], slice(6, 9))):
        t = recording.load(tmp_path / run.file).ticks
        reason = bench_floor.contention(t, bench_floor.sent_sticks(run))
        assert reason is not None and "centred sticks: another client" in reason
        row = next(line for line in text.splitlines() if line.startswith(f"| {run.label} |"))
        cells = [cell.strip() for cell in row.strip("|").split("|")]
        assert cells[measured] == ["-"] * 3 and cells[-1].startswith("INVALID: ")
        assert f"- {run.label}: " in text
    # The robot walked slower than the stick it was sent, which the summary would otherwise report as its speed.
    t = recording.load(tmp_path / session.runs[1].file).ticks
    model = bench_floor.velocity_of(bench_floor.Gait.TRI_GATE, bench_floor.sent_sticks(session.runs[1])).vx
    assert bench_floor.odometry_distance(t) / bench_floor.held_seconds(t) < 0.97 * model


def test_a_dead_man_stop_inside_the_hold_invalidates_the_run():
    sent = bench_floor.Sticks(0.0, 0.5, 0.0)
    command = np.zeros((40, 9))
    command[5:35, 1] = 0.5
    command[15:20, 1] = 0.0  # the dead-man zeroed the command until the next frame
    ticks = {"command": command, "link_lost": np.isin(np.arange(40), np.arange(15, 20))}
    assert bench_floor.contention(ticks, sent) == "the dead-man stop held 5 ticks of the hold"
    ticks["link_lost"][:] = False
    command[15:20, 1] = 0.5
    assert bench_floor.contention(ticks, sent) is None


def test_a_recording_without_telemetry_leaves_the_summary_and_the_report_whole(session_dir, tmp_path, monkeypatch):
    directory, session, _ = session_dir
    for run in session.runs:
        (tmp_path / run.file).write_bytes((directory / run.file).read_bytes())
    empty = next(run for run in session.runs if run.routine == "square")
    recording.RecordingWriter(tmp_path / empty.file, {"firmwareVersion": "0.1.1"}).close()
    bench_floor.Session.save(session, tmp_path)
    monkeypatch.setattr("sys.argv", ["bench_floor.py", "report", str(tmp_path)])
    assert bench_floor.main() == 0
    text = (tmp_path / "summary.md").read_text()
    assert f"| {empty.label} |" not in text and "No telemetry was recorded." in text
    assert "## Forward speed" in text and "## At rest" in text


def test_a_report_rewrites_the_summary_from_the_session_directory(session_dir, monkeypatch):
    directory, _, _ = session_dir
    monkeypatch.setattr("sys.argv", ["bench_floor.py", "report", str(directory), "--track",
                                     str(directory / "track.csv")])
    (directory / "summary.md").unlink()
    assert bench_floor.main() == 0
    assert "| Camera (m/s) |" in (directory / "summary.md").read_text()


def test_an_unknown_routine_is_refused_before_anything_runs():
    with pytest.raises(ValueError, match="no routine 'figure8'"):
        bench_floor.plan(["square", "figure8"])
