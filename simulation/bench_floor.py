"""Runs the floor session's routines on the robot or the simulation, records each run's telemetry, and summarises them:

    uv run python bench_floor.py run --host 192.168.1.151 --out floor-20261025
    uv run python bench_floor.py run --port COM5 --out floor-20261025 turn tri
    uv run python bench_floor.py run --sim --out runs/floor-sim
    uv run python bench_floor.py report floor-20261025 --track track.csv

Roadmap 3.1 names this script for the floor benchmark; its first routines are the ones roadmap 1.6 and the goal moves
need. The routines, in their default order, cheapest and safest first:

    rest     standing still for REST_S: the control loop's timing and the IMU at rest against domain_rand.py's ranges
    turn     a velocity turn in place on TRI at TURN_RX through --turn-deg: the DMP's yaw rate against the speed model's
    tri      three velocity walks forward on TRI at TRI_STICKS over --distance: the speed against the speed model's
    metre    METRE_RUNS goal walks of 1 m: the distance against a tape measure (roadmap 3.2) and the odometry's
    square   goal moves, four times a 1 m walk and a 90 deg turn: the odometry's closure
    circle   a goal arc of 0.5 m radius through 360 deg: the odometry's closure

Every run is one .hexrec telemetry recording (record.py's format, through Robot.recording), opened while the robot
stands still for STILL_S before the move and STILL_S after it, which also opens each recording with a still stretch for
score_track.py to align on. The velocity walks send the speed model's sticks for its speed at the stick (stride 0, the
model's own), and the summary takes the stick sent, the time it was held and the distance from the recording itself, so
a link's latency does not enter: the speed is the distance over the held time, as the firmware's first-order command
ramp keeps the distance a held stick walks.

Safety. The robot must report DEACTIVATED when the script connects, so it never takes over a robot someone else drives.
Each run waits for the operator to place the robot and clear the floor, stands it, and refuses to go on unless the
robot reports STAND, the handheld is not driving and the body rolls less than MAX_ROLL_DEG. After each run the robot is
lowered onto its belly and switched off, so it is moved between runs unpowered. Ctrl-C cancels a goal, rests the robot,
switches it off and writes the summary of the runs so far; so does leaving on any error. A run whose goal timed out,
was cancelled or was refused, or that the robot answered with any other error, is recorded as such and the session
goes on; the script's own refusal (a robot not standing level, or the handheld driving), a lost link and Ctrl-C end it.

One driver at a time. The app re-sends its last command every 200 ms while the robot is in STAND or WALK, centred
sticks when nobody touches it, and the firmware takes the last command from any client, so an app left open walks a
velocity run at a fraction of its stick. Every app tab is closed and the handheld switched off for the session, and
the summary checks each velocity run: from the first tick that carries the script's stick to the last, every tick
must carry the same command, or the run is marked INVALID with the reason and its speed or rate is not reported.

The floor's own figures are asked for after each run on the robot, all optional: the distance walked between the
start and end marks, the turn's heading change, a closure's distance from the start mark and its heading error, and
the battery voltage before each run, measured at the pack, since the robot reports none (AnalyticsData has no battery
field; roadmap 2.1). With an overhead camera's track (ground_truth.py --camera ... --out track.csv, running through the
session), `report --track` scores every recording against it through score_track.py.

The output directory holds session.json (each run's settings, outcome and the operator's figures), one .hexrec per run
and summary.md, whose tables are written for simulation/docs/sim-to-real.md. `report` writes summary.md again from
session.json and the recordings, with a track when given.
"""
import argparse
import json
import math
import sys
from collections import Counter
from contextlib import ExitStack
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path

import numpy as np

import report_recording
import score_track
from session_prompts import Scripted, Terminal
from src.hexapod import Gait, GoalTimedOut, Mode, Robot, RobotDisconnected, RobotError, Velocity
from src.hexapod.speed_model import Sticks, sticks_for, velocity_of
from src.sim import recording
from src.sim import telemetry_stats as ts

ROUTINES = ("rest", "turn", "tri", "metre", "square", "circle")
REST_S = 20.0
STILL_S = 2.0
STAND_S = 1.5  # the stand-up from the belly settles well within this
TRI_STICKS = (0.3, 0.6, 1.0)
TURN_RX = -0.5  # a negative rx turns counter-clockwise (speed_model.py)
DISTANCE_M = 2.0
TURN_DEG = 360.0
SQUARE_SIDE_M = 1.0
METRE_RUNS = 3
CIRCLE_RADIUS_M = 0.5
GOAL_SPEED = 0.5
MAX_ROLL_DEG = 10.0
# The DMP's yaw is clockwise-positive about the chip's Z (imu_frame.h YAW_SIGN), so it is negated for counter-clockwise.
DMP_YAW_SIGN = -1.0
# The rate is fitted after the command ramp (GAIT_COMMAND_TAU_S, 0.15 s) has settled and before the stop.
RATE_SKIP_S = 1.0
TICK_S = ts.CONTROL_PERIOD_US / 1e6
STICK_TOLERANCE = 1e-4  # the telemetry carries the sticks as float32


class Refused(Exception):
    pass


@dataclass
class Run:
    """One recording: what was asked for, how it ended, and what the operator measured on the floor."""

    routine: str
    label: str
    file: str
    battery_v: float | None = None
    stick: dict[str, float] = field(default_factory=dict)  # the sticks asked for, before the speed model's inversion
    goals: list[str] = field(default_factory=list)  # each goal's end, as "TURN REACHED"
    outcome: str = "not run"  # done, timed out, failed: <reason>, interrupted
    floor: dict[str, float] = field(default_factory=dict)  # the operator's figures, by name


@dataclass
class Session:
    started: str
    target: str
    features: dict
    runs: list[Run] = field(default_factory=list)

    def save(self, directory: Path) -> None:
        (directory / "session.json").write_text(json.dumps(asdict(self), indent=1) + "\n")

    @classmethod
    def load(cls, directory: Path) -> "Session":
        data = json.loads((directory / "session.json").read_text())
        return cls(data["started"], data["target"], data["features"], [Run(**run) for run in data["runs"]])


def plan(routines, distance: float = DISTANCE_M, turn_deg: float = TURN_DEG) -> list[Run]:
    runs = []
    for routine in routines:
        if routine == "rest":
            runs.append(Run("rest", f"standing still {REST_S:g} s", "rest.hexrec"))
        elif routine == "turn":
            runs.append(Run("turn", f"TRI turn rx {TURN_RX:+.2f} through {turn_deg:g} deg", "turn.hexrec",
                            stick={"rx": TURN_RX, "degrees": turn_deg}))
        elif routine == "tri":
            runs += [Run("tri", f"TRI forward ly {ly:.2f} over {distance:g} m", f"tri-ly{round(ly * 100):03d}.hexrec",
                         stick={"ly": ly, "metres": distance}) for ly in TRI_STICKS]
        elif routine == "metre":
            runs += [Run("metre", f"goal walk 1 m, TRI at speed {GOAL_SPEED:g}, {k} of {METRE_RUNS}",
                         f"metre-{k}.hexrec") for k in range(1, METRE_RUNS + 1)]
        elif routine == "square":
            runs.append(Run("square", f"square {SQUARE_SIDE_M:g} m, TRI goals at speed {GOAL_SPEED:g}",
                            "square.hexrec"))
        elif routine == "circle":
            runs.append(Run("circle", f"circle radius {CIRCLE_RADIUS_M:g} m, TRI goal at speed {GOAL_SPEED:g}",
                            "circle.hexrec"))
        else:
            raise ValueError(f"no routine {routine!r}; the routines are {', '.join(ROUTINES)}")
    return runs


# --- on the robot ---------------------------------------------------------------------------------------------------
def require_deactivated(robot: Robot) -> None:
    mode = robot.state().mode
    if mode is not Mode.DEACTIVATED:
        raise Refused(f"the robot is {mode.name if mode is not None else 'in an unknown mode'}: switch it off "
                      "(DEACTIVATED) before the session, so the script takes over no robot someone drives")


def require_standing(robot: Robot) -> None:
    state = robot.state()
    if state.mode is not Mode.STAND:
        raise Refused(f"the robot reports {state.mode.name if state.mode is not None else 'no mode'}, not STAND")
    if state.handheld:
        raise Refused("the handheld controller is driving")
    if state.roll is not None and abs(state.roll) > MAX_ROLL_DEG:
        raise Refused(f"the body rolls {state.roll:+.1f} deg: the robot is not standing on the floor")


def _hold(robot: Robot, velocity: Velocity, seconds: float) -> None:
    robot.set_velocity(velocity.vx, velocity.vy, velocity.yaw_rate)
    robot.sleep(seconds)
    robot.stop()


def _goal(run: Run, status) -> None:
    run.goals.append(f"{status.kind.name} {status.state.name}")


def act(robot: Robot, run: Run) -> None:
    """The run's move, between the still stretches."""
    if run.routine == "rest":
        robot.sleep(REST_S)
    elif run.routine == "turn":
        model = velocity_of(Gait.TRI_GATE, Sticks(0.0, 0.0, run.stick["rx"]))
        _hold(robot, model, math.radians(run.stick["degrees"]) / abs(model.yaw_rate))
    elif run.routine == "tri":
        model = velocity_of(Gait.TRI_GATE, Sticks(0.0, run.stick["ly"], 0.0))
        _hold(robot, model, run.stick["metres"] / model.vx)
    elif run.routine == "metre":
        _goal(run, robot.walk(1.0))
    elif run.routine == "square":
        for _ in range(4):
            _goal(run, robot.walk(SQUARE_SIDE_M))
            _goal(run, robot.turn(90))
    elif run.routine == "circle":
        _goal(run, robot.arc(CIRCLE_RADIUS_M, 360))


FLOOR_QUESTIONS = {
    "turn": {"heading_deg": "heading change on the floor, counter-clockwise (deg)"},
    "tri": {"metres": "distance walked on the floor, start mark to end mark (m)"},
    "metre": {"metres": "distance walked on the floor, start mark to end mark (m)"},
    "square": {"closure_cm": "distance from the start mark to where the robot ended (cm)",
               "heading_deg": "heading error at the end against the start, counter-clockwise (deg)"},
    "circle": {"closure_cm": "distance from the start mark to where the robot ended (cm)",
               "heading_deg": "heading error at the end against the start, counter-clockwise (deg)"},
}


def execute(robot: Robot, operator, directory: Path, run: Run) -> None:
    operator.confirm(f"{run.label}: place the robot on its start mark, facing along the course, and clear the floor")
    run.battery_v = operator.number("battery voltage at the pack (V)")
    robot.stand()
    robot.sleep(STAND_S)
    try:
        require_standing(robot)
    except Refused as refusal:
        run.outcome = f"refused: {refusal}"
        raise
    if robot.takes_goals():
        robot.reset_odometry()
    # Ctrl-C and a lost link leave through run_session's `with robot`, which rests the robot as far as the link allows.
    run.outcome = "interrupted"
    try:
        with robot.recording(directory / run.file):
            robot.sleep(STILL_S)
            try:
                act(robot, run)
                run.outcome = "done"
            except GoalTimedOut as error:
                run.outcome = f"timed out: {error}"
            robot.sleep(STILL_S)
    except RobotDisconnected:
        run.outcome = "failed: the link was lost"
        raise
    except RobotError as error:
        run.outcome = f"failed: {error}"
    robot.rest()
    for name, question in FLOOR_QUESTIONS.get(run.routine, {}).items():
        value = operator.number(question)
        if value is not None:
            run.floor[name] = value


def run_session(robot: Robot, operator, directory: Path, runs: list[Run], target: str) -> Session:
    """Runs every run in turn and leaves the robot deactivated; the session file and the summary are written after
    each run, and on the way out of any error or Ctrl-C."""
    robot.connect()
    try:
        require_deactivated(robot)  # before any mode or stick: connecting sets only the gait
    except BaseException:
        robot.close()
        raise
    directory.mkdir(parents=True, exist_ok=True)
    session = Session(datetime.now().isoformat(timespec="seconds"), target, robot.features())
    with ExitStack() as leaving:
        leaving.push(robot)  # Robot.__exit__: rests the robot and closes the link, on an error and Ctrl-C too
        try:
            operator.confirm("the session starts: robot switched off on the floor, battery charged, camera running "
                             "if one is used")
            robot.style(gait=Gait.TRI_GATE, speed=GOAL_SPEED, stride=0.0)
            for run in runs:
                session.runs.append(run)
                execute(robot, operator, directory, run)
                session.save(directory)
        finally:
            session.save(directory)
            write_summary(directory, session)
    return session


# --- from the recordings --------------------------------------------------------------------------------------------
def held(ticks: dict[str, np.ndarray]) -> np.ndarray:
    """Per tick, whether the robot walked a stick: WALK with any of lx, ly, rx off centre."""
    return (ticks["mode"] == Mode.WALK) & np.any(ticks["command"][:, :3] != 0.0, axis=1)


def sent_sticks(run: Run) -> Sticks:
    """The sticks a velocity run sent: the backends invert the speed model's velocity for the stick asked for."""
    asked = Sticks(0.0, run.stick.get("ly", 0.0), run.stick.get("rx", 0.0))
    return sticks_for(Gait.TRI_GATE, velocity_of(Gait.TRI_GATE, asked))


def contention(ticks, sent: Sticks) -> str | None:
    """Why a velocity run's hold measured something else than the script's stick, or None: from the first tick whose
    command carries the sticks sent to the last one, every tick must carry that same command (sticks and posture), as
    the script re-sends it unchanged. Another client's frames show as other commands in between, the app's as
    centred sticks; a dead-man stop as ticks with the link lost."""
    command = ticks["command"]
    ours = np.all(np.abs(command[:, :3] - np.array([sent.lx, sent.ly, sent.rx])) < STICK_TOLERANCE, axis=1)
    if not ours.any():
        return "no tick carries the script's stick"
    first, last = np.flatnonzero(ours)[[0, -1]]
    window = slice(first, last + 1)
    lost = ticks["link_lost"][window].astype(bool) if "link_lost" in ticks else np.zeros(last + 1 - first, bool)
    other = ~np.all(command[window] == command[first], axis=1) & ~lost
    reasons = []
    if other.any():
        centred = np.all(command[window][other, :3] == 0.0, axis=1)
        reasons.append(f"{np.count_nonzero(other)} of {len(other)} ticks of the hold carried another command, "
                       f"{np.count_nonzero(centred)} of them centred sticks: another client (an open app) drove too")
    if lost.any():
        reasons.append(f"the dead-man stop held {np.count_nonzero(lost)} ticks of the hold")
    return "; ".join(reasons) or None


def held_seconds(ticks) -> float:
    """From the first held tick's start to the last one's end: a batch the link lost does not shorten it."""
    t = ticks["t_us"][held(ticks)]
    return float(t[-1] - t[0]) / 1e6 + TICK_S if len(t) else 0.0


def held_stick(ticks, axis: int) -> float:
    on = held(ticks)
    return float(np.median(ticks["command"][on, axis])) if on.any() else float("nan")


def fitted_rate(t: np.ndarray, angle: np.ndarray, window: np.ndarray) -> float:
    """The slope (per second) of the unwrapped `angle` over the samples in `window`."""
    if np.count_nonzero(window) < 2:
        return float("nan")
    return float(np.polyfit(t[window], np.unwrap(angle[window]), 1)[0])


def steady(ticks, fresh: np.ndarray | None = None) -> np.ndarray:
    """The held ticks after RATE_SKIP_S of holding, optionally only those with a fresh IMU sample."""
    on = held(ticks)
    start = ticks["t_us"][on][0] / 1e6 + RATE_SKIP_S if on.any() else math.inf
    window = on & (ticks["t_us"] / 1e6 >= start)
    return window if fresh is None else window & fresh


def dmp_yaw_rate(ticks) -> float:
    """The DMP's counter-clockwise yaw rate (deg/s) over the steady hold."""
    window = steady(ticks, ticks["imu_fresh"].astype(bool))
    return math.degrees(fitted_rate(ticks["t_us"] / 1e6, DMP_YAW_SIGN * ticks["rpy"][:, 2], window))


def odometry_yaw_rate(ticks) -> float:
    if "odom_yaw" not in ticks:
        return float("nan")
    return math.degrees(fitted_rate(ticks["t_us"] / 1e6, ticks["odom_yaw"], steady(ticks)))


def odometry_distance(ticks) -> float:
    return float(ticks["odom_distance"][-1] - ticks["odom_distance"][0]) if "odom_distance" in ticks else math.nan


def odometry_closure(ticks) -> tuple[float, float]:
    """The odometry's distance (m) and heading (deg, wrapped) from the pose it was reset to, at the end."""
    if "odom_x" not in ticks:
        return math.nan, math.nan
    yaw = math.degrees(float(ticks["odom_yaw"][-1]))
    return float(math.hypot(ticks["odom_x"][-1], ticks["odom_y"][-1])), (yaw + 180.0) % 360.0 - 180.0


def camera(rec: recording.Recording, track: score_track.Track | None) -> score_track.Stretch | str | None:
    """The odometry scored against the camera's track, or why it could not be."""
    if track is None:
        return None
    try:
        result = score_track.score(track, score_track.Telemetry.from_recording(rec))
    except ValueError as error:
        return str(error)
    return result.stretches.get("odometry", "the track does not see the recording's start")


def _f(value, digits: int = 3) -> str:
    return "-" if value is None or (isinstance(value, float) and math.isnan(value)) else f"{value:.{digits}f}"


def goals(run: Run) -> str:
    """How the run's goals ended, as "7 REACHED, 1 TIMED_OUT"."""
    ends = Counter(goal.split()[-1] for goal in run.goals)
    return ", ".join(f"{count} {state}" for state, count in ends.items()) or "-"


def firmware(session: Session) -> str:
    if session.features.get("backend") == "simulation":
        return f"simulation {session.features.get('firmwareVersion', '?')}"
    return f"{session.features.get('firmwareVersion', '?')} `{session.features.get('firmwareElfSha256', '')[:12]}`"


def _health(rec: recording.Recording) -> list[str]:
    timing = ts.tick_timing(rec)
    stops, writes = ts.dead_man_stops(rec), ts.servo_writes(rec)
    failed = "-" if writes is None else str(sum(board.failed for board in writes.boards.values()))
    return [_f(timing.period_p99_us, 0), str(timing.overruns), "-" if stops is None else str(len(stops.fired_seqs)),
            failed]


HEALTH = "Period p99 (us) | Overruns | Dead-man stops | Failed writes"


def summary(directory: Path, session: Session, track: score_track.Track | None = None) -> str:
    fw = firmware(session)
    lines = [f"# Floor session {session.started}", "",
             f"Target `{session.target}`, firmware {fw}; battery measured at the pack before each run.",
             "Speeds are distances over the time the stick was held; the model is `speed_model.json` at the stick "
             "sent.", ""]
    loaded = {}  # the recordings with telemetry: a link lost or Ctrl-C right after one opened leaves none
    for run in session.runs:
        path = directory / run.file
        if path.exists() and len((rec := recording.load(path)).ticks["t_us"]):
            loaded[run.file] = rec
    battery = lambda run: _f(run.battery_v, 2)  # noqa: E731
    # A run standing still has no start motion to align a track on.
    scores = {run.file: camera(loaded[run.file], track) for run in session.runs
              if run.file in loaded and run.routine != "rest"}

    speed_rows, turn_rows, walk_rows, closure_rows, invalid = [], [], [], [], []
    for run in session.runs:
        rec = loaded.get(run.file)
        if rec is None or run.routine == "rest":
            continue
        t = rec.ticks
        cam = scores[run.file] if isinstance(scores[run.file], score_track.Stretch) else None
        if run.routine in ("tri", "turn"):
            reason = contention(t, sent_sticks(run))
            if reason is not None:
                invalid.append(f"- {run.label}: {reason}")
            outcome = run.outcome if reason is None else f"INVALID: {reason}"
        if run.routine == "tri":
            seconds, stick = held_seconds(t), held_stick(t, 1)
            model = velocity_of(Gait.TRI_GATE, Sticks(0.0, stick, 0.0)).vx if not math.isnan(stick) else math.nan
            # An invalid run's measured columns stay empty: it did not walk the script's stick alone.
            valid = reason is None and seconds > 0
            per_s = lambda metres: metres / seconds if valid and metres is not None else math.nan  # noqa: E731
            speed_rows.append([run.label, fw, battery(run), _f(stick, 2), _f(seconds, 2), _f(model),
                               _f(per_s(odometry_distance(t))), _f(per_s(run.floor.get("metres"))),
                               _f(per_s(cam.track_made_good) if cam else None), *_health(rec), outcome])
        elif run.routine == "turn":
            seconds, stick = held_seconds(t), held_stick(t, 2)
            model = (math.degrees(velocity_of(Gait.TRI_GATE, Sticks(0.0, 0.0, stick)).yaw_rate)
                     if not math.isnan(stick) else math.nan)
            floor = run.floor.get("heading_deg")
            turn_rows.append([run.label, fw, battery(run), _f(stick, 2), _f(seconds, 2), _f(model, 1),
                              *(_f(rate, 1) if reason is None else "-" for rate in (
                                  dmp_yaw_rate(t), odometry_yaw_rate(t),
                                  floor / seconds if floor is not None and seconds > 0 else None)),
                              *_health(rec), outcome])
        elif run.routine == "metre":
            walked = odometry_closure(t)[0]
            walk_rows.append([run.label, fw, battery(run), goals(run), _f(walked), _f(run.floor.get("metres")),
                              _f(cam.track_made_good) if cam else "-", _f(100 * cam.position_end, 1) if cam else "-",
                              *_health(rec), run.outcome])
        else:
            distance, heading = odometry_closure(t)
            floor_cm = run.floor.get("closure_cm")
            closure_rows.append([run.label, fw, battery(run), goals(run), _f(100 * distance, 1),
                                 _f(heading, 1), _f(floor_cm, 1), _f(run.floor.get("heading_deg"), 1),
                                 _f(100 * cam.track_made_good, 1) if cam else "-",
                                 _f(100 * cam.position_end, 1) if cam else "-", _f(cam.yaw_end, 1) if cam else "-",
                                 *_health(rec), run.outcome])

    def table(header: str, rows: list[list[str]]) -> list[str]:
        columns = header.count("|") + 1
        return [f"| {header} |", "|" + "---|" * columns, *(f"| {' | '.join(row)} |" for row in rows), ""]

    if speed_rows:
        lines += ["## Forward speed", ""]
        lines += table("Run | Firmware | Battery (V) | Stick ly | Held (s) | Model (m/s) | Odometry (m/s) | "
                       f"Floor (m/s) | Camera (m/s) | {HEALTH} | Outcome", speed_rows)
    if turn_rows:
        lines += ["## Turn rate", "", "Counter-clockwise positive; the DMP's and the odometry's rates are fitted over "
                  f"the hold after its first {RATE_SKIP_S:g} s.", ""]
        lines += table("Run | Firmware | Battery (V) | Stick rx | Held (s) | Model (deg/s) | DMP (deg/s) | "
                       f"Odometry (deg/s) | Floor (deg/s) | {HEALTH} | Outcome", turn_rows)
    if walk_rows:
        lines += ["## Goal walks", "", "Where a 1 m goal walk ended, start to end in a straight line.", ""]
        lines += table(f"Run | Firmware | Battery (V) | Goal | Odometry (m) | Floor (m) | Camera (m) | "
                       f"Odometry vs camera (cm) | {HEALTH} | Outcome", walk_rows)
    if closure_rows:
        lines += ["## Goal closure", "", "Where the robot ended against its start; the camera's columns are the "
                  "track's closure, and the odometry's position and heading error against the track at the end.", ""]
        lines += table("Run | Firmware | Battery (V) | Goals | Odometry (cm) | Odometry heading (deg) | Floor (cm) | "
                       f"Floor heading (deg) | Camera (cm) | Odometry vs camera (cm) | Heading vs camera (deg) | "
                       f"{HEALTH} | Outcome", closure_rows)
    if invalid:
        lines += ["Invalid runs: the robot did not walk the script's stick alone, so their speeds and rates are not "
                  "measurements; run them again with every app closed and the handheld off.", "", *invalid, ""]
    lines += _rest_table(session, loaded)
    unscored = [f"- {run.label}: {scores[run.file]}" for run in session.runs
                if isinstance(scores.get(run.file), str)]
    if unscored:
        lines += ["Not scored against the camera:", "", *unscored, ""]
    lines += ["## Recordings", ""]
    for run in session.runs:
        lines += [f"### {run.label}", "", f"`{run.file}`, {run.outcome}", ""]
        if run.file in loaded:
            lines += ["```", report_recording.report(directory / run.file), "```", ""]
        elif (directory / run.file).exists():
            lines += ["No telemetry was recorded.", ""]
    return "\n".join(lines)


def _rest_table(session: Session, loaded: dict) -> list[str]:
    rows = []
    for run in session.runs:
        if run.routine != "rest" or run.file not in loaded:
            continue
        imu = ts.imu_at_rest(loaded[run.file])
        timing = ts.tick_timing(loaded[run.file])
        vec = lambda v, scale=1.0: ", ".join(_f(scale * x, 4) for x in v)  # noqa: E731
        rows += [f"| Control period | mean {_f(timing.period_mean_us, 0)} us, p99 {_f(timing.period_p99_us, 0)} us, "
                 f"{timing.overruns} overruns | {ts.CONTROL_PERIOD_US} us |",
                 f"| Fresh IMU samples | {imu.samples} | |",
                 f"| Gyro bias (rad/s) | {vec(imu.gyro_bias)} | N(0, {report_recording.SIM_GYRO_BIAS_SIGMA}) |",
                 f"| Gyro noise (rad/s) | {vec(imu.gyro_noise)} | {report_recording.SIM_GYRO_NOISE} |",
                 f"| Roll, pitch, yaw noise (deg) | {vec(imu.rpy_noise_deg)} | {report_recording.SIM_RPY_NOISE_DEG} |",
                 f"| Gravity noise | {vec(imu.gravity_noise)} | {report_recording.SIM_GRAVITY_NOISE} |"]
    if not rows:
        return []
    return ["## At rest", "", "Measured per axis against what `domain_rand.py` draws.", "",
            "| Quantity | Measured | Simulation |", "|---|---|---|", *rows, ""]


def write_summary(directory: Path, session: Session, track: score_track.Track | None = None) -> Path:
    path = directory / "summary.md"
    path.write_text(summary(directory, session, track) + "\n")
    return path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    commands = parser.add_subparsers(dest="command", required=True)
    run = commands.add_parser("run", help="run routines on the robot or the simulation")
    where = run.add_mutually_exclusive_group(required=True)
    where.add_argument("--port", help="the robot's native USB port, e.g. COM5")
    where.add_argument("--host", help="the robot's address, e.g. 192.168.1.151 or hexapod.local")
    where.add_argument("--sim", action="store_true", help="the MuJoCo simulation, with no questions asked")
    run.add_argument("--out", type=Path, help="output directory (default floor-<time>)")
    run.add_argument("--distance", type=float, default=DISTANCE_M, help="metres each tri walk covers by the model")
    run.add_argument("--turn-deg", type=float, default=TURN_DEG, help="degrees the turn covers by the model")
    run.add_argument("routines", nargs="*", default=list(ROUTINES), help=f"any of {', '.join(ROUTINES)}")
    report = commands.add_parser("report", help="write summary.md again from a session directory")
    report.add_argument("directory", type=Path)
    report.add_argument("--track", help="an overhead camera's track of the session (ground_truth.py)")
    args = parser.parse_args()

    if args.command == "report":
        track = score_track.Track.load(args.track) if args.track else None
        print(f"wrote {write_summary(args.directory, Session.load(args.directory), track)}")
        return 0
    target = "simulation" if args.sim else args.port or args.host
    directory = args.out or Path(f"floor-{datetime.now():%Y%m%d-%H%M%S}")
    try:
        runs = plan(args.routines, args.distance, args.turn_deg)
        run_session(Robot(target), Scripted() if args.sim else Terminal(), directory, runs, target)
    except Refused as refusal:
        print(f"refused: {refusal}")
        return 2
    except RobotError as error:
        print(f"stopped: {error}")
        return 2
    except KeyboardInterrupt:
        print("interrupted: a robot the session stood was lowered and switched off as far as the link allowed; check "
              "that its legs are limp")
        return 130
    finally:
        if (directory / "summary.md").exists():
            print(f"wrote {directory / 'summary.md'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
