"""The real hexapod over its WebSocket (ws://<host>/api/ws) or its native USB serial port (COMx, /dev/tty*), in
platform_shared/message.proto frames; both links carry the same messages, so only the transport differs.

One thread runs an asyncio loop that owns the link; the public methods are synchronous. The robot answers a subscribe
to a state topic (mode, gait, goal status) with its current value, the snapshot, and then the subscribe echoed as its
ack, and pushes every change after; it streams its IMU angles at up to 40 Hz. A firmware from before the snapshots
acks nothing: the mode is then unknown until the first push, and the backend neither refreshes it nor takes goals.

Keep-alive: while the robot is in a mode the app heartbeats (STAND, WALK, WALK_NN; app/src/lib/control-link.ts), the
last input is re-sent every KEEPALIVE_S. The firmware drops a walk command after 600 ms without input while a stick
is deflected, and zeroes the whole command after 2 s otherwise (motion.h STEERING_TIMEOUT_MS, COMMAND_TIMEOUT_MS).
Every input carries the style's posture and stride, the last goal's once one was sent, so the robot keeps them. Not
its speed: a goal ignores centred commands, so the speed slider would reach only the velocity walks, and the speed
model was measured at the slider's 0. A goal is cancelled by any stick off centre, and by 2 s without a frame from the
client that sent it, so the keep-alive goes on through it with centred sticks.

Handheld: while the ESP-NOW handheld controller drives, the robot drops this client's sticks and refuses its goals
(423), and it reports so with its mode (ModeData.handheld). When that report rises, the keep-alive's sticks are
centred, so the robot stays stopped once the handheld lets go, and a walk in progress raises a RobotError at its next
sleep or set_velocity, or at the next goal_move; a goal ends CANCELLED. A walk while the handheld drives is refused at
once. After the handheld, the robot ignores this client's sticks until it has sent a centred one, so the first stick
after a takeover, and the first of a connection, is sent after a centred input. The report is pushed only on a change,
and a push can be dropped, so while the keep-alive runs or the handheld drives the mode is subscribed to again every
REFRESH_S, which the robot answers with the current mode; the snapshot follows every push sent before it and precedes
every push after, so it is never older than the mode last heard.

Goals: a firmware takes them when it answers the subscribe to the goal status with a snapshot before the ack, which
connect() waits for. The robot pushes a goal's status on each change, but a push can be dropped, so the status is
subscribed to again, for its snapshot, when none came for CONFIRM_S and to confirm the end a push reported.

Dead-man detection: the firmware reports that timeout only in its per-tick telemetry (TickSample.link_lost), which this
client does not subscribe to, so the client bounds it. Every input is followed by a ping on the same socket, and the
firmware answers pings in order, so input k reached it no earlier than it was sent
(s_k) and no later than the pong that followed it arrived (a_k). The silence it saw after a walking input k-1 is
therefore at most a_k - s_{k-1}; past STEERING_TIMEOUT_S the walk may have been dropped, and link_drops counts it.
The bound is conservative: a slow but unbroken link can count a drop that did not happen, never miss one that did
(up to the firmware's own EventBus hand-off from the socket to MotionService).
"""
import asyncio
import collections
import math
import threading
import time
from dataclasses import replace

import websockets
from google.protobuf.message import DecodeError

from .. import speed_model
from .base import (Gait, Goal, GoalStatus, Mode, Odometry, RobotDisconnected, RobotError, RobotState, RobotTimeout,
                   Style, Velocity)
from .messages import pb
from .serial_link import READY_TIMEOUT_S, SerialLink, is_serial_port

KEEPALIVE_S = 0.2  # the app's COMMAND_HEARTBEAT_MS
STEERING_TIMEOUT_S = 0.6
HEARTBEAT_MODES = (Mode.STAND, Mode.WALK, Mode.WALK_NN)
WALKING_MODES = (Mode.WALK, Mode.WALK_NN)
REPLY_TIMEOUT_S = 3.0
MODE_WAIT_S = 1.0  # fresh_mode's wait for the mode's snapshot, on the way out of a script
SILENCE_S = 2.0  # pongs arrive every KEEPALIVE_S: this long without a frame means the link is gone
CONFIRM_S = 1.0
# Under the handheld's 600 ms window (motion_inbox.h HANDHELD_PRIORITY_MS), so a refresh falls inside every drive.
REFRESH_S = 0.4
RESET_POLL_S = 0.02
ODOMETRY_ZERO = 1e-4  # m and rad: what a reset pose reads, a still robot's tick having moved it by less
# The DMP yaw (mpu6050.h dmpGetYawPitchRoll) is atan2(2(xy - wz), 2(w^2 + x^2) - 1), clockwise-positive about the
# chip's Z axis, and the chip's Z points up (its gravity vector reads world-up at rest, peripherals.h).
IMU_YAW_SIGN = -1.0


def _tag(name: str) -> int:
    return pb.Message.DESCRIPTOR.fields_by_name[name].number


def _goal_status(s: pb.GoalStatus) -> GoalStatus:
    return GoalStatus.from_body(s.id, s.state, s.kind, s.target, s.progress, s.x, s.y, s.yaw, s.passes)


class WebSocketTransport:
    def __init__(self, host: str):
        self._url = f"ws://{host}/api/ws"
        self.name = f"the robot at {self._url}"
        self._socket = None

    async def open(self) -> None:
        self._socket = await websockets.connect(self._url, open_timeout=REPLY_TIMEOUT_S, close_timeout=1)

    async def send(self, message: pb.Message) -> None:
        try:
            await self._socket.send(message.SerializeToString())
        except websockets.ConnectionClosed as error:
            raise RobotDisconnected(f"{self.name} closed the connection") from error

    async def recv(self) -> pb.Message:
        try:
            frame = await self._socket.recv()
        except websockets.ConnectionClosed as error:
            raise RobotDisconnected(f"{self.name} closed the connection") from error
        return pb.Message.FromString(frame)

    async def close(self) -> None:
        if self._socket is not None:
            await self._socket.close()


class SerialTransport:
    """SerialLink's reader thread hands each message, and the link's failure, to the loop."""

    def __init__(self, port: str):
        self._port = port
        self.name = f"the robot on {port}"
        self._link: SerialLink | None = None
        self._messages: asyncio.Queue[pb.Message | RobotDisconnected] | None = None

    async def open(self) -> None:
        loop = asyncio.get_running_loop()
        self._messages = asyncio.Queue()

        def deliver(item: pb.Message | RobotDisconnected) -> None:
            loop.call_soon_threadsafe(self._messages.put_nowait, item)

        self._link = await asyncio.to_thread(SerialLink, self._port, deliver, deliver)
        await asyncio.to_thread(self._link.wait_ready)

    async def send(self, message: pb.Message) -> None:
        self._link.send(message)

    async def recv(self) -> pb.Message:
        item = await self._messages.get()
        if isinstance(item, RobotDisconnected):
            raise item
        return item

    async def close(self) -> None:
        if self._link is not None:
            self._link.close()


def _known_mode(value: int) -> Mode | None:
    """The mode a reported value is, or None for one this facade has none of, such as POSE's 2 from older firmware:
    unknown, so no heartbeat keeps a walk alive in it."""
    try:
        return Mode(value)
    except ValueError:
        return None


class RealBackend:
    def __init__(self, target: str):
        self._transport = SerialTransport(target) if is_serial_port(target) else WebSocketTransport(target)
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._loop.run_forever, daemon=True)
        self._tasks: list[asyncio.Task] = []
        self._changed = threading.Condition()
        self._error: RobotError | None = None
        self._replies: dict[int, asyncio.Future] = {}
        self._next_id = 1
        self._imu = False
        self._mode: Mode | None = None
        self._gait: Gait | None = None
        self._yaw: float | None = None
        self._sticks = speed_model.CENTRED
        self._handheld = False
        self._snapshots = False  # the firmware answers a subscribe with the current value and an ack
        # Per tag: the frames of it heard, and the subscribes awaiting their ack with that count when they went out.
        self._heard: collections.Counter[int] = collections.Counter()
        self._acks: dict[int, list[tuple[asyncio.Future, int]]] = {}
        self._taken_over = False  # the handheld took over a walk, which the script has not been told of yet
        self._centre_first = True  # the robot may be waiting for this client to send a centred stick
        self._style = Style()  # the posture and stride every input carries: set_height's height, then each goal's
        self._goals = False
        self._goal: GoalStatus | None = None  # the newest status heard, pushed or asked for, and when
        self._goal_heard = 0.0
        self._drops = 0
        # Per ping in flight, after an input: when the input before that one was sent, and whether it walked.
        self._pings: collections.deque[tuple[float, bool] | None] = collections.deque()
        self._last_input: tuple[float, bool] | None = None

    # --- connection --------------------------------------------------------------------------------------------
    # A connect that fails partway closes what it opened: the robot would otherwise keep streaming to a dead client.
    def connect(self) -> None:
        self._thread.start()
        try:
            acks = self._run(self._open(), READY_TIMEOUT_S + 1)
            features = self._run(self._ask(pb.CorrelationRequest(features_data_request=pb.FeaturesDataRequest())))
            self._imu = features.features_data_response.imu
            mode_acked = self._run(self._acked(acks["mode"])) is not None
            # A firmware from before the snapshots acks none of these; one ack lost on the way is no such firmware.
            self._snapshots = mode_acked or any(ack.done() for ack in acks.values())
            self._goals = self._snapshots and bool(self._run(self._acked(acks["goal_status"])))
            if self._imu:
                self._wait(lambda: self._yaw is not None, "the robot reports an IMU but sends no angles")
        except BaseException:
            self.close()
            raise

    def close(self) -> None:
        if not self._thread.is_alive():
            return
        try:
            asyncio.run_coroutine_threadsafe(self._shutdown(), self._loop).result(2.0)
        except (TimeoutError, OSError, RobotError):
            pass  # already gone
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(2.0)

    async def _shutdown(self) -> None:
        for task in self._tasks:
            task.cancel()
        await asyncio.gather(*self._tasks, return_exceptions=True)
        await self._transport.close()

    async def _open(self) -> dict[str, asyncio.Future]:
        await self._transport.open()
        acks = {name: await self._subscribe(name) for name in ("mode", "gait", "imu", "goal_status")}
        self._tasks = [self._loop.create_task(task()) for task in (self._read, self._keep_alive, self._refresh_mode)]
        return acks

    # The future resolves on the subscribe's ack to whether a value of the topic, its snapshot, came before it.
    async def _subscribe(self, name: str) -> asyncio.Future:
        tag = _tag(name)
        ack = self._loop.create_future()
        with self._changed:
            self._acks.setdefault(tag, []).append((ack, self._heard[tag]))
        await self._send(pb.Message(sub_notif=pb.SubscribeNotification(tag=tag)))
        return ack

    # None when no ack came within the timeout, as from a firmware from before the snapshots.
    async def _acked(self, ack: asyncio.Future, timeout: float | None = None) -> bool | None:
        try:
            return await asyncio.wait_for(asyncio.shield(ack), timeout or REPLY_TIMEOUT_S)
        except TimeoutError:
            return None

    async def _read(self) -> None:
        try:
            while True:
                self._receive(await asyncio.wait_for(self._transport.recv(), SILENCE_S), time.monotonic())
        except TimeoutError:
            reason = f"nothing from {self._transport.name} for {SILENCE_S:g} s"
        except RobotDisconnected as error:
            reason = str(error)
        except (DecodeError, ValueError) as error:  # a frame or enum value from a firmware this client predates
            reason = f"{self._transport.name} sent a frame this client cannot read: {error}"
        with self._changed:
            self._error = RobotDisconnected(reason)
            self._changed.notify_all()
        for future in self._replies.values():
            if not future.done():
                future.set_exception(self._error)

    def _receive(self, message: pb.Message, arrived: float) -> None:
        kind = message.WhichOneof("message")
        if kind == "correlation_response":
            reply = message.correlation_response
            if reply.status_code < 400 and reply.WhichOneof("response") == "goal_status":
                with self._changed:
                    self._hear_goal(reply.goal_status, arrived)
            future = self._replies.pop(reply.correlation_id, None)
            if future is not None and not future.done():
                future.set_result(reply)
            return
        with self._changed:
            if kind == "sub_notif":
                tag = message.sub_notif.tag
                for ack, heard in self._acks.pop(tag, []):
                    if not ack.done():
                        ack.set_result(self._heard[tag] > heard)
                return
            if kind in ("mode", "gait", "goal_status"):
                self._heard[_tag(kind)] += 1
            if kind == "goal_status":
                self._hear_goal(message.goal_status, arrived)
            elif kind == "mode":
                self._mode = _known_mode(message.mode.mode)
                self._hear_handheld(message.mode.handheld)
            elif kind == "gait":
                self._gait = Gait(message.gait.gait)
            elif kind == "imu":
                self._yaw = IMU_YAW_SIGN * math.degrees(message.imu.z)
            elif kind == "pongmsg" and self._pings:
                previous = self._pings.popleft()
                if previous is not None and previous[1] and arrived - previous[0] > STEERING_TIMEOUT_S:
                    self._drops += 1
            self._changed.notify_all()

    # The robot drops the sticks of a walk the handheld took over and walks none of them after it, so they are centred.
    def _hear_handheld(self, driving: bool) -> None:
        if driving and not self._handheld:
            self._taken_over = self._sticks != speed_model.CENTRED
            self._sticks = speed_model.CENTRED
            self._centre_first = True
        self._handheld = driving

    def _raise_if_taken_over(self) -> None:
        if self._taken_over:
            self._taken_over = False
            raise RobotError("the handheld controller took over: the robot dropped this script's walk")

    # Pushes and replies are handled in the order they arrived, so the last heard is the newest.
    def _hear_goal(self, status: pb.GoalStatus, arrived: float) -> None:
        self._goal, self._goal_heard = _goal_status(status), arrived

    # Each refresh goes out REFRESH_S after the last, its snapshot heard or not, so a lost one stretches nothing; the
    # snapshot comes in as a push does.
    async def _refresh_mode(self) -> None:
        while self._error is None:
            await asyncio.sleep(REFRESH_S)
            with self._changed:
                due = self._snapshots and (self._handheld or self._mode in HEARTBEAT_MODES)
            if due:
                try:
                    await self._send(pb.Message(sub_notif=pb.SubscribeNotification(tag=_tag("mode"))))
                except RobotError:
                    pass  # the reader reports a link that is gone

    async def _keep_alive(self) -> None:
        while self._error is None:
            await asyncio.sleep(KEEPALIVE_S)
            await self._send_input(heartbeat=True)

    # The input and the ping after it go out back to back on the loop's thread, so nothing interleaves.
    async def _send_input(self, heartbeat: bool = False) -> None:
        with self._changed:
            s, style, mode = self._sticks, self._style, self._mode
            sending = not heartbeat or mode in HEARTBEAT_MODES
            centre_first = sending and self._centre_first and s != speed_model.CENTRED
            # Sent while the handheld drives, a centred stick is dropped too, so it is no end of the robot's wait.
            if sending and not self._handheld:
                self._centre_first = False
        previous = None
        try:
            if sending:
                def input_of(sticks):
                    return pb.ControllerInputData(left=pb.Vector(x=sticks.lx, y=sticks.ly),
                                                  right=pb.Vector(x=sticks.rx, y=0.0), height=style.h, s1=style.s1,
                                                  feet_distance=style.fd, stride=style.stride)

                if centre_first:
                    await self._send(pb.Message(input=input_of(speed_model.CENTRED)))
                await self._send(pb.Message(input=input_of(s)))
                previous = self._last_input
                self._last_input = time.monotonic(), mode in WALKING_MODES and s != speed_model.CENTRED
            else:
                self._last_input = None  # the robot left the heartbeat modes: its walk ended by a mode, not the link
            await self._send(pb.Message(pingmsg=pb.PingMsg()))
            self._pings.append(previous)
        except RobotError:
            pass  # the reader reports a link that is gone, or one that stays silent past SILENCE_S

    async def _send(self, message: pb.Message) -> None:
        await self._transport.send(message)

    async def _ask(self, request: pb.CorrelationRequest, check: bool = True) -> pb.CorrelationResponse:
        request.correlation_id = self._next_id
        self._next_id += 1
        future = self._loop.create_future()
        self._replies[request.correlation_id] = future
        try:
            await self._send(pb.Message(correlation_request=request))
            reply = await asyncio.wait_for(future, REPLY_TIMEOUT_S)
        finally:
            self._replies.pop(request.correlation_id, None)  # a lost reply leaves no entry behind
        if check and reply.status_code >= 400:
            reason = f": {reply.error_message}" if reply.error_message else ""
            raise RobotError(f"the robot refused the request with status {reply.status_code}{reason}")
        return reply

    # --- synchronous helpers -----------------------------------------------------------------------------------
    def _run(self, coroutine, timeout: float | None = None):
        if self._error is not None:
            coroutine.close()
            raise self._error
        try:
            return asyncio.run_coroutine_threadsafe(coroutine, self._loop).result(timeout or REPLY_TIMEOUT_S + 1)
        except (TimeoutError, OSError) as error:
            raise RobotTimeout(f"no answer from {self._transport.name}") from error

    def _wait(self, done, what: str) -> None:
        deadline = time.monotonic() + REPLY_TIMEOUT_S
        with self._changed:
            while not done():
                if self._error is not None:
                    raise self._error
                left = deadline - time.monotonic()
                if left <= 0:
                    raise RobotTimeout(what)
                self._changed.wait(left)

    # --- the backend interface ---------------------------------------------------------------------------------
    # main.cpp's mode bridge pushes a mode only once the control task has applied it, so a goal sent after the report
    # finds the robot in the mode; one its arbiter defers (leaving a busy animation) is reported when it applies.
    def set_mode(self, mode: Mode) -> None:
        self._run(self._send(pb.Message(mode=pb.ModeData(mode=int(mode)))))
        self._wait(lambda: self._mode is mode, f"the robot did not report {mode.name}")

    def set_gait(self, gait: Gait) -> None:
        self._run(self._send(pb.Message(gait=pb.GaitData(gait=int(gait)))))
        self._wait(lambda: self._gait is gait, f"the robot did not report {gait.name}")

    def set_height(self, height: float) -> None:
        with self._changed:
            self._style = replace(self._style, h=height)
        self._run(self._send_input())

    # A stop always passes, and ends a walk the handheld took over without an error.
    def set_velocity(self, velocity: Velocity) -> Velocity:
        sticks = speed_model.sticks_for(self._gait, velocity)
        with self._changed:
            if sticks == speed_model.CENTRED:
                self._taken_over = False
            else:
                self._raise_if_taken_over()
                if self._handheld:
                    raise RobotError("the handheld controller is driving: the robot drops this script's sticks")
            self._sticks = sticks
        self._run(self._send_input())
        return speed_model.velocity_of(self._gait, sticks)

    def max_velocity(self) -> Velocity:
        return speed_model.max_velocity(self._gait)

    def state(self) -> RobotState:
        with self._changed:
            if self._error is not None:
                raise self._error
            return RobotState(t=time.monotonic(), mode=self._mode, gait=self._gait,
                              yaw=self._yaw if self._imu else None, link_drops=self._drops, handheld=self._handheld)

    # Subscribes to the mode again, for its snapshot. A firmware from before the snapshots answers nothing but pushes
    # every change, so the mode last heard is then as current as any. From a firmware that answers subscribes, an ack
    # without the snapshot means the snapshot was lost, and no answer within MODE_WAIT_S that the link stalls, which
    # holds back the pushes too: either way the mode last heard may be the very one the robot has left.
    def fresh_mode(self) -> Mode | None:
        async def ask() -> bool | None:
            return await self._acked(await self._subscribe("mode"), MODE_WAIT_S)

        snapshot = self._run(ask(), MODE_WAIT_S + 1)
        with self._changed:
            if snapshot is False or (snapshot is None and self._snapshots):
                return None
            return self._mode

    def now(self) -> float:
        return time.monotonic()

    def supports_goals(self) -> bool:
        return self._goals

    def goal_move(self, goal: Goal, style: Style) -> GoalStatus:
        with self._changed:
            self._raise_if_taken_over()
            self._sticks = speed_model.CENTRED
        self._run(self._send_input())
        kind, distance, angle, radius = goal.in_body_frame()
        move = pb.GoalMove(kind=int(kind), distance_m=distance, angle_rad=angle, radius_m=radius, speed=style.speed,
                           gait=int(style.gait), stride=style.stride, s1=style.s1, h=style.h, fd=style.fd)
        status = _goal_status(self._run(self._ask(pb.CorrelationRequest(goal_move=move))).goal_status)
        with self._changed:
            self._style = style
        return status

    def goal_cancel(self) -> None:
        self._run(self._ask(pb.CorrelationRequest(goal_cancel=pb.GoalCancel())))

    def goal_status(self) -> GoalStatus:
        with self._changed:
            if self._error is not None:
                raise self._error
            heard, at = self._goal, self._goal_heard
        if heard is not None and not heard.ended and time.monotonic() - at < CONFIRM_S:
            return heard
        return self._run(self._goal_snapshot())

    # Subscribes to the goal status again: the robot answers with the current status before the ack.
    async def _goal_snapshot(self) -> GoalStatus:
        if await self._acked(await self._subscribe("goal_status")) is None:
            raise RobotTimeout(f"{self._transport.name} did not answer for the goal status")
        with self._changed:
            return self._goal

    def odometry(self) -> Odometry:
        o = self._read_odometry()
        return Odometry.from_body(o.x, o.y, o.yaw, o.kinematic_yaw, o.distance)

    def _read_odometry(self) -> pb.OdometryData:
        return self._run(self._ask(pb.CorrelationRequest(odometry_get=pb.OdometryGet()))).odometry

    def _odometry_zeroed(self) -> bool:
        o = self._read_odometry()
        return max(abs(o.x), abs(o.y), abs(o.yaw)) <= ODOMETRY_ZERO

    # The control task zeroes the pose on its next tick, after the reply.
    def reset_odometry(self) -> None:
        self._run(self._ask(pb.CorrelationRequest(odometry_get=pb.OdometryGet(reset=True))))
        deadline = time.monotonic() + REPLY_TIMEOUT_S
        while not self._odometry_zeroed():
            if time.monotonic() > deadline:
                raise RobotTimeout(f"the odometry of {self._transport.name} did not read its reset pose")
            self.sleep(RESET_POLL_S)

    def sleep(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        with self._changed:
            while (left := deadline - time.monotonic()) > 0:
                if self._error is not None:
                    raise self._error
                self._raise_if_taken_over()
                self._changed.wait(left)
            if self._error is not None:
                raise self._error
            self._raise_if_taken_over()
