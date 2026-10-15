"""The real hexapod over its WebSocket (ws://<host>/api/ws) or its native USB serial port (COMx, /dev/tty*), in
platform_shared/message.proto frames; both links carry the same messages, so only the transport differs.

One thread runs an asyncio loop that owns the link; the public methods are synchronous. Once subscribed, the robot
pushes its mode and gait on every change and its IMU angles at up to 40 Hz. It has no request for its current mode,
so the mode is unknown until the first push or set_mode.

Keep-alive: while the robot is in a mode the app heartbeats (STAND, WALK, WALK_NN; app/src/lib/control-link.ts), the
last input is re-sent every KEEPALIVE_S. The firmware drops a walk command after 600 ms without input while a stick
is deflected, and zeroes the whole command after 2 s otherwise (motion.h STEERING_TIMEOUT_MS, COMMAND_TIMEOUT_MS).

Dead-man detection: the firmware does not report that timeout, so the client bounds it. Every input is followed by a
ping on the same socket, and the firmware answers pings in order, so input k reached it no earlier than it was sent
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

import websockets
from google.protobuf.message import DecodeError

from .. import speed_model
from .base import Gait, Mode, RobotDisconnected, RobotError, RobotState, RobotTimeout, Velocity
from .messages import pb
from .serial_link import READY_TIMEOUT_S, SerialLink, is_serial_port

KEEPALIVE_S = 0.2  # the app's COMMAND_HEARTBEAT_MS
STEERING_TIMEOUT_S = 0.6
HEARTBEAT_MODES = (Mode.STAND, Mode.WALK, Mode.WALK_NN)
WALKING_MODES = (Mode.WALK, Mode.WALK_NN)
REPLY_TIMEOUT_S = 3.0
SILENCE_S = 2.0  # pongs arrive every KEEPALIVE_S: this long without a frame means the link is gone
# The DMP yaw (mpu6050.h dmpGetYawPitchRoll) is atan2(2(xy - wz), 2(w^2 + x^2) - 1), clockwise-positive about the
# chip's Z axis, and the chip's Z points up (its gravity vector reads world-up at rest, peripherals.h).
IMU_YAW_SIGN = -1.0


def _tag(name: str) -> int:
    return pb.Message.DESCRIPTOR.fields_by_name[name].number


class WebSocketTransport:
    def __init__(self, host: str):
        self._url = f"ws://{host}/api/ws"
        self.name = f"the robot at {self._url}"
        self._socket = None

    async def open(self) -> None:
        self._socket = await websockets.connect(self._url, open_timeout=REPLY_TIMEOUT_S, close_timeout=1,
                                                ping_interval=None)

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
        self._height = 0.0
        self._drops = 0
        # Per ping in flight, after an input: when the input before that one was sent, and whether it walked.
        self._pings: collections.deque[tuple[float, bool] | None] = collections.deque()
        self._last_input: tuple[float, bool] | None = None

    # --- connection --------------------------------------------------------------------------------------------
    # A connect that fails partway closes what it opened: the robot would otherwise keep streaming to a dead client.
    def connect(self) -> None:
        self._thread.start()
        try:
            self._run(self._open(), READY_TIMEOUT_S + 1)
            features = self._run(self._ask(pb.CorrelationRequest(features_data_request=pb.FeaturesDataRequest())))
            self._imu = features.features_data_response.imu
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

    async def _open(self) -> None:
        await self._transport.open()
        for name in ("mode", "gait", "imu"):
            await self._send(pb.Message(sub_notif=pb.SubscribeNotification(tag=_tag(name))))
        self._tasks = [self._loop.create_task(self._read()), self._loop.create_task(self._keep_alive())]

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
            future = self._replies.pop(message.correlation_response.correlation_id, None)
            if future is not None and not future.done():
                future.set_result(message.correlation_response)
            return
        with self._changed:
            if kind == "mode":
                self._mode = Mode(message.mode.mode)
            elif kind == "gait":
                self._gait = Gait(message.gait.gait)
            elif kind == "imu":
                self._yaw = IMU_YAW_SIGN * math.degrees(message.imu.z)
            elif kind == "pongmsg" and self._pings:
                previous = self._pings.popleft()
                if previous is not None and previous[1] and arrived - previous[0] > STEERING_TIMEOUT_S:
                    self._drops += 1
            self._changed.notify_all()

    async def _keep_alive(self) -> None:
        while self._error is None:
            await asyncio.sleep(KEEPALIVE_S)
            await self._send_input(heartbeat=True)

    # The input and the ping after it go out back to back on the loop's thread, so nothing interleaves.
    async def _send_input(self, heartbeat: bool = False) -> None:
        with self._changed:
            s, height, mode = self._sticks, self._height, self._mode
        previous = None
        try:
            if not heartbeat or mode in HEARTBEAT_MODES:
                data = pb.ControllerInputData(left=pb.Vector(x=s.lx, y=s.ly), right=pb.Vector(x=s.rx, y=0.0),
                                              height=height)
                await self._send(pb.Message(input=data))
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

    async def _ask(self, request: pb.CorrelationRequest) -> pb.CorrelationResponse:
        request.correlation_id = self._next_id
        self._next_id += 1
        future = self._loop.create_future()
        self._replies[request.correlation_id] = future
        await self._send(pb.Message(correlation_request=request))
        reply = await asyncio.wait_for(future, REPLY_TIMEOUT_S)
        if reply.status_code >= 400:
            raise RobotError(f"the robot refused the request with status {reply.status_code}")
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
    # main.cpp's mode bridge forwards every mode request on the robot's EventBus to the subscribers, so the echo says
    # the request arrived; one its arbiter defers (leaving a busy animation) is echoed before it applies.
    def set_mode(self, mode: Mode) -> None:
        self._run(self._send(pb.Message(mode=pb.ModeData(mode=int(mode)))))
        self._wait(lambda: self._mode is mode, f"the robot did not report {mode.name}")

    def set_gait(self, gait: Gait) -> None:
        self._run(self._send(pb.Message(gait=pb.GaitData(gait=int(gait)))))
        self._wait(lambda: self._gait is gait, f"the robot did not report {gait.name}")

    def set_height(self, height: float) -> None:
        with self._changed:
            self._height = height
        self._run(self._send_input())

    def set_velocity(self, velocity: Velocity) -> Velocity:
        sticks = speed_model.sticks_for(self._gait, velocity)
        with self._changed:
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
                              yaw=self._yaw if self._imu else None, link_drops=self._drops)

    def now(self) -> float:
        return time.monotonic()

    def sleep(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        with self._changed:
            while (left := deadline - time.monotonic()) > 0:
                if self._error is not None:
                    raise self._error
                self._changed.wait(left)
            if self._error is not None:
                raise self._error
