"""The real robot's backend against a fake robot on a local WebSocket."""
import asyncio
import math
import threading
import time

import pytest
from websockets.asyncio.server import serve

from src.hexapod import Gait, Mode, Robot, RobotDisconnected, RobotError, RobotTimeout, Velocity, speed_model
from src.hexapod.backends import real
from src.hexapod.backends.real import RealBackend
from src.platform_shared import message_pb2 as pb

STEERING_TIMEOUT_S = 0.6  # firmware/include/motion.h STEERING_TIMEOUT_MS


def _tag(name):
    return pb.Message.DESCRIPTOR.fields_by_name[name].number


class FakeRobot:
    """The firmware's side of /api/ws: echoes mode and gait requests to subscribers as its EventBus bridges do,
    answers pings in order and the features request, streams the IMU yaw at 20 Hz, and records every frame when it
    handles it. `obey=False` drops mode requests as the firmware drops one it rejects; `silent` stops all sending;
    `stall(s)` stops handling frames for s seconds, as a WiFi stall does: they queue up and are handled late."""

    def __init__(self, imu=True, obey=True):
        self.imu, self.obey = imu, obey
        self.status, self.silent = 200, False
        self.yaw_rad = 0.3
        self.mode = None
        self.subscribed = set()
        self.received = []  # (monotonic time handled, Message)
        self.connections = 0
        self._stall_until = 0.0
        self._socket = None
        self._loop = asyncio.new_event_loop()
        threading.Thread(target=self._loop.run_forever, daemon=True).start()
        self.port = asyncio.run_coroutine_threadsafe(self._start(), self._loop).result(5)

    async def _start(self):
        self._server = await serve(self._handle, "127.0.0.1", 0)
        return self._server.sockets[0].getsockname()[1]

    async def _handle(self, socket):
        self._socket = socket
        self.connections += 1
        streamer = asyncio.create_task(self._stream(socket))
        try:
            async for frame in socket:
                await asyncio.sleep(max(0.0, self._stall_until - time.monotonic()))
                message = pb.Message.FromString(frame)
                self.received.append((time.monotonic(), message))
                for reply in self._replies(message):
                    if not self.silent:
                        await socket.send(reply.SerializeToString())
        finally:
            streamer.cancel()
            self.connections -= 1

    def _replies(self, message):
        kind = message.WhichOneof("message")
        if kind == "sub_notif":
            self.subscribed.add(message.sub_notif.tag)
        elif kind == "pingmsg":
            yield pb.Message(pongmsg=pb.PongMsg())
        elif kind == "mode" and self.obey:
            self.mode = message.mode.mode
            if _tag("mode") in self.subscribed:
                yield pb.Message(mode=pb.ModeData(mode=self.mode))
        elif kind == "gait" and _tag("gait") in self.subscribed:
            yield pb.Message(gait=pb.GaitData(gait=message.gait.gait))
        elif kind == "correlation_request":
            response = pb.CorrelationResponse(correlation_id=message.correlation_request.correlation_id,
                                              status_code=self.status)
            response.features_data_response.imu = self.imu
            yield pb.Message(correlation_response=response)

    async def _stream(self, socket):
        while True:
            if self.imu and not self.silent and _tag("imu") in self.subscribed:
                await socket.send(pb.Message(imu=pb.IMUData(z=self.yaw_rad)).SerializeToString())
            await asyncio.sleep(0.05)

    def stall(self, seconds):
        self._stall_until = time.monotonic() + seconds

    def kinds(self):
        return [message.WhichOneof("message") for _, message in self.received]

    def inputs(self):
        return [(t, message.input) for t, message in self.received if message.WhichOneof("message") == "input"]

    def longest_walking_silence(self):
        """The longest the firmware went without input after a walking one: past 600 ms its dead-man fired."""
        mode, previous, longest = None, None, 0.0
        for t, message in self.received:
            kind = message.WhichOneof("message")
            if kind == "mode":
                mode = message.mode.mode
            elif kind == "input":
                if previous is not None:
                    longest = max(longest, t - previous)
                steering = message.input.left.x or message.input.left.y or message.input.right.x
                previous = t if mode == pb.WALK and steering else None
        return longest

    def push_mode(self, mode):
        """A mode change from elsewhere, as the app's stop button or the handheld controller makes."""
        self.mode, self.pushed_at = mode, time.monotonic()
        frame = pb.Message(mode=pb.ModeData(mode=mode)).SerializeToString()
        asyncio.run_coroutine_threadsafe(self._socket.send(frame), self._loop).result(5)

    def drop(self):
        asyncio.run_coroutine_threadsafe(self._socket.close(), self._loop).result(5)

    def stop(self):
        asyncio.run_coroutine_threadsafe(self._stop(), self._loop).result(5)
        self._loop.call_soon_threadsafe(self._loop.stop)

    async def _stop(self):
        self._server.close()
        await self._server.wait_closed()


# A send returns once the client sent it; the fake robot handles it a moment later on its own thread.
def _eventually(done, timeout=1.0):
    deadline = time.monotonic() + timeout
    while not done():
        assert time.monotonic() < deadline, "the fake robot never got there"
        time.sleep(0.01)


@pytest.fixture
def fake():
    robot = FakeRobot()
    yield robot
    robot.stop()


@pytest.fixture
def robot(fake, monkeypatch):
    monkeypatch.setattr(real, "REPLY_TIMEOUT_S", 1.0)
    client = Robot(backend=RealBackend(f"127.0.0.1:{fake.port}"))
    client.connect()
    yield client
    client.close()


def test_connecting_subscribes_sets_the_gait_and_reads_the_imu(robot, fake):
    assert {_tag("mode"), _tag("gait"), _tag("imu")} <= fake.subscribed
    assert [m.gait.gait for _, m in fake.received if m.WhichOneof("message") == "gait"] == [pb.TRI_GATE]
    state = robot.state()
    assert state.mode is None and state.gait is Gait.TRI_GATE  # the firmware pushes its mode only on a change
    assert state.yaw == pytest.approx(-math.degrees(0.3))  # the DMP yaw is clockwise-positive


def test_a_robot_without_an_imu_has_no_yaw():
    fake = FakeRobot(imu=False)
    try:
        with Robot(backend=RealBackend(f"127.0.0.1:{fake.port}")) as robot:
            assert robot.state().yaw is None
    finally:
        fake.stop()


def test_stand_waits_for_the_mode_and_sends_the_height(robot, fake):
    robot.stand(height_mm=25)
    assert robot.state().mode is Mode.STAND
    _, last = fake.inputs()[-1]
    assert last.height == pytest.approx(-0.5)
    assert (last.left.x, last.left.y, last.right.x, last.speed, last.s1) == (0, 0, 0, 0, 0)


def test_a_mode_the_robot_never_reports_times_out(monkeypatch):
    monkeypatch.setattr(real, "REPLY_TIMEOUT_S", 0.5)
    fake = FakeRobot(obey=False)
    try:
        client = Robot(backend=RealBackend(f"127.0.0.1:{fake.port}"))
        client.connect()
        started = time.monotonic()
        with pytest.raises(RobotTimeout):
            client.stand()
        assert time.monotonic() - started < 1.5
        client.close()
    finally:
        fake.stop()


def test_a_walk_is_kept_alive_inside_the_dead_man_and_stopped_at_once(robot, fake):
    robot.stand()
    robot.set_velocity(0.1, 0.0, 0.0)
    robot.sleep(1.5)
    walking = [t for t, data in fake.inputs() if data.left.y]
    assert len(walking) >= 7
    assert max(b - a for a, b in zip(walking, walking[1:])) < 0.35
    assert fake.longest_walking_silence() < STEERING_TIMEOUT_S and robot.state().link_drops == 0
    stopped = time.monotonic()
    robot.stop()
    _eventually(lambda: fake.inputs()[-1][1].left.y == 0)
    assert fake.inputs()[-1][0] - stopped < 0.1
    robot.sleep(0.5)  # WALK keeps its heartbeat, now with centred sticks
    assert all(data.left.y == 0 for t, data in fake.inputs() if t > stopped)


# REP-103 on the wire as the simulation measured it: +lx walks left, +rx turns counter-clockwise.
@pytest.mark.parametrize("velocity, stick", [
    (Velocity(0.1, 0, 0), "ly+"), (Velocity(-0.1, 0, 0), "ly-"), (Velocity(0, 0.1, 0), "lx+"),
    (Velocity(0, -0.1, 0), "lx-"), (Velocity(0, 0, 0.5), "rx+"), (Velocity(0, 0, -0.5), "rx-"),
])
def test_each_axis_reaches_its_stick(robot, fake, velocity, stick):
    robot.stand()
    robot.set_velocity(velocity.vx, velocity.vy, velocity.yaw_rate)
    _eventually(lambda: any((fake.inputs()[-1][1].left.x, fake.inputs()[-1][1].left.y, fake.inputs()[-1][1].right.x)))
    _, data = fake.inputs()[-1]
    sent = {"lx": data.left.x, "ly": data.left.y, "rx": data.right.x}
    expected = speed_model.sticks_for(Gait.TRI_GATE, velocity)
    assert (sent["lx"], sent["ly"], sent["rx"]) == pytest.approx((expected.lx, expected.ly, expected.rx))
    assert math.copysign(1, sent[stick[:2]]) == (1 if stick[2] == "+" else -1)
    assert [axis for axis, value in sent.items() if value] == [stick[:2]]


def test_leaving_on_an_exception_stops_sits_and_deactivates(fake, monkeypatch):
    monkeypatch.setattr(real, "REPLY_TIMEOUT_S", 1.0)
    with pytest.raises(ValueError):
        with Robot(backend=RealBackend(f"127.0.0.1:{fake.port}")) as robot:
            robot.stand()
            robot.set_velocity(0.1, 0, 0)
            raise ValueError("a bug in the script")
    _eventually(lambda: fake.connections == 0)
    frames = [(m.WhichOneof("message"), m) for _, m in fake.received if m.WhichOneof("message") in ("input", "mode")]
    last_walk = max(i for i, (kind, m) in enumerate(frames) if kind == "input" and m.input.left.y)
    tail = frames[last_walk + 1:]
    assert tail[0][0] == "input" and tail[0][1].input.left.y == 0  # the stop comes first
    modes = [m.mode.mode for kind, m in tail if kind == "mode"]
    assert modes == [pb.STAND, pb.DEACTIVATED] and fake.mode == pb.DEACTIVATED
    sit = next(i for i, (kind, m) in enumerate(tail) if kind == "input" and m.input.height == 1.0)
    assert sit < [i for i, (kind, _) in enumerate(tail) if kind == "mode"][-1]


# The firmware stops a walk that hears nothing for 600 ms and says nothing about it; the client must notice.
def test_a_stall_past_the_dead_man_is_reported_as_a_move_cut_short(robot, fake):
    robot.stand()
    threading.Timer(0.5, fake.stall, (0.9,)).start()
    with pytest.raises(RobotError, match="dead-man") as raised:
        robot.move_forward(0.3)
    assert not isinstance(raised.value, RobotDisconnected)
    assert fake.longest_walking_silence() > STEERING_TIMEOUT_S  # the robot really would have stopped


# The heartbeat must not fight the app's stop button, and the stop it caused is not a dead-man drop.
def test_a_move_the_app_deactivated_midway_is_reported_and_not_resumed(robot, fake):
    robot.stand()
    threading.Timer(0.5, fake.push_mode, (pb.DEACTIVATED,)).start()
    with pytest.raises(RobotError, match="DEACTIVATED"):
        robot.move_forward(0.3)
    assert robot.state().link_drops == 0
    assert any(data.left.y for t, data in fake.inputs() if t < fake.pushed_at)
    assert not any(data.left.y for t, data in fake.inputs() if t > fake.pushed_at + 0.1)


def test_a_healthy_move_reports_no_drop(robot, fake):
    robot.stand()
    robot.move_forward(0.15)
    assert robot.state().link_drops == 0 and fake.longest_walking_silence() < STEERING_TIMEOUT_S


def test_a_deactivated_robot_gets_no_heartbeat_but_the_link_is_still_watched(robot, fake):
    robot.stand()
    robot.deactivate()
    _eventually(lambda: fake.mode == pb.DEACTIVATED)
    inputs, pings = len(fake.inputs()), fake.kinds().count("pingmsg")
    robot.sleep(0.6)
    assert len(fake.inputs()) == inputs and fake.kinds().count("pingmsg") >= pings + 2


def test_a_dropped_link_raises(robot, fake):
    fake.drop()
    with pytest.raises(RobotDisconnected):
        robot.sleep(1.0)
    with pytest.raises(RobotDisconnected):
        robot.state()


def test_a_silent_link_counts_as_disconnected(fake, monkeypatch):
    monkeypatch.setattr(real, "SILENCE_S", 0.6)
    client = Robot(backend=RealBackend(f"127.0.0.1:{fake.port}"))
    client.connect()
    fake.silent = True
    started = time.monotonic()
    with pytest.raises(RobotDisconnected):
        client.sleep(3.0)
    assert time.monotonic() - started < 1.5
    client.close()


def test_a_refused_request_fails_the_connect_and_leaves_no_connection(fake):
    fake.status = 503
    with pytest.raises(RobotError):
        RealBackend(f"127.0.0.1:{fake.port}").connect()
    _eventually(lambda: fake.connections == 0)
