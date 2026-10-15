"""The real robot's backend against a fake robot, over both of its links: a local WebSocket and a fake serial port
in place of pyserial's, with the firmware's COBS framing on it."""
import asyncio
import math
import threading
import time

import pytest
import serial
from websockets.asyncio.server import serve

import robot_animate
from src.hexapod import Gait, Mode, Robot, RobotDisconnected, RobotError, RobotTimeout, Velocity, speed_model
from src.hexapod.backends import real, serial_link
from src.hexapod.backends.real import RealBackend
from src.hexapod.backends.serial_link import SerialLink
from src.platform_shared import message_pb2 as pb
from src.robot import serial_frame

STEERING_TIMEOUT_S = 0.6  # firmware/include/motion.h STEERING_TIMEOUT_MS
BOOT_S = 0.5  # how long the fake board ignores the host after a reset; the ESP32-S3 takes about as long
HELD_GAP_S = 0.02  # between the answers to frames held through a boot, each in its own USB transfer
ROM_BANNER = b"ESP-ROM:esp32s3-20210327\r\nrst:0x15 (USB_UART_CHIP_RESET),boot:0x8 (SPI_FAST_FLASH_BOOT)\r\n"
SERIAL_PORT = "COM7"


def _tag(name):
    return pb.Message.DESCRIPTOR.fields_by_name[name].number


class FakeSerialPort:
    """pyserial's Serial as SerialLink drives it, with the robot's USB Serial/JTAG on the far end. pyserial raises
    SerialException from read and write once the device is unplugged, and PortNotOpenError once the port is closed."""

    def __init__(self, robot, **settings):
        self._robot = robot
        self.timeout = settings["timeout"]
        self.port = None
        self.dtr = self.rts = True  # pyserial's defaults
        self.lines_at_open = None
        self.is_open = False
        self._to_host = bytearray()
        self._data = threading.Condition()
        self._unplugged = False
        self._side = None

    def open(self):
        self.lines_at_open = (self.dtr, self.rts)
        self.is_open = True
        self._side = self._robot.plug(self)

    def write(self, data):
        self._check()
        self._side.feed(bytes(data))
        return len(data)

    def read(self, size):
        with self._data:
            self._data.wait_for(lambda: self._to_host or self._unplugged or not self.is_open, self.timeout)
            self._check()
            data = bytes(self._to_host[:size])
            del self._to_host[:size]
            return data

    def close(self):
        with self._data:
            self.is_open = False
            self._data.notify_all()
        if not self._unplugged:
            self._side.hang_up()

    def deliver(self, data):
        with self._data:
            self._to_host += data
            self._data.notify_all()

    def unplug(self):
        with self._data:
            self._unplugged = True
            self._data.notify_all()

    def _check(self):
        if not self.is_open:
            raise serial.PortNotOpenError()
        if self._unplugged:
            raise serial.SerialException("ClearCommError failed (the device does not recognize the command)")


class SerialSide:
    """The robot's end of a fake serial port: the same frames as a WebSocket, inside the firmware's COBS framing.
    `arrived` is when the frame last iterated reached the board."""

    def __init__(self, port, loop):
        self._port, self._loop = port, loop
        self._frames = asyncio.Queue()
        self.arrived = 0.0
        self._decoder = serial_frame.Decoder(self._arrive)

    def _arrive(self, frame):
        self._loop.call_soon_threadsafe(self._frames.put_nowait, (time.monotonic(), frame))

    def feed(self, data):
        self._decoder.feed(data)

    def hang_up(self):
        self._loop.call_soon_threadsafe(self._frames.put_nowait, None)

    def __aiter__(self):
        return self

    async def __anext__(self):
        item = await self._frames.get()
        if item is None:
            raise StopAsyncIteration
        self.arrived, frame = item
        return frame

    async def send(self, frame):
        self._port.deliver(serial_frame.encode(frame))

    async def close(self):
        self._port.unplug()
        self.hang_up()


class FakeRobot:
    """The firmware's side of /api/ws and of the native USB serial link: echoes mode and gait requests to subscribers
    as its EventBus bridges do, answers pings in order and the features request, streams the IMU yaw at 20 Hz, and
    records every frame when it handles it. `obey=False` drops mode requests as the firmware drops one it rejects;
    `silent` stops all sending; `stall(s)` stops handling frames for s seconds, as a WiFi stall does: they queue up
    and are handled late. `reset_on_open` reboots the board when a serial port opens, as some USB hosts do: the ROM
    prints its banner, and the frames of the next BOOT_S are lost ("drops") or held until the firmware reads them
    ("queues"); which of the two a real host does is unmeasured."""

    def __init__(self, imu=True, obey=True, reset_on_open=None):
        self.imu, self.obey, self.reset_on_open = imu, obey, reset_on_open
        self.status, self.silent = 200, False
        self.yaw_rad = 0.3
        self.mode = None
        self.subscribed = set()
        self.received = []  # (monotonic time handled, Message)
        self.lost_in_boot = 0
        self.connections = 0
        self.ports = []
        self._stall_until = 0.0
        self._booted_at = 0.0
        self._link = None
        self._loop = asyncio.new_event_loop()
        threading.Thread(target=self._loop.run_forever, daemon=True).start()
        self.ws_port = asyncio.run_coroutine_threadsafe(self._start(), self._loop).result(5)

    async def _start(self):
        self._server = await serve(self._handle, "127.0.0.1", 0)
        return self._server.sockets[0].getsockname()[1]

    def serial_port(self, **settings):
        """Stands in for serial.Serial."""
        port = FakeSerialPort(self, **settings)
        self.ports.append(port)
        return port

    def plug(self, port):
        side = SerialSide(port, self._loop)
        if self.reset_on_open:
            self._booted_at = time.monotonic() + BOOT_S
            port.deliver(ROM_BANNER)
        asyncio.run_coroutine_threadsafe(self._attach(side), self._loop).result(5)
        return side

    async def _attach(self, side):
        self._loop.create_task(self._handle(side))

    async def _handle(self, link):
        self._link = link
        self.connections += 1
        streamer = asyncio.create_task(self._stream(link))
        try:
            async for frame in link:
                await asyncio.sleep(max(0.0, self._stall_until - time.monotonic()))
                if getattr(link, "arrived", math.inf) < self._booted_at:
                    if self.reset_on_open == "drops":
                        self.lost_in_boot += 1
                        continue
                    await asyncio.sleep(max(0.0, self._booted_at - time.monotonic()) + HELD_GAP_S)
                message = pb.Message.FromString(frame)
                self.received.append((time.monotonic(), message))
                for reply in self._replies(message):
                    if not self.silent:
                        await link.send(reply.SerializeToString())
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

    async def _stream(self, link):
        while True:
            if self.imu and not self.silent and _tag("imu") in self.subscribed:
                await link.send(pb.Message(imu=pb.IMUData(z=self.yaw_rad)).SerializeToString())
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
        asyncio.run_coroutine_threadsafe(self._link.send(frame), self._loop).result(5)

    def drop(self):
        """The WebSocket closes, or the USB cable comes out."""
        asyncio.run_coroutine_threadsafe(self._link.close(), self._loop).result(5)

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


def _target(fake, transport, monkeypatch):
    """What a script passes to Robot to reach the fake robot over the given link."""
    if transport == "websocket":
        return f"127.0.0.1:{fake.ws_port}"
    monkeypatch.setattr(serial_link, "Serial", fake.serial_port)
    return SERIAL_PORT


@pytest.fixture(params=["websocket", "serial"])
def transport(request):
    return request.param


@pytest.fixture
def fake():
    robot = FakeRobot()
    yield robot
    robot.stop()


@pytest.fixture
def target(fake, transport, monkeypatch):
    return _target(fake, transport, monkeypatch)


@pytest.fixture
def robot(target, monkeypatch):
    monkeypatch.setattr(real, "REPLY_TIMEOUT_S", 1.0)
    client = Robot(target)
    client.connect()
    yield client
    client.close()


def test_connecting_subscribes_sets_the_gait_and_reads_the_imu(robot, fake):
    assert {_tag("mode"), _tag("gait"), _tag("imu")} <= fake.subscribed
    assert [m.gait.gait for _, m in fake.received if m.WhichOneof("message") == "gait"] == [pb.TRI_GATE]
    state = robot.state()
    assert state.mode is None and state.gait is Gait.TRI_GATE  # the firmware pushes its mode only on a change
    assert state.yaw == pytest.approx(-math.degrees(0.3))  # the DMP yaw is clockwise-positive


# The firmware answers pings (ws_frame.h), so the client keeps websockets' keep-alive, which finds a link that died
# without a close frame, such as a robot that lost power, even while nothing else is sent.
def test_the_websocket_link_keeps_its_keep_alive(fake):
    async def opened_interval():
        link = real.WebSocketTransport(f"127.0.0.1:{fake.ws_port}")
        await link.open()
        try:
            return link._socket.ping_interval
        finally:
            await link.close()

    assert asyncio.run(opened_interval()) is not None


def test_a_robot_without_an_imu_has_no_yaw(transport, monkeypatch):
    fake = FakeRobot(imu=False)
    try:
        with Robot(_target(fake, transport, monkeypatch)) as robot:
            assert robot.state().yaw is None
    finally:
        fake.stop()


def test_stand_waits_for_the_mode_and_sends_the_height(robot, fake):
    robot.stand(height_mm=25)
    assert robot.state().mode is Mode.STAND
    _, last = fake.inputs()[-1]
    assert last.height == pytest.approx(-0.5)
    assert (last.left.x, last.left.y, last.right.x, last.speed, last.s1) == (0, 0, 0, 0, 0)


def test_a_mode_the_robot_never_reports_times_out(transport, monkeypatch):
    monkeypatch.setattr(real, "REPLY_TIMEOUT_S", 0.5)
    fake = FakeRobot(obey=False)
    try:
        client = Robot(_target(fake, transport, monkeypatch))
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


def test_leaving_on_an_exception_stops_sits_and_deactivates(fake, target, monkeypatch):
    monkeypatch.setattr(real, "REPLY_TIMEOUT_S", 1.0)
    with pytest.raises(ValueError):
        with Robot(target) as robot:
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


def test_a_silent_link_counts_as_disconnected(fake, target, monkeypatch):
    monkeypatch.setattr(real, "SILENCE_S", 0.6)
    client = Robot(target)
    client.connect()
    fake.silent = True
    started = time.monotonic()
    with pytest.raises(RobotDisconnected):
        client.sleep(3.0)
    assert time.monotonic() - started < 1.5
    client.close()


def test_a_refused_request_fails_the_connect_and_leaves_no_connection(fake, target):
    fake.status = 503
    with pytest.raises(RobotError):
        RealBackend(target).connect()
    _eventually(lambda: fake.connections == 0)
    assert not any(port.is_open for port in fake.ports)


# --- the serial link ---------------------------------------------------------------------------------------------

@pytest.mark.parametrize("name, is_serial", [
    ("COM5", True), ("com12", True), ("/dev/ttyACM0", True), ("/dev/cu.usbmodem101", True),
    ("192.168.4.1", False), ("hexapod.local", False), ("127.0.0.1:8080", False), ("COMPUTER", False),
])
def test_a_serial_port_name_selects_the_serial_link(name, is_serial):
    assert serial_link.is_serial_port(name) is is_serial


# Opening a port with pyserial's defaults (DTR and RTS high) passes through the line state that resets the ESP32-S3.
def test_the_serial_link_opens_without_touching_the_reset_lines(fake, monkeypatch):
    with Robot(_target(fake, "serial", monkeypatch)):
        assert [(port.port, port.lines_at_open) for port in fake.ports] == [(SERIAL_PORT, (False, False))]


def test_a_board_that_resets_on_open_is_waited_for(monkeypatch):
    fake = FakeRobot(reset_on_open="drops")
    try:
        with Robot(_target(fake, "serial", monkeypatch)) as robot:
            robot.stand()
            robot.move_forward(0.1)
            assert robot.state().link_drops == 0
        assert fake.lost_in_boot > 0
        assert fake.mode == pb.DEACTIVATED
    finally:
        fake.stop()


# The backend counts pongs against its own pings to bound the dead-man silence: a late answer to a probe must not
# reach it.
def test_late_answers_to_the_boot_probes_stay_inside_the_link(monkeypatch):
    fake = FakeRobot(reset_on_open="queues")
    _target(fake, "serial", monkeypatch)
    received = []
    link = SerialLink(SERIAL_PORT, on_message=received.append)
    try:
        link.wait_ready()
        time.sleep(0.3)
        assert fake.kinds().count("pingmsg") > 1
        assert [m.WhichOneof("message") for m in received] == []
    finally:
        link.close()
        fake.stop()


def test_the_serial_link_correlates_requests_and_times_out_on_silence(fake, monkeypatch):
    _target(fake, "serial", monkeypatch)
    link = SerialLink(SERIAL_PORT)
    try:
        link.wait_ready()
        reply = link.request(pb.CorrelationRequest(features_data_request=pb.FeaturesDataRequest()))
        sent = [m.correlation_request for _, m in fake.received if m.WhichOneof("message") == "correlation_request"]
        assert reply.correlation_id == sent[-1].correlation_id and reply.features_data_response.imu
        fake.silent = True
        started = time.monotonic()
        with pytest.raises(RobotTimeout):
            link.request(pb.CorrelationRequest(features_data_request=pb.FeaturesDataRequest()), timeout=0.3)
        assert time.monotonic() - started < 0.6
    finally:
        link.close()
    _eventually(lambda: fake.connections == 0)


def test_robot_animate_drives_the_robot_through_the_same_link(fake, monkeypatch):
    _target(fake, "serial", monkeypatch)
    assert robot_animate.main(["--port", SERIAL_PORT, "mode", "ANIMATE"]) == 0
    _eventually(lambda: fake.mode == pb.ANIMATE)
    _eventually(lambda: fake.connections == 0)  # the tool closes the port behind it
