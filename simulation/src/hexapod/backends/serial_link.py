"""The robot's native USB serial link (firmware SerialAdapter): one socket_message.Message per COBS frame
(src/robot/serial_frame.py), the messages its WebSocket carries. robot_animate.py and the Robot's serial backend
both talk through it.

A reader thread owns the port's input and hands every message the link does not claim itself to `on_message`;
send and request are safe from any thread. Once the link fails it stays failed and `on_closed` hears why, once.
"""
import re
import threading
import time
from typing import Callable

from google.protobuf.message import DecodeError
from serial import Serial, SerialException, SerialTimeoutException

from ...robot import serial_frame
from .base import RobotDisconnected, RobotError, RobotTimeout
from .messages import pb

BAUD = 115200  # USB Serial/JTAG ignores it and runs at USB speed
READ_TIMEOUT_S = 0.05
WRITE_TIMEOUT_S = 1.0
REQUEST_TIMEOUT_S = 5.0
READY_TIMEOUT_S = 5.0  # covers a reset and the boot up to SerialAdapter::begin
PROBE_S = 0.2
SERIAL_PORT = re.compile(r"COM\d+|/dev/.+", re.IGNORECASE)


def is_serial_port(target: str) -> bool:
    """COMx on Windows, a /dev/ node elsewhere."""
    return SERIAL_PORT.fullmatch(target) is not None


class SerialLink:
    def __init__(self, port: str, on_message: Callable[[pb.Message], None] | None = None,
                 on_closed: Callable[[RobotDisconnected], None] | None = None):
        self.port = port
        self.on_message = on_message
        self._on_closed = on_closed
        self._decoder = serial_frame.Decoder(self._receive)
        self._writing = threading.Lock()
        self._changed = threading.Condition()
        self._replies: dict[int, pb.CorrelationResponse | None] = {}
        self._next_id = 1
        self._subscribed: set[int] = set()
        self._probing = False
        self._pongs = 0
        self._error: RobotDisconnected | None = None
        self._closing = False
        # DTR and RTS are set low before the port opens, so the lines never pass through the
        # RTS-high, DTR-low state that resets the ESP32-S3 over USB Serial/JTAG.
        self._serial = Serial(baudrate=BAUD, timeout=READ_TIMEOUT_S, write_timeout=WRITE_TIMEOUT_S, dsrdtr=False,
                              rtscts=False)
        self._serial.port = port
        self._serial.dtr = False
        self._serial.rts = False
        try:
            self._serial.open()
        except SerialException as error:
            raise RobotError(f"cannot open the serial port {port}: {error}") from error
        self._reader = threading.Thread(target=self._read, daemon=True)
        self._reader.start()

    def close(self) -> None:
        self._closing = True
        self._serial.close()
        self._reader.join(1.0)

    def wait_ready(self, timeout: float = READY_TIMEOUT_S) -> None:
        """Pings until the firmware answers. Some hosts reset the board when the port opens anyway, and what is sent
        into the boot is lost or held back until the firmware starts reading; the running firmware answers a ping
        within milliseconds, so listening one more PROBE_S after the first pong takes the late answers off the line
        before anyone else counts pongs."""
        deadline = time.monotonic() + timeout
        with self._changed:
            self._probing, self._pongs = True, 0
        try:
            while True:
                self.send(pb.Message(pingmsg=pb.PingMsg()))
                with self._changed:
                    self._changed.wait_for(lambda: self._pongs or self._error is not None,
                                           max(0.0, min(PROBE_S, deadline - time.monotonic())))
                    self._raise_error()
                    if self._pongs:
                        break
                if time.monotonic() >= deadline:
                    raise RobotTimeout(f"no answer from the robot on {self.port} (is it the robot's native USB port?)")
            time.sleep(PROBE_S)
        finally:
            with self._changed:
                self._probing = False

    def send(self, message: pb.Message) -> None:
        frame = serial_frame.encode(message.SerializeToString())
        with self._writing:
            self._raise_error()
            try:
                self._serial.write(frame)
            except SerialTimeoutException as error:
                raise RobotTimeout(f"the robot on {self.port} stopped reading") from error
            except SerialException as error:
                raise self._fail(f"the robot on {self.port} went away: {error}") from error

    def request(self, request: pb.CorrelationRequest, timeout: float = REQUEST_TIMEOUT_S) -> pb.CorrelationResponse:
        with self._changed:
            request.correlation_id = self._next_id
            self._next_id += 1
            self._replies[request.correlation_id] = None
        try:
            self.send(pb.Message(correlation_request=request))
            with self._changed:
                self._changed.wait_for(
                    lambda: self._replies[request.correlation_id] is not None or self._error is not None, timeout)
                reply = self._replies[request.correlation_id]
                if reply is not None:
                    return reply
                self._raise_error()
                raise RobotTimeout(f"no answer from the robot on {self.port} (is it the robot's native USB port?)")
        finally:
            with self._changed:
                del self._replies[request.correlation_id]

    def subscribe(self, tag: int) -> bool:
        """Once per tag: firmware before the idempotent subscribe records every subscription, so a repeat would double
        each frame there. Whether it was sent now, and so is answered with the snapshot and the ack."""
        if tag in self._subscribed:
            return False
        self.send(pb.Message(sub_notif=pb.SubscribeNotification(tag=tag)))
        self._subscribed.add(tag)
        return True

    def _read(self) -> None:
        while not self._closing and self._error is None:
            try:
                data = self._serial.read(4096)
            except SerialException as error:  # also the PortNotOpenError of a port closed under the read
                if not self._closing:
                    self._fail(f"the robot on {self.port} went away: {error}")
                return
            self._decoder.feed(data)

    def _receive(self, payload: bytes) -> None:
        if self._error is not None:
            return
        try:
            message = pb.Message.FromString(payload)
        except DecodeError as error:  # the CRC passed, so this is a firmware newer than this client
            self._fail(f"the robot on {self.port} sent a frame this client cannot read: {error}")
            return
        kind = message.WhichOneof("message")
        with self._changed:
            if kind == "correlation_response" and message.correlation_response.correlation_id in self._replies:
                self._replies[message.correlation_response.correlation_id] = message.correlation_response
                self._changed.notify_all()
                return
            if kind == "pongmsg" and self._probing:
                self._pongs += 1
                self._changed.notify_all()
                return
        if self.on_message is not None:
            self.on_message(message)

    def _fail(self, reason: str) -> RobotDisconnected:
        with self._changed:
            first = self._error is None
            if first:
                self._error = RobotDisconnected(reason)
                self._changed.notify_all()
            error = self._error
        if first and self._on_closed is not None:
            self._on_closed(error)
        return error

    def _raise_error(self) -> None:
        if self._error is not None:
            raise self._error
