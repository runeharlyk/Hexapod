"""Drives the robot's animation system over the native USB serial link, for bench tests before the web
app exists. The framing is SerialAdapter's: a little-endian uint16 length followed by one
socket_message.Message.

    uv run python robot_animate.py --port COM5 list
    uv run python robot_animate.py --port COM5 upload wave        # ../animations/wave.json -> /animations/wave.pb
    uv run python robot_animate.py --port COM5 validate wave
    uv run python robot_animate.py --port COM5 play wave SPEED=1.5  # prints status and mode until it settles
    uv run python robot_animate.py --port COM5 stop               # prints status and mode until it settles
    uv run python robot_animate.py --port COM5 mode ANIMATE       # sticky ANIMATE, e.g. before puppeteering
    uv run python robot_animate.py --port COM5 watch              # print AnimationStatus until Ctrl-C

A COM port is exclusive on Windows, so play and stop follow the run themselves instead of relying on watch.
"""
import argparse
import queue
import struct
import sys
import threading
import time
from pathlib import Path

import serial

from src.platform_shared import message_pb2
from src.robot.animation_files import load_json, to_proto

ROOT = Path(__file__).resolve().parents[1]
CHUNK = 512
MAX_FRAME = 2048
REQUEST_TIMEOUT_S = 5.0
QUIET_S = 2.0
ANIMATION_STATUS_TAG = message_pb2.Message.DESCRIPTOR.fields_by_name["animation_status"].number
MODE_TAG = message_pb2.Message.DESCRIPTOR.fields_by_name["mode"].number


class Link:
    def __init__(self, port: str):
        # DTR and RTS are set low before the port opens, so the lines never pass through the
        # RTS-high, DTR-low state that resets the ESP32-S3 over USB Serial/JTAG.
        self.ser = serial.Serial(baudrate=115200, timeout=0.05, dsrdtr=False, rtscts=False)
        self.ser.port = port
        self.ser.dtr = False
        self.ser.rts = False
        self.ser.open()
        self.rx = bytearray()
        self.responses: dict[int, message_pb2.CorrelationResponse] = {}
        self.next_id = 1
        self.on_status = None
        self.on_mode = None
        threading.Thread(target=self._reader, daemon=True).start()

    def send(self, msg: message_pb2.Message) -> None:
        payload = msg.SerializeToString()
        self.ser.write(struct.pack("<H", len(payload)) + payload)

    def request(self, fill) -> message_pb2.CorrelationResponse:
        msg = message_pb2.Message()
        msg.correlation_request.correlation_id = self.next_id
        self.next_id += 1
        fill(msg.correlation_request)
        self.send(msg)
        deadline = time.time() + REQUEST_TIMEOUT_S
        while time.time() < deadline:
            res = self.responses.pop(msg.correlation_request.correlation_id, None)
            if res is not None:
                return res
            time.sleep(0.01)
        raise SystemExit("no response from the robot (is the native USB port the one you opened?)")

    def subscribe(self, tag: int) -> None:
        msg = message_pb2.Message()
        msg.sub_notif.tag = tag
        self.send(msg)

    def _reader(self) -> None:
        while True:
            self.rx += self.ser.read(4096)
            while len(self.rx) >= 2:
                (length,) = struct.unpack_from("<H", self.rx, 0)
                if length == 0 or length > MAX_FRAME:
                    self.rx.clear()
                    break
                if len(self.rx) < 2 + length:
                    break
                payload = bytes(self.rx[2:2 + length])
                del self.rx[:2 + length]
                msg = message_pb2.Message()
                try:
                    msg.ParseFromString(payload)
                except Exception:
                    continue
                kind = msg.WhichOneof("message")
                if kind == "correlation_response":
                    self.responses[msg.correlation_response.correlation_id] = msg.correlation_response
                elif kind == "animation_status" and self.on_status:
                    self.on_status(msg.animation_status)
                elif kind == "mode" and self.on_mode:
                    self.on_mode(msg.mode)


def upload(link: Link, name: str) -> None:
    data = to_proto(load_json(ROOT / "animations" / f"{name}.json")).SerializeToString()
    path = f"/animations/{name}.pb"
    for offset in range(0, len(data), CHUNK):
        chunk = data[offset:offset + CHUNK]

        def fill(req, offset=offset, chunk=chunk):
            req.file_write_chunk.path = path
            req.file_write_chunk.offset = offset
            req.file_write_chunk.total_size = len(data)
            req.file_write_chunk.content = chunk

        res = link.request(fill)
        if res.status_code != 200:
            raise SystemExit(f"chunk at {offset} refused with {res.status_code}")
    print(f"uploaded {len(data)} bytes to {path}")
    validate(link, name)


def validate(link: Link, name: str) -> None:
    res = link.request(lambda req: setattr(req.animation_validate, "name", name))
    r = res.animation_report
    print(f"{name}: {'ok' if r.ok else 'INVALID ' + r.error}  clamped {r.clamped_mask:018b}")


def list_animations(link: Link) -> None:
    res = link.request(lambda req: req.animation_list_request.SetInParent())
    for e in res.animation_list.entries:
        print(f"{e.name:16s} {e.size} bytes")


def show_status(s) -> None:
    state = message_pb2.AnimationState.Name(s.state)
    print(f"{s.name:16s} {state:12s} t={s.t:6.2f}  clamped {s.clamped_mask:018b}")


def show_mode(m) -> None:
    print(f"mode {message_pb2.ModesEnum.Name(m.mode)}")


def follow(link: Link, send) -> None:
    """Sends a request and prints status and mode traffic until the run settles: a mode change after an idle
    status (the hand-back), or QUIET_S without traffic (a sticky ANIMATE never hands back, and a refused
    play or a stop with nothing playing sends nothing). A running player reports at 5 Hz, so it is never
    quiet that long."""
    events = queue.Queue()
    link.on_status = lambda s: events.put(("status", s))
    link.on_mode = lambda m: events.put(("mode", m))
    link.subscribe(ANIMATION_STATUS_TAG)
    link.subscribe(MODE_TAG)
    send()
    idle = False
    while True:
        try:
            kind, value = events.get(timeout=QUIET_S)
        except queue.Empty:
            return
        if kind == "status":
            show_status(value)
            idle = value.state == message_pb2.ANIM_IDLE
        else:
            show_mode(value)
            if idle:
                return


def play(link: Link, name: str, params: list[str]) -> None:
    msg = message_pb2.Message()
    msg.animation_play.name = name
    for item in params:
        key, value = item.split("=")
        p = msg.animation_play.params.add()
        p.id = message_pb2.AnimationParam.DESCRIPTOR.fields_by_name["id"].enum_type.values_by_name[key].number
        p.value = float(value)
    follow(link, lambda: link.send(msg))


def stop(link: Link) -> None:
    msg = message_pb2.Message()
    msg.animation_stop.SetInParent()
    follow(link, lambda: link.send(msg))


def mode(link: Link, name: str) -> None:
    msg = message_pb2.Message()
    msg.mode.mode = message_pb2.ModesEnum.Value(name)
    link.send(msg)


def watch(link: Link) -> None:
    link.on_status = show_status
    link.subscribe(ANIMATION_STATUS_TAG)
    print("watching AnimationStatus, Ctrl-C to stop")
    try:
        while True:
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True)
    sub = ap.add_subparsers(dest="command", required=True)
    sub.add_parser("list")
    sub.add_parser("upload").add_argument("name")
    sub.add_parser("validate").add_argument("name")
    p = sub.add_parser("play")
    p.add_argument("name")
    p.add_argument("params", nargs="*")
    sub.add_parser("stop")
    sub.add_parser("mode").add_argument("name")
    sub.add_parser("watch")
    args = ap.parse_args(argv)
    link = Link(args.port)
    time.sleep(0.3)
    if args.command == "list":
        list_animations(link)
    elif args.command == "upload":
        upload(link, args.name)
    elif args.command == "validate":
        validate(link, args.name)
    elif args.command == "play":
        play(link, args.name, args.params)
    elif args.command == "stop":
        stop(link)
    elif args.command == "mode":
        mode(link, args.name)
    else:
        watch(link)
    return 0


if __name__ == "__main__":
    sys.exit(main())
