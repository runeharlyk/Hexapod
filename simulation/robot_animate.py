"""Drives the robot's animation system over the native USB serial link, for bench tests before the web
app exists. The link is src/hexapod/backends/serial_link.py, which Robot("COMx") uses too: COBS with a CRC
around one socket_message.Message.

    uv run python robot_animate.py --port COM5 list
    uv run python robot_animate.py --port COM5 upload wave        # ../animations/wave.json -> /animations/wave.pb
    uv run python robot_animate.py --port COM5 validate wave
    uv run python robot_animate.py --port COM5 play wave SPEED=1.5  # prints status and mode until it settles
    uv run python robot_animate.py --port COM5 stop               # prints status and mode until it settles
    uv run python robot_animate.py --port COM5 mode ANIMATE       # sticky ANIMATE, e.g. before puppeteering
    uv run python robot_animate.py --port COM5 watch              # print AnimationStatus until Ctrl-C
    uv run python robot_animate.py --port COM5 shell              # the commands above on one open port

A COM port is exclusive on Windows, so play and stop follow the run themselves instead of relying on watch,
and shell keeps the port open so a looping or holding animation can be stopped and plays can be chained.
"""
import argparse
import queue
import shlex
import sys
import threading
import time
from pathlib import Path

from src.hexapod import RobotError, RobotTimeout
from src.hexapod.backends.serial_link import SerialLink
from src.platform_shared import message_pb2
from src.robot.animation_files import load_json, to_proto

ROOT = Path(__file__).resolve().parents[1]
CHUNK = 512
QUIET_S = 2.0
# A firmware from before the snapshots acks no subscribe, so the wait for the acks is bounded.
ACK_WAIT_S = 1.0
ANIMATION_STATUS_TAG = message_pb2.Message.DESCRIPTOR.fields_by_name["animation_status"].number
MODE_TAG = message_pb2.Message.DESCRIPTOR.fields_by_name["mode"].number


def upload(link: SerialLink, name: str) -> None:
    data = to_proto(load_json(ROOT / "animations" / f"{name}.json")).SerializeToString()
    path = f"/animations/{name}.pb"
    for offset in range(0, len(data), CHUNK):
        req = message_pb2.CorrelationRequest()
        req.file_write_chunk.path = path
        req.file_write_chunk.offset = offset
        req.file_write_chunk.total_size = len(data)
        req.file_write_chunk.content = data[offset:offset + CHUNK]
        res = link.request(req)
        if res.status_code != 200:
            raise SystemExit(f"chunk at {offset} refused with {res.status_code}")
    print(f"uploaded {len(data)} bytes to {path}")
    validate(link, name)


def validate(link: SerialLink, name: str) -> None:
    req = message_pb2.CorrelationRequest()
    req.animation_validate.name = name
    res = link.request(req)
    r = res.animation_report
    print(f"{name}: {'ok' if r.ok else 'INVALID ' + r.error}  clamped {r.clamped_mask:018b}")


def list_animations(link: SerialLink) -> None:
    req = message_pb2.CorrelationRequest()
    req.animation_list_request.SetInParent()
    res = link.request(req)
    for e in res.animation_list.entries:
        print(f"{e.name:16s} {e.size} bytes")


def show_status(s) -> None:
    state = message_pb2.AnimationState.Name(s.state)
    print(f"{s.name:16s} {state:12s} t={s.t:6.2f}  clamped {s.clamped_mask:018b}")


def show_mode(m) -> None:
    print(f"mode {message_pb2.ModesEnum.Name(m.mode)}")


def follow(link: SerialLink, send) -> None:
    """Sends a request and prints status and mode traffic until the run settles: a mode change after an idle
    status (the hand-back), or QUIET_S without traffic (a sticky ANIMATE never hands back, and a refused
    play or a stop with nothing playing sends nothing). A running player reports at 5 Hz, so it is never
    quiet that long. A hold, or QUIET_S of PLAYING whose clock has wrapped, never settles on its own, so
    following ends there with the reason; Ctrl-C ends it too."""
    events = queue.Queue()
    # The robot answers each subscribe with the topic's current value and then the ack: the state before the command,
    # no answer to it, so what comes before the acks is not followed. A tag the link already subscribes to is not
    # subscribed again, so it brings neither.
    unacked = {ANIMATION_STATUS_TAG, MODE_TAG}
    acked = threading.Event()

    def route(msg):
        kind = msg.WhichOneof("message")
        if kind == "sub_notif":
            unacked.discard(msg.sub_notif.tag)
            if not unacked:
                acked.set()
        elif kind in ("animation_status", "mode") and acked.is_set():
            events.put((kind, getattr(msg, kind)))

    link.on_message = route
    for tag in (ANIMATION_STATUS_TAG, MODE_TAG):
        if not link.subscribe(tag):
            unacked.discard(tag)
    if not unacked:
        acked.set()
    acked.wait(ACK_WAIT_S)
    acked.set()
    send()
    idle = False
    state, since, last_t, wrapped = None, time.time(), 0.0, False
    try:
        while True:
            try:
                kind, value = events.get(timeout=QUIET_S)
            except queue.Empty:
                return
            if kind == "mode":
                show_mode(value)
                if idle:
                    return
                continue
            show_status(value)
            idle = value.state == message_pb2.ANIM_IDLE
            if value.state != state:
                state, since, wrapped = value.state, time.time(), False
            elif value.state == message_pb2.ANIM_PLAYING and value.t < last_t:
                wrapped = True
            last_t = value.t
            if state == message_pb2.ANIM_HOLD:
                print("holding the last keyframe until stopped; not following further")
                return
            if wrapped and time.time() - since >= QUIET_S:
                print("looping (or repeating) until stopped; not following further")
                return
    except KeyboardInterrupt:
        print("stopped following")
    finally:
        link.on_message = None


def play(link: SerialLink, name: str, params: list[str]) -> None:
    msg = message_pb2.Message()
    msg.animation_play.name = name
    for item in params:
        key, value = item.split("=")
        p = msg.animation_play.params.add()
        p.id = message_pb2.AnimationParam.DESCRIPTOR.fields_by_name["id"].enum_type.values_by_name[key].number
        p.value = float(value)
    follow(link, lambda: link.send(msg))


def stop(link: SerialLink) -> None:
    msg = message_pb2.Message()
    msg.animation_stop.SetInParent()
    follow(link, lambda: link.send(msg))


def mode(link: SerialLink, name: str) -> None:
    msg = message_pb2.Message()
    msg.mode.mode = message_pb2.ModesEnum.Value(name)
    link.send(msg)


def watch(link: SerialLink) -> None:
    link.on_message = lambda msg: show_status(msg.animation_status) if msg.HasField("animation_status") else None
    link.subscribe(ANIMATION_STATUS_TAG)
    print("watching AnimationStatus, Ctrl-C to stop")
    try:
        while True:
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        link.on_message = None


def add_commands(sub) -> None:
    sub.add_parser("list")
    sub.add_parser("upload").add_argument("name")
    sub.add_parser("validate").add_argument("name")
    p = sub.add_parser("play")
    p.add_argument("name")
    p.add_argument("params", nargs="*")
    sub.add_parser("stop")
    sub.add_parser("mode").add_argument("name")
    sub.add_parser("watch")


def parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True)
    sub = ap.add_subparsers(dest="command", required=True)
    add_commands(sub)
    sub.add_parser("shell")
    return ap


def shell(link: SerialLink) -> None:
    """Reads the commands, without --port, from stdin until quit or the end of input."""
    ap = argparse.ArgumentParser(prog="", add_help=False, exit_on_error=False)
    add_commands(ap.add_subparsers(dest="command", required=True))
    print("commands: list, upload NAME, validate NAME, play NAME [ID=VALUE ...], stop, mode NAME, watch, quit")
    while True:
        try:
            line = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if line == "quit":
            return
        if not line:
            continue
        try:
            run(link, ap.parse_args(shlex.split(line)))
        except KeyboardInterrupt:
            print("interrupted")
        except (argparse.ArgumentError, SystemExit, KeyError, ValueError, RobotTimeout) as e:
            # SystemExit is argparse rejecting the line or a refused upload; the port stays open.
            print(f"error: {e}")


def main(argv=None) -> int:
    args = parser().parse_args(argv)
    try:
        link = SerialLink(args.port)
        try:
            link.wait_ready()
            if args.command == "shell":
                shell(link)
            else:
                run(link, args)
        finally:
            link.close()
    except RobotError as e:
        raise SystemExit(f"error: {e}") from e
    return 0


def run(link: SerialLink, args) -> None:
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


if __name__ == "__main__":
    sys.exit(main())
