"""Records the robot's per-tick telemetry to a .hexrec file:

    uv run python record.py --robot 192.168.1.39 --seconds 60

Asks the robot for its features (firmware version and ELF hash, kept in the file's header), then
subscribes to the telemetry batches and appends every one as it arrives. A link that drops, or stays
silent for SILENCE_S, is reconnected and the gap marked. Ctrl-C ends the recording; the file is
complete up to the last frame. Analyse it with report_recording.py.
"""
import argparse
import asyncio
import time
from datetime import datetime

import websockets
from google.protobuf import json_format

from src.platform_shared import message_pb2
from src.sim.recording import RecordingWriter

TAG = message_pb2.Message.DESCRIPTOR.fields_by_name["telemetry_batch"].number
# The robot streams a batch every 25 ms while recording: this long without one means the link is
# gone, even when no close ever arrives (a brown-out, or walking out of WiFi range).
SILENCE_S = 3.0


def _host_us() -> int:
    return time.monotonic_ns() // 1000


def subscription() -> bytes:
    return message_pb2.Message(sub_notif=message_pb2.SubscribeNotification(tag=TAG)).SerializeToString()


async def read_features(socket) -> dict:
    request = message_pb2.Message()
    request.correlation_request.correlation_id = 1
    request.correlation_request.features_data_request.SetInParent()
    await socket.send(request.SerializeToString())
    while True:
        reply = message_pb2.Message.FromString(await asyncio.wait_for(socket.recv(), timeout=5))
        if reply.WhichOneof("message") == "correlation_response" and reply.correlation_response.correlation_id == 1:
            return json_format.MessageToDict(reply.correlation_response.features_data_response)


async def record(url: str, out: str, seconds: float | None) -> int:
    deadline = None if seconds is None else time.monotonic() + seconds
    writer = None
    frames = 0
    try:
        while deadline is None or time.monotonic() < deadline:
            try:
                async with websockets.connect(url, open_timeout=10, close_timeout=1, ping_interval=None,
                                              max_size=None) as socket:
                    if writer is None:
                        writer = RecordingWriter(out, await read_features(socket))
                    else:
                        writer.gap(_host_us())
                    await socket.send(subscription())
                    while deadline is None or time.monotonic() < deadline:
                        remaining = None if deadline is None else max(deadline - time.monotonic(), 0.01)
                        wait = SILENCE_S if remaining is None else min(remaining, SILENCE_S)
                        try:
                            frame = await asyncio.wait_for(socket.recv(), timeout=wait)
                        except TimeoutError:
                            if deadline is not None and time.monotonic() >= deadline:
                                return frames
                            raise ConnectionError(f"no frame for {SILENCE_S:g} s") from None
                        if message_pb2.Message.FromString(frame).WhichOneof("message") == "telemetry_batch":
                            writer.write(frame, _host_us())
                            frames += 1
            except TimeoutError:
                print("no answer from the robot; retrying")
            except (OSError, websockets.ConnectionClosed) as error:
                print(f"link lost ({error}); reconnecting")
                await asyncio.sleep(1)
        return frames
    finally:
        if writer is not None:
            writer.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--robot", required=True, help="host name or address, e.g. 192.168.1.39")
    parser.add_argument("--seconds", type=float, help="stop after this long; without it, Ctrl-C stops")
    parser.add_argument("--out", help="output file (default: recording-<time>.hexrec)")
    args = parser.parse_args()
    out = args.out or f"recording-{datetime.now():%Y%m%d-%H%M%S}.hexrec"
    try:
        frames = asyncio.run(record(f"ws://{args.robot}/api/ws", out, args.seconds))
    except KeyboardInterrupt:
        frames = None
    print(f"wrote {out}" + ("" if frames is None else f" ({frames} batches)"))


if __name__ == "__main__":
    main()
