"""Fits the compass's hard-iron offset to a recording and optionally stores it on the robot:

    uv run python record.py --robot 192.168.1.39 --seconds 60      # turn the robot slowly through every orientation
    uv run python calibrate_compass.py recording.hexrec [--write 192.168.1.39]

The recording holds the readings before the stored offset, so a robot that was calibrated before is fitted afresh.
Without --write it only prints the result. Turning the robot through every orientation fits all three axes. Yawing it
on the floor fits x and y, which are all the heading uses; z is then left as it is on the robot. The fit is refused,
and nothing written, when x and y are not determined or the field it measured is not the Earth's.
"""
import argparse
import asyncio
import sys

import websockets

from src.platform_shared import api_pb2, message_pb2
from src.sim.compass_fit import MAX_STD_ERROR_UT, fit_recording
from src.sim.recording import load

EARTH_FIELD_UT = (25.0, 65.0)  # the geomagnetic field's strength anywhere on Earth
REPLY_TIMEOUT_S = 5.0


async def _ask(socket, request: message_pb2.CorrelationRequest) -> message_pb2.CorrelationResponse:
    request.correlation_id = 1
    await socket.send(message_pb2.Message(correlation_request=request).SerializeToString())
    while True:
        reply = message_pb2.Message.FromString(await asyncio.wait_for(socket.recv(), timeout=REPLY_TIMEOUT_S))
        if reply.WhichOneof("message") == "correlation_response" and reply.correlation_response.correlation_id == 1:
            return reply.correlation_response


async def _read_peripheral_settings(socket) -> api_pb2.PeripheralSettings:
    """Subscribes to the settings topic, which answers with its value and then the subscribe as the ack, and ends the
    subscription."""
    tag = message_pb2.Message.DESCRIPTOR.fields_by_name["peripheral_settings"].number
    await socket.send(message_pb2.Message(sub_notif=message_pb2.SubscribeNotification(tag=tag)).SerializeToString())
    settings = None
    while True:
        frame = message_pb2.Message.FromString(await asyncio.wait_for(socket.recv(), timeout=REPLY_TIMEOUT_S))
        kind = frame.WhichOneof("message")
        if kind == "peripheral_settings":
            settings = frame.peripheral_settings
        elif kind == "sub_notif" and frame.sub_notif.tag == tag:
            break
    await socket.send(message_pb2.Message(unsub_notif=message_pb2.UnsubscribeNotification(tag=tag)).SerializeToString())
    if settings is None:
        raise RuntimeError("the robot sent no peripheral_settings: its firmware is older than the settings topics")
    return settings


async def store_offset(address: str, x: float, y: float, z: float | None) -> None:
    """Writes the offset into the robot's peripheral settings, keeping the I2C configuration it holds, and its z
    offset when z is None."""
    async with websockets.connect(f"ws://{address}/api/ws", open_timeout=10) as socket:
        settings = await _read_peripheral_settings(socket)
        settings.mag_offset_x, settings.mag_offset_y = x, y
        if z is not None:
            settings.mag_offset_z = z
        reply = await _ask(socket, message_pb2.CorrelationRequest(peripheral_settings_update=settings))
        if reply.status_code >= 400:
            raise RuntimeError(f"the robot rejected the offset (status {reply.status_code})")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("path", help="a .hexrec file from record.py")
    parser.add_argument("--write", metavar="ADDRESS", help="store the offset on the robot at this host name or address")
    args = parser.parse_args(argv)
    try:
        fit = fit_recording(load(args.path))
    except ValueError as error:
        print(f"refused: {error}", file=sys.stderr)
        return 1
    z_known = fit.z_determined
    offset = [f"{v:.2f}" for v in fit.offset[:2]] + [f"{fit.offset[2]:.2f}" if z_known else "not determined"]
    print(f"offset {', '.join(offset)} uT from {fit.samples} samples; standard error "
          f"{fit.std_error[0]:.2f} (x), {fit.std_error[1]:.2f} (y)" + (f", {fit.std_error[2]:.2f} (z) uT" if z_known else " uT"))
    print(f"residual {fit.residual:.2f} uT; " + (f"field {fit.radius:.1f} uT" if z_known
                                                else f"horizontal field {fit.horizontal_radius:.1f} uT"))
    problems = []
    if not fit.xy_determined:
        problems.append(f"x and y are not determined (standard error above {MAX_STD_ERROR_UT} uT); record again, "
                        "turning the robot through a full circle")
    elif z_known and not EARTH_FIELD_UT[0] <= fit.radius <= EARTH_FIELD_UT[1]:
        problems.append(f"a field of {fit.radius:.0f} uT is not Earth's; a magnet or soft iron nearby distorts it")
    elif not z_known and fit.horizontal_radius > EARTH_FIELD_UT[1]:
        problems.append(f"a horizontal field of {fit.horizontal_radius:.0f} uT is not Earth's; "
                        "a magnet or soft iron nearby distorts it")
    for problem in problems:
        print(f"refused: {problem}", file=sys.stderr)
    if problems:
        return 1
    if not z_known:
        print("z is not determined (the robot was not tilted); the heading does not use it, so z is left as it is")
    if args.write:
        try:
            asyncio.run(store_offset(args.write, *fit.offset[:2], fit.offset[2] if z_known else None))
        except (OSError, asyncio.TimeoutError, RuntimeError, websockets.WebSocketException) as error:
            print(f"not stored: {error}", file=sys.stderr)
            return 1
        print(f"stored on {args.write}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
