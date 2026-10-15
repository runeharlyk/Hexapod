"""Protobuf messages on the robot's native USB port, framed as the firmware frames them
(firmware/include/communication/serial_frame.h, app/src/lib/transport/serial-framing.ts).

A frame is 0x00, COBS(version, message, CRC-16/CCITT-FALSE big-endian), 0x00. Bytes outside a valid
frame (the ROM's boot banner after a reset) are dropped, and a delimiter that ends no valid frame
starts the next one, so a reader that starts mid-stream finds its way. platform_shared/
serial_frame_vectors.json pins all three implementations.
"""
from typing import Callable

VERSION = 1
MAX_MESSAGE = 4096
_MAX_BODY = 1 + MAX_MESSAGE + 2
# A COBS code byte for every 254 body bytes and one more, as serial_frame.h counts them.
_MAX_ENCODED = _MAX_BODY + _MAX_BODY // 254 + 1


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: polynomial 0x1021, initial 0xFFFF, not reflected."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode(message: bytes) -> bytes:
    """The bytes to write for one message, delimiters included."""
    if len(message) > MAX_MESSAGE:
        raise ValueError(f"a serial frame carries at most {MAX_MESSAGE} bytes, not {len(message)}")
    body = bytes([VERSION]) + message
    crc = crc16(body)
    body += bytes([crc >> 8, crc & 0xFF])

    out = bytearray([0])
    code_at = len(out)
    out.append(1)
    for byte in body:
        if byte:
            out.append(byte)
            out[code_at] += 1
        if byte == 0 or out[code_at] == 0xFF:
            code_at = len(out)
            out.append(1)
    out.append(0)
    return bytes(out)


def _unframe(segment: bytes) -> bytes | None:
    body = bytearray()
    i = 0
    while i < len(segment):
        code = segment[i]
        i += 1
        if i + code - 1 > len(segment):
            return None
        body += segment[i:i + code - 1]
        i += code - 1
        if code != 0xFF and i < len(segment):
            body.append(0)
    if len(body) < 3 or body[0] != VERSION or len(body) - 3 > MAX_MESSAGE:
        return None
    if crc16(bytes(body[:-2])) != (body[-2] << 8 | body[-1]):
        return None
    return bytes(body[1:-2])


class Decoder:
    """Splits a byte stream, fed in pieces of any size, into messages."""

    def __init__(self, on_frame: Callable[[bytes], None]):
        self._on_frame = on_frame
        self._in_frame = False
        self._segment = bytearray()

    def feed(self, data: bytes) -> None:
        for byte in data:
            if not self._in_frame:
                if byte == 0:
                    self._in_frame = True
                    self._segment.clear()
                continue
            if byte:
                self._segment.append(byte)
                if len(self._segment) > _MAX_ENCODED:
                    self._in_frame = False
                    self._segment.clear()
                continue
            if not self._segment:
                continue
            message = _unframe(bytes(self._segment))
            self._segment.clear()
            if message is not None:
                self._on_frame(message)
                self._in_frame = False
