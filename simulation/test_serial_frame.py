"""The Python serial framing against platform_shared/serial_frame_vectors.json, which the firmware's
host test and the app's unit test read too, so robot_animate.py cannot drift from the robot."""
import json
from pathlib import Path

import pytest

from src.robot import serial_frame

VECTORS = json.loads(
    (Path(__file__).resolve().parents[1] / "platform_shared" / "serial_frame_vectors.json").read_text()
)["vectors"]


def decode(stream: bytes, chunk: int) -> list[str]:
    frames: list[str] = []
    decoder = serial_frame.Decoder(lambda message: frames.append(message.hex()))
    for at in range(0, len(stream), chunk):
        decoder.feed(stream[at:at + chunk])
    return frames


def test_reads_every_vector():
    assert len(VECTORS) == 7


@pytest.mark.parametrize("vector", VECTORS, ids=[v["name"] for v in VECTORS])
def test_decodes_each_vector_alike_in_any_chunking(vector):
    stream = bytes.fromhex(vector["stream"])
    for chunk in (len(stream), 1, 7):
        assert decode(stream, chunk) == vector["frames"]


def test_encodes_a_single_frame_as_the_firmware_writes_it():
    single = [v for v in VECTORS if len(v["frames"]) == 1]
    assert len(single) > 3
    for vector in single:
        assert serial_frame.encode(bytes.fromhex(vector["frames"][0])).hex() in vector["stream"]


def test_finds_the_frames_after_a_truncated_one_and_the_boot_banner():
    a, b = bytes([1, 2, 0, 3]), bytes([9, 9])
    cut = serial_frame.encode(a)[:-3]
    stream = cut + b"ESP-ROM:esp32s3-20210327\r\n" + serial_frame.encode(b) + serial_frame.encode(a)
    assert decode(stream, 5) == [b.hex(), a.hex()]


def test_refuses_a_message_over_the_limit():
    with pytest.raises(ValueError):
        serial_frame.encode(bytes(serial_frame.MAX_MESSAGE + 1))
