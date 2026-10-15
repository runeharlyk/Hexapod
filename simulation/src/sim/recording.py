"""Telemetry recordings from the robot (.hexrec files) as NumPy arrays.

Format: b"HEXAREC1", uint32 length and the JSON of the robot's features at the start (firmware version,
ELF hash), then records of uint64 host receive time (us), uint32 length and one encoded
socket_message.Message; a record of length 0 marks a gap where the link dropped. All integers are
little-endian. Times are the robot's esp_timer microseconds unless named host_*. Vectors are in the
firmware's body frame (X lateral, Y forward, Z up); rpy is the DMP's roll, pitch, yaw in radians.
"""
from __future__ import annotations

import json
import struct
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from src.platform_shared import message_pb2

MAGIC = b"HEXAREC1"
_LENGTH = struct.Struct("<I")
_RECORD = struct.Struct("<QI")

_SCALARS = ("seq", "t_us", "period_us", "compute_us", "imu_fresh", "command_age_ms", "mode", "gait", "mag_fresh")
_VECTORS = ("rpy", "gyro", "gravity", "angles", "command", "mag")


@dataclass
class Recording:
    features: dict
    ticks: dict[str, np.ndarray]
    batch_rx_us: np.ndarray
    link_gaps: int = 0
    missing_batches: int = 0
    missing_ticks: int = 0
    dropped_ticks: int = 0


class RecordingWriter:
    """Appends frames as they arrive, so an interrupted recording keeps everything written before."""

    def __init__(self, path, features: dict):
        self._file = open(path, "wb")
        header = json.dumps(features).encode()
        self._file.write(MAGIC + _LENGTH.pack(len(header)) + header)

    def write(self, frame: bytes, host_rx_us: int) -> None:
        self._file.write(_RECORD.pack(host_rx_us, len(frame)) + frame)
        self._file.flush()

    def gap(self, host_rx_us: int) -> None:
        self._file.write(_RECORD.pack(host_rx_us, 0))
        self._file.flush()

    def close(self) -> None:
        self._file.close()


def _read(data: bytes):
    if data[: len(MAGIC)] != MAGIC:
        raise ValueError("not a hexapod telemetry recording")
    (length,) = _LENGTH.unpack_from(data, len(MAGIC))
    start = len(MAGIC) + _LENGTH.size
    features = json.loads(data[start : start + length])
    offset = start + length

    def records():
        nonlocal offset
        while offset + _RECORD.size <= len(data):
            host_rx_us, size = _RECORD.unpack_from(data, offset)
            begin = offset + _RECORD.size
            if begin + size > len(data):
                return  # the recorder stopped in the middle of this record
            yield host_rx_us, data[begin : begin + size]
            offset = begin + size

    return features, records()


def load(path) -> Recording:
    features, records = _read(Path(path).read_bytes())
    ticks = {name: [] for name in (*_SCALARS, *_VECTORS)}
    batch_rx_us, batch_seqs = [], []
    link_gaps = dropped = 0
    for host_rx_us, frame in records:
        if not frame:
            link_gaps += 1
            continue
        message = message_pb2.Message.FromString(frame)
        if message.WhichOneof("message") != "telemetry_batch":
            continue
        batch = message.telemetry_batch
        batch_seqs.append(batch.batch_seq)
        batch_rx_us.append(host_rx_us)
        dropped = batch.dropped_ticks
        for tick in batch.ticks:
            for name in _SCALARS:
                ticks[name].append(getattr(tick, name))
            for name in _VECTORS:
                ticks[name].append(list(getattr(tick, name)))
    arrays = {name: np.asarray(values) for name, values in ticks.items()}
    seq = arrays["seq"].astype(np.int64)
    batches = np.asarray(batch_seqs, dtype=np.int64)
    # A batch the link could not take is skipped by the robot, and dropped ticks are counted there;
    # both leave holes in seq, which every statistic below must step over rather than bridge.
    return Recording(
        features=features,
        ticks=arrays,
        batch_rx_us=np.asarray(batch_rx_us),
        link_gaps=link_gaps,
        missing_batches=int(np.sum(np.diff(batches) - 1)) if len(batches) > 1 else 0,
        missing_ticks=int(np.sum(np.diff(seq) - 1)) if len(seq) > 1 else 0,
        dropped_ticks=int(dropped),
    )
