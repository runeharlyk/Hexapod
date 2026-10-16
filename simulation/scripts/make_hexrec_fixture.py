"""Writes the app's .hexrec test fixture with the writer record.py uses, from synthetic ticks:

    uv run python scripts/make_hexrec_fixture.py

Beside app/tests/unit/fixtures/recording.hexrec it writes recording.frames.json: the features and every record's
host receive time and frame as hex ("" for a gap). The app's tests feed those exact frames to its writer and its
recorder, so they check the framing without depending on how ts-proto and protobuf-python encode a message.
Prints telemetry_stats.tick_timing of the written file and the median period, which the app's tests assert.
"""
import json
import sys
from pathlib import Path

import numpy as np

SIMULATION = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIMULATION))

from src.platform_shared import message_pb2  # noqa: E402
from src.sim.recording import RecordingWriter, load  # noqa: E402
from src.sim.telemetry_stats import tick_timing  # noqa: E402

FIXTURES = SIMULATION.parent / "app" / "tests" / "unit" / "fixtures"
FEATURES = {"firmwareName": "hexapod", "firmwareVersion": "0.9.0", "imu": True, "servo": True,
            "firmwareElfSha256": "ab" * 32}
PERIODS = [5000, 5001, 4999, 5000, 12000, 5003, 4998, 5000, 5000]


def tick(seq: int) -> message_pb2.TickSample:
    return message_pb2.TickSample(
        seq=seq, t_us=1_000_000 + seq * 5000, period_us=PERIODS[seq], compute_us=1200 + seq * 10,
        imu_fresh=seq % 2 == 0, rpy=[0.5, -0.25, 1.0], gyro=[0.0, 0.125, -0.5], gravity=[0.0, 0.0, 1.0],
        angles=[float(i) for i in range(18)], command=[0.0, 0.5, 0.0, 0.0, 0.25, 0.0, 0.0, 0.75, -0.5],
        command_age_ms=seq * 3, mode=4, gait=1, mag=[21.5, -4.25, 40.0], mag_fresh=seq % 3 == 0)


def batch(batch_seq: int, seqs: list[int], dropped: int) -> bytes:
    return message_pb2.Message(telemetry_batch=message_pb2.TelemetryBatch(
        batch_seq=batch_seq, dropped_ticks=dropped, ticks=[tick(s) for s in seqs])).SerializeToString()


RECORDS = [
    (2_000_000, batch(7, [0, 1, 2], 0)),
    (2_025_000, batch(8, [3, 4], 0)),
    (2_040_000, b""),
    (2_075_000, batch(10, [6, 7, 8], 2)),
]


def main() -> None:
    out = FIXTURES / "recording.hexrec"
    writer = RecordingWriter(out, FEATURES)
    for host_rx_us, frame in RECORDS:
        if frame:
            writer.write(frame, host_rx_us)
        else:
            writer.gap(host_rx_us)
    writer.close()
    sidecar = {"features": FEATURES,
               "records": [{"host_rx_us": host_rx_us, "frame": frame.hex()} for host_rx_us, frame in RECORDS]}
    (FIXTURES / "recording.frames.json").write_text(json.dumps(sidecar, indent=2) + "\n", newline="\n")

    rec = load(out)
    seq = rec.ticks["seq"].astype(np.int64)
    contiguous = np.concatenate([[False], np.diff(seq) == 1])
    period = rec.ticks["period_us"][contiguous].astype(float)
    print(tick_timing(rec), "period p50", float(np.percentile(period, 50)), "link gaps", rec.link_gaps)


if __name__ == "__main__":
    main()
