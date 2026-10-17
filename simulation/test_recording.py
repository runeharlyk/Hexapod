"""Telemetry recordings: the file format, the statistics the report draws from them, and record.py
against a stand-in robot socket that speaks the firmware's protocol."""
import asyncio

import numpy as np
import pytest
import websockets

import record
import report_recording
from src.platform_shared import message_pb2
from src.sim import telemetry_stats as ts
from src.sim.recording import RecordingWriter, load

BATCH = 5
FEATURES = {"firmwareVersion": "0.0.1", "firmwareElfSha256": "ab" * 32}


def batch_frame(batch_seq: int, first_seq: int, *, gyro=(0.0, 0.0, 0.0), period_us=5000, dropped=0, command=None):
    msg = message_pb2.Message()
    b = msg.telemetry_batch
    b.batch_seq, b.dropped_ticks = batch_seq, dropped
    for i in range(BATCH):
        t = b.ticks.add()
        t.seq = first_seq + i
        t.t_us = 1_000_000 + 5000 * t.seq
        t.period_us, t.compute_us, t.imu_fresh = period_us, 900, True
        t.rpy.extend([0.0, 0.0, 0.0])
        t.gyro.extend(gyro)
        t.gravity.extend([0.0, 0.0, 1.0])
        t.angles.extend([float(t.seq)] * 18)
        t.command.extend(command or [0.0] * 9)
    return msg.SerializeToString()


def test_a_recording_round_trips_with_its_holes_counted(tmp_path):
    path = tmp_path / "r.hexrec"
    w = RecordingWriter(path, FEATURES)
    w.write(batch_frame(0, 0), 10)
    w.write(batch_frame(1, 5), 20)
    w.gap(30)
    w.write(batch_frame(3, 15, dropped=2), 40)  # batch 2 and its five ticks never arrived
    w.close()

    rec = load(path)
    assert rec.features == FEATURES
    assert rec.ticks["seq"].tolist() == [*range(10), *range(15, 20)]
    assert rec.ticks["angles"].shape == (15, 18)
    assert rec.link_gaps == 1
    assert rec.missing_batches == 1
    assert rec.missing_ticks == 5
    assert rec.dropped_ticks == 2


def test_a_recording_cut_off_mid_record_keeps_what_came_before(tmp_path):
    path = tmp_path / "r.hexrec"
    w = RecordingWriter(path, FEATURES)
    w.write(batch_frame(0, 0), 10)
    w.write(batch_frame(1, 5), 20)
    w.close()
    path.write_bytes(path.read_bytes()[:-7])
    assert load(path).ticks["seq"].tolist() == list(range(5))


def test_a_file_that_is_no_recording_is_refused(tmp_path):
    path = tmp_path / "r.hexrec"
    path.write_bytes(b"LEIKAREC\x01\x00")
    with pytest.raises(ValueError):
        load(path)


def test_timing_does_not_count_a_hole_in_the_ticks_as_an_overrun(tmp_path):
    path = tmp_path / "r.hexrec"
    w = RecordingWriter(path, FEATURES)
    w.write(batch_frame(0, 0), 10)
    # After the hole the first tick's period spans the missing ticks.
    w.write(batch_frame(2, 10, period_us=5000), 20)
    w.close()
    rec = load(path)
    rec.ticks["period_us"][5] = 30_000
    timing = ts.tick_timing(rec)
    assert timing.overruns == 0
    assert timing.period_mean_us == pytest.approx(5000)


def test_the_imu_at_rest_leaves_out_motion_and_steering(tmp_path):
    rng = np.random.default_rng(1)
    path = tmp_path / "r.hexrec"
    w = RecordingWriter(path, FEATURES)
    for b in range(200):
        gyro = (0.01 + rng.normal(0, 0.003), -0.02 + rng.normal(0, 0.003), rng.normal(0, 0.003))
        w.write(batch_frame(b, b * BATCH, gyro=gyro), b)
    # Walking: fast body rates, and a stick held while the body happens to be still.
    for b in range(200, 220):
        w.write(batch_frame(b, b * BATCH, gyro=(1.0, 0.5, 0.0)), b)
    for b in range(220, 240):
        w.write(batch_frame(b, b * BATCH, command=[0.0, 0.8] + [0.0] * 7), b)
    w.close()

    imu = ts.imu_at_rest(load(path))
    assert imu.samples == 200 * BATCH
    assert imu.gyro_bias == pytest.approx([0.01, -0.02, 0.0], abs=0.002)
    assert imu.gyro_noise.max() < 0.01


def test_the_report_flags_a_bias_outside_the_trained_range(tmp_path):
    path = tmp_path / "r.hexrec"
    w = RecordingWriter(path, FEATURES)
    for b in range(40):
        w.write(batch_frame(b, b * BATCH, gyro=(0.18 + 0.001 * (b % 3), 0.0, 0.0)), b)
    w.close()
    text = report_recording.report(path)
    assert "outside the trained range: gyro bias" in text


def test_the_recorder_keeps_the_features_and_every_batch_across_a_dropped_link(tmp_path):
    sessions = []

    async def robot(socket):
        session = {"subscribed": False}
        sessions.append(session)
        async for raw in socket:
            msg = message_pb2.Message.FromString(raw)
            kind = msg.WhichOneof("message")
            if kind == "correlation_request":
                reply = message_pb2.Message()
                reply.correlation_response.correlation_id = msg.correlation_request.correlation_id
                reply.correlation_response.status_code = 200
                reply.correlation_response.features_data_response.firmware_version = "0.0.1"
                await socket.send(reply.SerializeToString())
            elif kind == "sub_notif" and msg.sub_notif.tag == record.TAG:
                session["subscribed"] = True
                first = 0 if len(sessions) == 1 else 100
                for b in range(3):
                    await socket.send(batch_frame(first // BATCH + b, first + b * BATCH))
                if len(sessions) == 1:
                    return  # the first session drops; the recorder must come back
                await asyncio.sleep(10)

    async def run():
        async with websockets.serve(robot, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            return await record.record(f"ws://127.0.0.1:{port}", str(tmp_path / "r.hexrec"), seconds=3.0)

    frames = asyncio.run(run())
    rec = load(tmp_path / "r.hexrec")
    assert frames == 6
    assert rec.features["firmwareVersion"] == "0.0.1"
    assert rec.link_gaps == 1
    assert rec.ticks["seq"][:BATCH * 3].tolist() == list(range(15))
    assert all(s["subscribed"] for s in sessions)


OK, FAILED, SKIPPED, RESTARTED = (message_pb2.ServoWrite.Value(f"SERVO_WRITE_{name}")
                                  for name in ("OK", "FAILED", "SKIPPED", "RESTARTED"))


def servo_frame(batch_seq: int, ticks: list[tuple]) -> bytes:
    """A batch of ticks given as (seq, right board's write, left board's, link_lost, write time in us)."""
    msg = message_pb2.Message()
    b = msg.telemetry_batch
    b.batch_seq = batch_seq
    for seq, right, left, lost, write_us in ticks:
        t = b.ticks.add()
        t.seq, t.t_us, t.period_us, t.compute_us = seq, 1_000_000 + 5000 * seq, 5000, 900 + write_us
        # As the firmware sends every tick, with fixed-count vectors.
        for vector, n in (("rpy", 3), ("gyro", 3), ("gravity", 3), ("angles", 18), ("command", 9), ("mag", 3)):
            getattr(t, vector).extend([0.0] * n)
        t.pwm.extend([300 + seq + s for s in range(18)])
        t.servo_write_right, t.servo_write_left, t.link_lost, t.servo_write_us = right, left, lost, write_us
    return msg.SerializeToString()


def write_servo_recording(path, ticks: list[tuple]) -> None:
    w = RecordingWriter(path, FEATURES)
    for first in range(0, len(ticks), BATCH):
        w.write(servo_frame(ticks[first][0] // BATCH, ticks[first:first + BATCH]), first)
    w.close()


def test_a_recording_loads_each_ticks_servo_write_and_dead_man_flag(tmp_path):
    path = tmp_path / "r.hexrec"
    ticks = [(seq, OK, SKIPPED if seq % 3 else FAILED, seq >= 7, 1000 + seq) for seq in range(10)]
    write_servo_recording(path, ticks)
    rec = load(path)
    assert rec.ticks["pwm"].shape == (10, 18)
    assert rec.ticks["pwm"][4].tolist() == [304 + s for s in range(18)]
    assert rec.ticks["servo_write_us"].tolist() == [1000 + seq for seq in range(10)]
    assert rec.ticks["servo_write_right"].tolist() == [OK] * 10
    assert rec.ticks["servo_write_left"].tolist() == [t[2] for t in ticks]
    assert rec.ticks["link_lost"].tolist() == [seq >= 7 for seq in range(10)]


def test_a_recording_from_before_the_servo_write_loads_and_says_so(tmp_path):
    path = tmp_path / "r.hexrec"
    w = RecordingWriter(path, FEATURES)
    w.write(batch_frame(0, 0), 10)
    w.close()
    rec = load(path)
    assert not {"pwm", "servo_write_us", "servo_write_right", "servo_write_left", "link_lost"} & rec.ticks.keys()
    assert rec.ticks["angles"].shape == (5, 18)
    assert "servo writes and dead-man stops: not in this recording" in report_recording.report(path)


# Forty ticks with the sixth batch (seq 25-29) lost: the right board fails three writes, is skipped while it backs off
# and fails its retry, which took a 21 ms bus timeout; the left board is restarted once; the dead-man stop fires at 3,
# in the lost batch (seen first at 30) and at 37.
def servo_course() -> list[tuple]:
    ticks = []
    for seq in (s for s in range(40) if not 25 <= s < 30):
        right = FAILED if seq in (10, 11, 12, 20) else SKIPPED if 13 <= seq < 20 else OK
        left = RESTARTED if seq == 22 else OK
        lost = 3 <= seq <= 6 or 30 <= seq <= 34 or seq >= 37
        ticks.append((seq, right, left, lost, 21_000 if seq == 20 else 1100))
    return ticks


def test_the_report_tells_failed_writes_from_skipped_ones_and_lists_each_dead_man_stop(tmp_path):
    path = tmp_path / "r.hexrec"
    write_servo_recording(path, servo_course())
    text = report_recording.report(path)
    assert "  right: 24 ok, 4 failed, 7 skipped while backing off, 0 restarted; failed at seq 10, 11, 12, 20" in text
    assert "  left: 34 ok, 0 failed, 0 skipped while backing off, 1 restarted" in text
    assert "longest 21000 us at seq 20" in text
    assert ("dead-man stops: 3 at seq 3, 30, 37 "
            "(30 right after a hole in the ticks: fired within it or on its first tick); "
            "12 ticks in force") in text


def test_a_dead_man_stop_from_before_the_recording_is_not_counted_as_one_it_saw(tmp_path):
    path = tmp_path / "r.hexrec"
    write_servo_recording(path, [(seq, OK, OK, seq < 4, 1100) for seq in range(10)])
    stops = ts.dead_man_stops(load(path))
    assert stops.fired_seqs.tolist() == []
    assert stops.in_force_at_start
    assert "dead-man stops: 0; one in force when the recording began; 4 ticks in force" in report_recording.report(path)
