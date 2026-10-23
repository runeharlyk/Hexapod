"""The compass hard-iron fit: a recording of the robot turned through every orientation gives back the
offset the field was shifted by, a poor recording is refused, and --write stores the offset on the robot
without disturbing its other peripheral settings."""
import asyncio

import numpy as np
import pytest
import websockets

import calibrate_compass
from src.platform_shared import message_pb2
from src.sim.compass_fit import fit_recording, fit_sphere
from src.sim.recording import RecordingWriter, load

TRUE_OFFSET = np.array([60.0, -85.0, 30.0])  # uT, a hard iron as strong as the Earth's field
EARTH_UT = 48.0
BATCH = 5
FEATURES = {"firmwareVersion": "0.0.1"}


def directions(n, rng):
    v = rng.normal(size=(n, 3))
    return v / np.linalg.norm(v, axis=1, keepdims=True)


def rotation(axis, angle):
    c, s = np.cos(angle), np.sin(angle)
    matrices = {"x": [[1, 0, 0], [0, c, -s], [0, s, c]], "y": [[c, 0, s], [0, 1, 0], [-s, 0, c]],
                "z": [[c, -s, 0], [s, c, 0], [0, 0, 1]]}
    return np.array(matrices[axis])


def inclined_readings(n, rng, tilt_deg, yaw_span_deg=360.0, field=EARTH_UT, inclination_deg=70.0, noise=0.3):
    """A robot yawed through `yaw_span_deg` and rolled and pitched up to `tilt_deg` in the field of a site where it
    points `inclination_deg` below the horizon (the world frame has z up, so the field's z is negative)."""
    inclination = np.radians(inclination_deg)
    world = field * np.array([np.cos(inclination), 0.0, -np.sin(inclination)])
    tilt = np.radians(tilt_deg)
    readings = []
    for _ in range(n):
        yaw = np.radians(rng.uniform(0.0, yaw_span_deg))
        body = rotation("z", yaw) @ rotation("x", rng.uniform(-tilt, tilt)) @ rotation("y", rng.uniform(-tilt, tilt))
        readings.append(body.T @ world)
    return TRUE_OFFSET + np.array(readings) + rng.normal(scale=noise, size=(n, 3))


def write_recording(path, readings, repeat=1):
    """Each compass reading is polled `repeat` times, as the 40 Hz poll sees the 15 Hz chip."""
    polls = np.repeat(readings, repeat, axis=0)
    polls = polls[: len(polls) // BATCH * BATCH]
    writer = RecordingWriter(path, FEATURES)
    for first in range(0, len(polls), BATCH):
        message = message_pb2.Message()
        message.telemetry_batch.batch_seq = first // BATCH
        for seq in range(first, first + BATCH):
            tick = message.telemetry_batch.ticks.add(seq=seq, mag_fresh=True)
            tick.mag.extend(polls[seq].tolist())
        writer.write(message.SerializeToString(), host_rx_us=seq)
    writer.close()
    return load(path)


def test_the_offset_is_recovered_from_a_robot_turned_through_every_orientation(tmp_path):
    rng = np.random.default_rng(1)
    readings = TRUE_OFFSET + EARTH_UT * directions(400, rng) + rng.normal(scale=0.3, size=(400, 3))
    fit = fit_recording(write_recording(tmp_path / "turned.hexrec", readings))
    np.testing.assert_allclose(fit.offset, TRUE_OFFSET, atol=0.2)
    assert fit.radius == pytest.approx(EARTH_UT, abs=0.2)
    assert fit.residual < 0.5
    assert fit.xy_determined and fit.z_determined


def test_a_reading_polled_several_times_counts_once(tmp_path):
    rng = np.random.default_rng(2)
    readings = TRUE_OFFSET + EARTH_UT * directions(100, rng) + rng.normal(scale=0.3, size=(100, 3))
    fit = fit_recording(write_recording(tmp_path / "polled.hexrec", readings, repeat=3))
    assert fit.samples == 100


def test_ticks_without_a_compass_read_are_ignored(tmp_path):
    rng = np.random.default_rng(3)
    readings = TRUE_OFFSET + EARTH_UT * directions(100, rng) + rng.normal(scale=0.3, size=(100, 3))
    rec = write_recording(tmp_path / "gaps.hexrec", readings)
    rec.ticks["mag_fresh"][::2] = False
    assert fit_recording(rec).samples == 50


# The Earth's field points steeply down, so a robot yawed on the floor leaves the readings on one horizontal circle:
# x and y, all the heading uses, are fixed; z and the radius are not.
def test_a_robot_only_yawed_fixes_x_and_y_but_not_z():
    fit = fit_sphere(inclined_readings(400, np.random.default_rng(4), tilt_deg=0.0))
    assert fit.xy_determined
    assert not fit.z_determined
    np.testing.assert_allclose(fit.offset[:2], TRUE_OFFSET[:2], atol=0.2)
    assert fit.offset[2] == 0.0
    assert fit.horizontal_radius == pytest.approx(EARTH_UT * np.cos(np.radians(70.0)), abs=0.3)


# Yawed and tilted up to 30 or 45 degrees in the same inclined field, the readings cluster in direction, yet the
# centre is well determined on every axis.
@pytest.mark.parametrize("tilt_deg", [30.0, 45.0, 90.0])
def test_a_tilted_robot_in_an_inclined_field_fixes_every_axis(tilt_deg):
    fit = fit_sphere(inclined_readings(400, np.random.default_rng(5), tilt_deg))
    assert fit.z_determined
    np.testing.assert_allclose(fit.offset, TRUE_OFFSET, atol=0.5)
    assert fit.radius == pytest.approx(EARTH_UT, abs=0.5)


def test_a_robot_yawed_through_a_sliver_fixes_nothing():
    fit = fit_sphere(inclined_readings(400, np.random.default_rng(6), tilt_deg=0.0, yaw_span_deg=15.0))
    assert not fit.xy_determined
    assert not fit.z_determined


def test_too_few_samples_are_refused():
    with pytest.raises(ValueError, match="compass samples"):
        fit_sphere(np.ones((5, 3)))


def test_a_recording_without_a_compass_is_refused(tmp_path):
    rec = write_recording(tmp_path / "none.hexrec", np.zeros((50, 3)))
    rec.ticks["mag_fresh"][:] = False
    with pytest.raises(ValueError, match="compass samples"):
        fit_recording(rec)


def test_a_recording_that_never_streamed_the_compass_is_refused(tmp_path, capsys):
    path = tmp_path / "old.hexrec"
    writer = RecordingWriter(path, FEATURES)
    message = message_pb2.Message()
    for seq in range(BATCH):
        message.telemetry_batch.ticks.add(seq=seq)
    writer.write(message.SerializeToString(), host_rx_us=0)
    writer.close()
    assert calibrate_compass.main([str(path)]) == 1
    assert "compass samples" in capsys.readouterr().err


def run_cli(path, *extra):
    return calibrate_compass.main([str(path), *extra])


def test_a_good_recording_is_accepted(tmp_path, capsys):
    rng = np.random.default_rng(5)
    readings = TRUE_OFFSET + EARTH_UT * directions(300, rng)
    path = tmp_path / "good.hexrec"
    write_recording(path, readings)
    assert run_cli(path) == 0
    assert "offset 60.0" in capsys.readouterr().out


def test_a_field_that_is_not_the_earths_is_refused(tmp_path, capsys):
    rng = np.random.default_rng(6)
    path = tmp_path / "magnet.hexrec"
    write_recording(path, TRUE_OFFSET + 300.0 * directions(300, rng))
    assert run_cli(path) == 1
    assert "not Earth's" in capsys.readouterr().err


def test_a_yawed_robot_is_accepted_and_z_is_reported_as_left_alone(tmp_path, capsys):
    path = tmp_path / "yawed.hexrec"
    write_recording(path, inclined_readings(300, np.random.default_rng(7), tilt_deg=0.0))
    assert run_cli(path) == 0
    out = capsys.readouterr().out
    assert "z is not determined" in out
    assert "horizontal field" in out


def test_a_robot_yawed_through_a_sliver_is_refused_with_its_reason(tmp_path, capsys):
    path = tmp_path / "sliver.hexrec"
    write_recording(path, inclined_readings(300, np.random.default_rng(8), tilt_deg=0.0, yaw_span_deg=15.0))
    assert run_cli(path) == 1
    assert "x and y are not determined" in capsys.readouterr().err


def test_a_yawed_field_stronger_than_the_earths_is_refused(tmp_path, capsys):
    path = tmp_path / "yawed-magnet.hexrec"
    write_recording(path, inclined_readings(300, np.random.default_rng(9), tilt_deg=0.0, field=300.0))
    assert run_cli(path) == 1
    assert "horizontal field" in capsys.readouterr().err


def serve_robot(updates, stored_z=0.0, update_status=200, frames=None, topic=True):
    """A stand-in robot holding I2C settings and a z offset, serving them as the firmware serves a state topic (the
    value, then the subscribe as the ack; without `topic`, the ack alone, as a firmware from before the topics) and
    answering no request for them; `updates` collects the settings it is sent, `frames` the kinds of the frames it gets."""
    tag = message_pb2.Message.DESCRIPTOR.fields_by_name["peripheral_settings"].number

    async def robot(socket):
        async for raw in socket:
            message = message_pb2.Message.FromString(raw)
            kind = message.WhichOneof("message")
            if frames is not None:
                frames.append(kind)
            if kind == "sub_notif" and message.sub_notif.tag == tag:
                if topic:
                    stored = message_pb2.Message().peripheral_settings
                    stored.sda, stored.scl, stored.frequency, stored.mag_offset_z = 47, 21, 100000, stored_z
                    await socket.send(message_pb2.Message(peripheral_settings=stored).SerializeToString())
                await socket.send(raw)
            elif kind == "correlation_request":
                request = message.correlation_request
                reply = message_pb2.Message()
                reply.correlation_response.correlation_id = request.correlation_id
                if request.WhichOneof("request") == "peripheral_settings_update":
                    updates.append(request.peripheral_settings_update)
                    reply.correlation_response.status_code = update_status
                await socket.send(reply.SerializeToString())

    return robot


def write_to_robot(path, robot):
    async def run():
        async with websockets.serve(robot, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            return await asyncio.to_thread(run_cli, path, "--write", f"127.0.0.1:{port}")

    return asyncio.run(run())


def test_write_stores_the_offset_and_keeps_the_other_settings(tmp_path):
    rng = np.random.default_rng(8)
    path = tmp_path / "good.hexrec"
    write_recording(path, TRUE_OFFSET + EARTH_UT * directions(300, rng) + rng.normal(scale=0.2, size=(300, 3)))
    updates = []
    assert write_to_robot(path, serve_robot(updates, stored_z=12.0)) == 0
    (sent,) = updates
    assert (sent.sda, sent.scl, sent.frequency) == (47, 21, 100000)
    np.testing.assert_allclose([sent.mag_offset_x, sent.mag_offset_y, sent.mag_offset_z], TRUE_OFFSET, atol=0.3)


def test_the_offset_is_written_over_the_settings_read_from_the_topic(tmp_path):
    path = tmp_path / "good.hexrec"
    write_recording(path, TRUE_OFFSET + EARTH_UT * directions(300, np.random.default_rng(11)))
    updates, frames = [], []
    assert write_to_robot(path, serve_robot(updates, frames=frames)) == 0
    assert frames == ["sub_notif", "unsub_notif", "correlation_request"]
    assert (updates[0].sda, updates[0].scl, updates[0].frequency) == (47, 21, 100000)


def test_a_robot_without_the_settings_topic_is_named_and_nothing_is_written(tmp_path, capsys):
    path = tmp_path / "good.hexrec"
    write_recording(path, TRUE_OFFSET + EARTH_UT * directions(300, np.random.default_rng(12)))
    updates = []
    assert write_to_robot(path, serve_robot(updates, topic=False)) == 1
    assert "its firmware is older than the settings topics" in capsys.readouterr().err
    assert updates == []


def test_write_of_a_yawed_robot_keeps_the_z_offset_the_robot_holds(tmp_path):
    path = tmp_path / "yawed.hexrec"
    write_recording(path, inclined_readings(300, np.random.default_rng(10), tilt_deg=0.0))
    updates = []
    assert write_to_robot(path, serve_robot(updates, stored_z=12.0)) == 0
    (sent,) = updates
    np.testing.assert_allclose([sent.mag_offset_x, sent.mag_offset_y], TRUE_OFFSET[:2], atol=0.2)
    assert sent.mag_offset_z == 12.0


def test_a_robot_that_refuses_the_offset_fails_the_write(tmp_path, capsys):
    path = tmp_path / "good.hexrec"
    write_recording(path, TRUE_OFFSET + EARTH_UT * directions(300, np.random.default_rng(9)))
    assert write_to_robot(path, serve_robot([], update_status=400)) == 1
    assert "rejected" in capsys.readouterr().err
