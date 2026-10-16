"""The backend's protobuf bindings are gitignored and generated, so one built before a schema gained a field must be
rebuilt rather than silently dropping that field from every message the backend sends."""
import os

import pytest

from src.hexapod.backends.messages import BINDING, PROTO_DIR, regenerate_if_stale


@pytest.fixture
def layout(tmp_path):
    proto_dir = tmp_path / "platform_shared"
    proto_dir.mkdir()
    for name in ("animation", "api", "message"):
        (proto_dir / f"{name}.proto").write_text("")
    ran = tmp_path / "ran"
    compiler = tmp_path / "compile_protos.py"
    compiler.write_text(f"open({str(ran)!r}, 'a').write('x')\n")
    binding = tmp_path / "message_pb2.py"
    binding.write_text("")
    return binding, proto_dir, compiler, ran


def _age(path, seconds_ago):
    stamp = os.stat(path).st_mtime - seconds_ago
    os.utime(path, (stamp, stamp))


def _runs(ran):
    return len(ran.read_text()) if ran.exists() else 0


def test_a_binding_older_than_any_schema_is_regenerated(layout):
    binding, proto_dir, compiler, ran = layout
    _age(binding, 100)
    _age(proto_dir / "api.proto", 50)
    regenerate_if_stale(binding, proto_dir, compiler)
    assert _runs(ran) == 1


def test_a_missing_binding_is_generated(layout):
    binding, proto_dir, compiler, ran = layout
    binding.unlink()
    regenerate_if_stale(binding, proto_dir, compiler)
    assert _runs(ran) == 1


def test_a_binding_is_kept_when_there_is_no_schema_to_compare(layout):
    binding, proto_dir, compiler, ran = layout
    for proto in proto_dir.glob("*.proto"):
        proto.unlink()
    regenerate_if_stale(binding, proto_dir, compiler)
    assert _runs(ran) == 0


def test_a_binding_newer_than_every_schema_is_kept(layout):
    binding, proto_dir, compiler, ran = layout
    for proto in proto_dir.glob("*.proto"):
        _age(proto, 100)
    regenerate_if_stale(binding, proto_dir, compiler)
    assert _runs(ran) == 0


def test_the_loaded_bindings_are_current_and_carry_the_stride_field():
    from src.platform_shared import message_pb2

    newest = max(p.stat().st_mtime for p in PROTO_DIR.glob("*.proto"))
    assert BINDING.stat().st_mtime >= newest
    assert message_pb2.ControllerInputData.DESCRIPTOR.fields_by_name["stride"].number == 7
