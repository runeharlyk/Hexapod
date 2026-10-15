"""Writes platform_shared/wire_golden.json: the number of every enum value and every message field in
platform_shared/*.proto, with each field's type.

The app and the firmware exchange these as numbers, so a reordered MOTION_STATE or GaitType, or a renumbered tag,
breaks a deployed app without either side failing to build. test_wire_golden.py compares the protos with the
committed file. A change it reports is either a mistake, or a wire change the app and the firmware must then ship
together; for the latter, regenerate the file in the same commit:

    uv run python gen_wire_golden.py
"""
import json
import subprocess
import sys
import tempfile
from pathlib import Path

from google.protobuf import descriptor_pb2

PROTO_DIR = Path(__file__).resolve().parents[1] / "platform_shared"
GOLDEN = PROTO_DIR / "wire_golden.json"
ABOUT = ("Every enum value and message field number of platform_shared/*.proto, which the app and the firmware "
         "exchange as numbers. simulation/test_wire_golden.py fails when the protos differ; after an intentional "
         "wire change, which the app and the firmware must ship together, run "
         "`uv run python gen_wire_golden.py` in simulation/ and commit the result with it.")
TYPE_NAMES = {number: name.removeprefix("TYPE_").lower()
              for name, number in descriptor_pb2.FieldDescriptorProto.Type.items()}
REPEATED = descriptor_pb2.FieldDescriptorProto.LABEL_REPEATED


def _descriptors() -> descriptor_pb2.FileDescriptorSet:
    """Compiled from the .proto files themselves, so no stale generated module can mask a change."""
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "protos.pb"
        protos = sorted(str(p) for p in PROTO_DIR.glob("*.proto"))
        subprocess.run([sys.executable, "-m", "grpc_tools.protoc", f"-I{PROTO_DIR}", f"--descriptor_set_out={out}",
                        *protos], check=True)
        return descriptor_pb2.FileDescriptorSet.FromString(out.read_bytes())


def _add_enum(enums: dict, scope: str, enum) -> None:
    enums[f"{scope}.{enum.name}"] = {v.name: v.number for v in enum.value}


def _add_message(enums: dict, messages: dict, scope: str, message) -> None:
    name = f"{scope}.{message.name}"
    messages[name] = {f.name: f"{f.number} {'repeated ' if f.label == REPEATED else ''}"
                              f"{f.type_name.lstrip('.') or TYPE_NAMES[f.type]}" for f in message.field}
    for nested in message.nested_type:
        _add_message(enums, messages, name, nested)
    for enum in message.enum_type:
        _add_enum(enums, name, enum)


def wire_numbers() -> dict:
    """{"enums": {qualified enum: {value: number}}, "messages": {qualified message: {field: "number type"}}}."""
    enums, messages = {}, {}
    for file in _descriptors().file:
        for enum in file.enum_type:
            _add_enum(enums, file.package, enum)
        for message in file.message_type:
            _add_message(enums, messages, file.package, message)
    return {"enums": dict(sorted(enums.items())), "messages": dict(sorted(messages.items()))}


def main() -> None:
    GOLDEN.write_text(json.dumps({"about": ABOUT, **wire_numbers()}, indent=1) + "\n", newline="\n")
    print(f"wrote {GOLDEN}")


if __name__ == "__main__":
    main()
