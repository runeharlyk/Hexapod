"""The robot's wire messages (platform_shared/message.proto), shared by its WebSocket and serial links."""
import subprocess
import sys
from pathlib import Path

SIMULATION = Path(__file__).resolve().parents[3]
COMPILE_PROTOS = SIMULATION / "scripts" / "compile_protos.py"
PROTO_DIR = SIMULATION.parent / "platform_shared"
BINDING = SIMULATION / "src" / "platform_shared" / "message_pb2.py"


def regenerate_if_stale(binding: Path = BINDING, proto_dir: Path = PROTO_DIR, compile_protos: Path = COMPILE_PROTOS):
    """Generate the gitignored protobuf bindings when they are missing or older than any schema, as the native
    library is rebuilt for a newer firmware source. A binding from before a field was added drops it on every send."""
    newest_proto = max(p.stat().st_mtime for p in proto_dir.glob("*.proto"))
    if not binding.exists() or newest_proto > binding.stat().st_mtime:
        subprocess.run([sys.executable, str(compile_protos)], check=True)


regenerate_if_stale()
from ...platform_shared import message_pb2 as pb  # noqa: E402
