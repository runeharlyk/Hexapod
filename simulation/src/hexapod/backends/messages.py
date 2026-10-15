"""The robot's wire messages (platform_shared/message.proto), shared by its WebSocket and serial links."""
import subprocess
import sys
from pathlib import Path

COMPILE_PROTOS = Path(__file__).resolve().parents[3] / "scripts" / "compile_protos.py"
try:
    from ...platform_shared import message_pb2 as pb
except ImportError:  # gitignored and generated on first use, as src/robot/animation_files.py does
    subprocess.run([sys.executable, str(COMPILE_PROTOS)], check=True)
    from ...platform_shared import message_pb2 as pb
