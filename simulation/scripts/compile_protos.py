"""Generate Python protobuf modules for the simulation from platform_shared/*.proto.

All three schemas are generated. protoc emits absolute `import x_pb2` lines for imported files,
which break inside the src.platform_shared package, so they are rewritten to package-relative ones.
"""
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PROTO_DIR = ROOT / "platform_shared"
OUT_DIR = ROOT / "simulation" / "src" / "platform_shared"
PROTO_FILES = ["animation.proto", "api.proto", "message.proto"]
IMPORT = re.compile(r"^import (\w+_pb2) as (\w+)$", re.MULTILINE)


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    (OUT_DIR / "__init__.py").touch()
    cmd = [sys.executable, "-m", "grpc_tools.protoc", f"-I{PROTO_DIR}", f"--python_out={OUT_DIR}"]
    cmd += [str(PROTO_DIR / f) for f in PROTO_FILES]
    subprocess.run(cmd, check=True)
    for f in PROTO_FILES:
        module = OUT_DIR / f.replace(".proto", "_pb2.py")
        module.write_text(IMPORT.sub(r"from . import \1 as \2", module.read_text()))
    print(f"protoc: {', '.join(PROTO_FILES)} -> {OUT_DIR}")


if __name__ == "__main__":
    main()
