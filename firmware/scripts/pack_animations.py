#!/usr/bin/env python3
"""Packs animations/*.json into firmware/data/animations/*.pb for the LittleFS image.

Runs as a PlatformIO pre-script on every build and standalone. Uses the PlatformIO python, which
pre_build.py already provisions with protobuf and grpcio-tools; the generated animation_pb2 module
lands under .pio so nothing is written into the source tree.
"""
import subprocess
import sys
from pathlib import Path


try:
    ROOT = Path(__file__).resolve().parents[2]
except NameError:  # PlatformIO execs pre-scripts without __file__
    Import("env")  # noqa: F821 - injected by PlatformIO
    ROOT = Path(env["PROJECT_DIR"])  # noqa: F821


def project_root() -> Path:
    return ROOT


def generated_module(root: Path):
    # Inside the simulation (its tests import this script) the generated module already exists and a
    # second copy would collide in the protobuf descriptor pool, so that one wins when importable.
    try:
        from src.platform_shared import animation_pb2
        return animation_pb2
    except ImportError:
        pass
    out = root / ".pio" / "animation_proto"
    out.mkdir(parents=True, exist_ok=True)
    proto = root / "platform_shared" / "animation.proto"
    stamp = out / "animation_pb2.py"
    if not stamp.exists() or stamp.stat().st_mtime < proto.stat().st_mtime:
        subprocess.run([sys.executable, "-m", "grpc_tools.protoc", f"-I{proto.parent}",
                        f"--python_out={out}", str(proto)], check=True)
    sys.path.insert(0, str(out))
    import animation_pb2  # noqa: E402
    return animation_pb2


def pack(src_dir: Path, dst_dir: Path) -> list[Path]:
    pb = generated_module(project_root())
    from google.protobuf import json_format
    dst_dir.mkdir(parents=True, exist_ok=True)
    for stale in dst_dir.glob("*.pb"):
        stale.unlink()
    written = []
    for src in sorted(src_dir.glob("*.json")):
        msg = json_format.Parse(src.read_text(), pb.Animation())
        if msg.schema != 1 or not msg.keyframes:
            raise SystemExit(f"{src.name}: schema must be 1 and keyframes non-empty")
        if msg.name != src.stem:
            raise SystemExit(f"{src.name}: name '{msg.name}' must equal the file stem")
        dst = dst_dir / f"{src.stem}.pb"
        dst.write_bytes(msg.SerializeToString())
        written.append(dst)
    print(f"animations: packed {len(written)} file(s) -> {dst_dir}")
    return written


def main() -> None:
    root = project_root()
    pack(root / "animations", root / "firmware" / "data" / "animations")


main()
