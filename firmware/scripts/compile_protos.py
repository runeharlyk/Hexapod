#!/usr/bin/env python3
# Generate nanopb C sources from platform_shared/*.proto into firmware/src/platform_shared.
# The same .proto files are compiled by the app with ts-proto, keeping one wire schema.
import subprocess
import sys
from pathlib import Path


def ensure_protobuf_installed():
    # nanopb_generator.py runs protoc through grpc_tools; google.protobuf alone is not enough.
    try:
        import grpc_tools.protoc  # noqa: F401
        return True
    except ImportError:
        print("Installing protobuf dependencies...")
        result = subprocess.run(
            [sys.executable, "-m", "pip", "install", "protobuf", "grpcio-tools"],
            capture_output=True, text=True,
        )
        if result.returncode != 0:
            print(f"Failed to install protobuf: {result.stderr}")
            return False
        return True


def project_root():
    # firmware/scripts/compile_protos.py -> firmware/scripts -> firmware -> repo root
    return Path(__file__).parent.parent.parent


def compile_nanopb():
    root = project_root()
    proto_dir = root / "platform_shared"
    output_dir = root / "firmware" / "src" / "platform_shared"
    nanopb_gen = root / "submodules" / "nanopb" / "generator" / "nanopb_generator.py"

    output_dir.mkdir(parents=True, exist_ok=True)
    proto_files = sorted(proto_dir.glob("*.proto"))
    if not proto_files:
        print(f"No .proto files in {proto_dir}")
        return False

    cmd = [sys.executable, str(nanopb_gen), "-I", str(proto_dir), "-D", str(output_dir)]
    cmd += [str(f) for f in proto_files]
    print(f"nanopb: {' '.join(str(f.name) for f in proto_files)} -> {output_dir}")

    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print("Error compiling protos:")
        print(result.stdout)
        print(result.stderr)
        return False
    print(f"Compiled {len(proto_files)} proto file(s)")
    return True


def main():
    if not ensure_protobuf_installed():
        sys.exit(1)
    if not compile_nanopb():
        sys.exit(1)
    print("Proto compilation complete")


if __name__ == "__main__":
    main()
