from pathlib import Path
import subprocess
import sys

Import("env")

project_dir = Path(env["PROJECT_DIR"])
(project_dir / "firmware" / "data").mkdir(exist_ok=True)

proto_script = project_dir / "firmware" / "scripts" / "compile_protos.py"
if proto_script.exists():
    print("Running proto compilation...")
    result = subprocess.run([sys.executable, str(proto_script)], cwd=str(project_dir))
    if result.returncode != 0:
        sys.exit("Proto compilation failed")
