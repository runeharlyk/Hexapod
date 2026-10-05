from pathlib import Path
import subprocess
import sys

Import("env")

project_dir = Path(env["PROJECT_DIR"])
(project_dir / "firmware" / "data").mkdir(exist_ok=True)

sys.path.insert(0, str(project_dir / "firmware" / "scripts"))
from secrets_file import SecretsError, ensure_secrets  # noqa: E402

include_dir = project_dir / "firmware" / "include"
try:
    notice = ensure_secrets(include_dir / "secrets.example.h", include_dir / "secrets.h")
except SecretsError as error:
    sys.exit(str(error))
if notice:
    print(notice)

proto_script = project_dir / "firmware" / "scripts" / "compile_protos.py"
if proto_script.exists():
    print("Running proto compilation...")
    result = subprocess.run([sys.executable, str(proto_script)], cwd=str(project_dir))
    if result.returncode != 0:
        sys.exit("Proto compilation failed")
