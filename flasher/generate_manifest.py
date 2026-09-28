"""Write the ESP Web Tools manifest for the published factory image.

Usage: python generate_manifest.py <site_dir> <version>
Expects <site_dir>/firmware/esp32-wroom-camera.factory.bin and writes <site_dir>/manifest.json.
Without the image nothing is written, and the flasher page reports that no firmware is published.
The part path resolves relative to the manifest URL, which is how ESP Web Tools loads it.
"""

import json
import sys
from pathlib import Path

site_dir = Path(sys.argv[1])
version = sys.argv[2]
image = site_dir / "firmware" / "esp32-wroom-camera.factory.bin"

if not image.is_file():
    print(f"No firmware at {image}, no manifest written")
    sys.exit(0)

manifest = {
    "name": "Hexapod - ESP32-S3 WROOM with camera",
    "version": version,
    "new_install_prompt_erase": True,
    "builds": [{
        "chipFamily": "ESP32-S3",
        "parts": [{"path": f"firmware/{image.name}", "offset": 0}],
    }],
}
(site_dir / "manifest.json").write_text(json.dumps(manifest, indent=2))
print(f"Wrote manifest for version {version}")
