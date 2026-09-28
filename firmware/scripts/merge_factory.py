"""Post-build step: merge bootloader, partition table, OTA data and app into firmware.factory.bin.

The factory image flashes at offset 0 in one write, which is what the web flasher (flasher/) and
the release workflow publish. Offsets come from the project partition table and the IDF-generated
flasher_args.json, so a partition layout change needs no edit here.
"""

import csv
import json
from os.path import join

Import("env")  # noqa: F821 - injected by PlatformIO

platform = env.PioPlatform()
esptool = join(platform.get_package_dir("tool-esptoolpy"), "esptool.py")


def partition_offsets(csv_path):
    offsets = {}
    with open(csv_path) as f:
        for row in csv.reader(f):
            if len(row) < 4 or row[0].startswith("#"):
                continue
            name, kind, subtype, offset = (cell.strip() for cell in row[:4])
            if kind == "data" and subtype == "ota":
                offsets.setdefault("otadata", offset)
            if kind == "app":
                offsets.setdefault("app", offset)
    return offsets


def merge(source, target, env):
    build_dir = env.subst("$BUILD_DIR")
    with open(join(build_dir, "flasher_args.json")) as f:
        flasher_args = json.load(f)
    layout = partition_offsets(env.subst(env.BoardConfig().get("build.partitions")))

    parts = [
        (flasher_args["bootloader"]["offset"], join(build_dir, "bootloader.bin")),
        (flasher_args["partition-table"]["offset"], join(build_dir, "partitions.bin")),
        (layout["otadata"], join(build_dir, "ota_data_initial.bin")),
        (layout["app"], join(build_dir, env.subst("${PROGNAME}.bin"))),
    ]
    # esptool 5 spells its options with dashes; the IDF writes them with underscores.
    flash_args = [arg.replace("_", "-") if arg.startswith("--") else arg
                  for arg in flasher_args["write_flash_args"]]
    output = join(build_dir, "firmware.factory.bin")
    cmd = [env.subst("$PYTHONEXE"), esptool, "--chip", flasher_args["extra_esptool_args"]["chip"],
           "merge-bin", "-o", output, *flash_args]
    for offset, path in parts:
        cmd += [offset, path]
    if env.Execute(env.VerboseAction(" ".join(f'"{c}"' for c in cmd), "Merging factory image")):
        env.Exit(1)


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", merge)
