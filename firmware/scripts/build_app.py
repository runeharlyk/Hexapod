from pathlib import Path
from os.path import exists, getmtime
import os, gzip, mimetypes, glob

Import("env")

project_dir = env["PROJECT_DIR"]
buildFlags = env.ParseFlags(env["BUILD_FLAGS"])

interface_dir = f"{project_dir}/app"
output_file = f"{project_dir}/firmware/include/WWWData.h"
source_www_dir = f"{interface_dir}/src"
build_dir = f"{interface_dir}/build"
filesystem_dir = f"{project_dir}/data"

Path(filesystem_dir).mkdir(exist_ok=True)
Path(output_file).parent.mkdir(parents=True, exist_ok=True)
mimetypes.init()

def latest_ts():
    files = [p for p in glob.glob(f"{source_www_dir}/**/*", recursive=True) if os.path.isfile(p)]
    return max(getmtime(p) for p in files) if files else 0

def needs_rebuild():
    if not exists(output_file):
        return True
    return getmtime(output_file) < latest_ts()

def pkg_mgr():
    if exists(os.path.join(interface_dir, "pnpm-lock.yaml")):
        return "pnpm"
    if exists(os.path.join(interface_dir, "yarn.lock")):
        return "yarn"
    if exists(os.path.join(interface_dir, "package-lock.json")):
        return "npm"

def build_web():
    m = pkg_mgr()
    if not m:
        raise Exception("No lock-file found. Please install dependencies for interface")
    cwd = os.getcwd()
    try:
        os.chdir(interface_dir)
        env.Execute(f"{m} install")
        env.Execute(f"{m} run build:embedded")
    finally:
        os.chdir(cwd)

def write_header():
    assets = []
    for p in sorted(Path(build_dir).rglob("*.*"), key=lambda x: x.relative_to(build_dir).as_posix()):
        rel = p.relative_to(build_dir).as_posix()
        mime = mimetypes.guess_type(rel)[0] or "application/octet-stream"
        data = gzip.compress(p.read_bytes(), mtime=0)
        assets.append((rel, mime, data))

    offsets = []
    cursor = 0
    for _, _, data in assets:
        offsets.append(cursor)
        cursor += len(data)

    with open(output_file, "w", newline="\n") as f:
        f.write("#pragma once\n")
        f.write("#include <Arduino.h>\n")
        f.write("#include <functional>\n\n")
        f.write("typedef std::function<void(const char* uri, const char* contentType, const uint8_t* content, size_t len)> RouteRegistrationHandler;\n")
        f.write("struct WWWAsset { const char* uri; const char* mime; uint32_t off; uint32_t len; };\n\n")

        f.write("static const uint8_t WWW_BLOB[] PROGMEM = {\n")
        col = 0
        for _, _, data in assets:
            for b in data:
                if col == 0:
                    f.write("\t")
                f.write(f"0x{b:02X},")
                col = (col + 1) % 16
                if col == 0:
                    f.write("\n")
        if col != 0:
            f.write("\n")
        f.write("};\n\n")

        for i,(rel,_,_) in enumerate(assets):
            f.write(f'static const char WWW_URI_{i}[] = "/{rel}";\n')
        for i,(_,mime,_) in enumerate(assets):
            f.write(f'static const char WWW_MIME_{i}[] = "{mime}";\n')
        f.write("\n")

        f.write("static const WWWAsset WWW_ASSETS[] = {\n")
        for i,(_,_,data) in enumerate(assets):
            f.write(f"\t{{WWW_URI_{i}, WWW_MIME_{i}, {offsets[i]}, {len(data)}}},\n")
        f.write("};\n")
        f.write(f"static const size_t WWW_ASSETS_COUNT = {len(assets)};\n\n")

        f.write("class WWWData { public: static void registerRoutes(RouteRegistrationHandler h) { for (size_t i = 0; i < WWW_ASSETS_COUNT; i++) { const WWWAsset& a = WWW_ASSETS[i]; h(a.uri, a.mime, WWW_BLOB + a.off, a.len); } } };\n")

print("running: build_app.py")
if needs_rebuild():
    build_web()
    write_header()
