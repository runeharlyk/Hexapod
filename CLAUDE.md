# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

A 6-legged (18-servo) hexapod robot built on an ESP32-S3. The repo has three independent but related parts:

- **`firmware/`** — ESP32 firmware (C++ on ESP-IDF 5.4, built with PlatformIO via the pioarduino platform fork). Runs the gait engine, kinematics, sensor reading, and the WiFi/BLE/WebSocket communication stack. This is the source of truth for gait and mode.
- **`app/`** — SvelteKit web controller (TypeScript). Deployed to GitHub Pages, also embeddable into the firmware. Talks to the robot over BLE or WebSocket using protobuf messages generated from `platform_shared/*.proto`.
- **`simulation/`** — Python MuJoCo simulation + RL training (`train_mj.py`) for sim-to-real transfer to the robot; managed with `uv`. A trained policy is exported to a dependency-free C++ header (`export_policy.py`) and embedded in the firmware (`WALK_NN` mode).

The root `animations/` directory is the bundled animation library (JSON in the `platform_shared/animation.proto` schema) shared by all three platforms.

The NumPy kinematics/gait (`simulation/src/robot/firmware_gait.py`) is a faithful port of the firmware kinematics/gait (`firmware/include/kinematics.h`, `gait.h`) and is the sim-to-real deploy target — when changing motion math, keep both in sync.
Replay, the speed model and the Python `Robot`'s simulation run no port: `src/sim/firmware_loop.py` drives MuJoCo with the firmware's own code, built as a native library by `src/sim/virtual_robot.py` on first use (a C++20 compiler as `$CXX`, else `g++`).

## Commands

### Firmware (run from repo root)
```sh
pio run                         # build the only env, esp32-wroom-camera (ESP32-S3 WROOM cam)
pio run -t upload               # build + flash firmware
pio run -t uploadfs             # build + flash the LittleFS filesystem image (firmware/data -> /config etc.)
pio test -e native              # host unit tests for the gait/kinematics math (firmware/test/)
pio device monitor              # serial monitor @ 115200 with esp32 exception decoder
```
`pio` on PATH may be an older PlatformIO core that the pioarduino platform rejects; use `~/.platformio/penv/Scripts/pio` (>= 6.1.18). After switching platform versions, clear `.pio/build/<env>` — a stale `CMakeCache.txt` points at tool paths that no longer exist.

The `native` test env needs a C++20 host compiler on PATH (`g++` >= 10, or set `CXX`); `utils/math_utils.h` falls back to a portable matrix multiply when ESP-DSP is absent.

Three pre-scripts run on every firmware build.
`firmware/scripts/pre_build.py` regenerates the nanopb sources from `platform_shared/*.proto` (needs python with `grpcio-tools`), and `firmware/scripts/pack_animations.py` packs the root `animations/*.json` into `firmware/data/animations/*.pb` for the LittleFS image.
`firmware/scripts/build_app.py` bakes the Svelte app into `firmware/include/WWWData.h` when `EMBED_WEBAPP=1`.
By default `EMBED_WEBAPP=0` (see `firmware/features.ini`), so the app is served separately and the firmware build does not require Node.

### Web app (run from `app/`)
```sh
pnpm install
pnpm dev                # vite dev server (--host)
pnpm build              # production build -> app/build (used for GitHub Pages deploy)
pnpm build:embedded     # build with VITE_USE_HOST_NAME=true (for embedding into firmware)
pnpm virtual-robot      # compile the firmware's motion code to WebAssembly for the simulation view (needs Emscripten; also run by dev/build)
pnpm check              # svelte-check type checking
pnpm lint               # prettier --check + eslint
pnpm format             # prettier --write
pnpm test               # integration (playwright) + unit (vitest)
pnpm test:unit          # vitest only
pnpm test:unit -- <file># run a single test file, e.g. tests/unit/throttler.spec.ts
pnpm test:integration   # playwright only
```

### Simulation (run from `simulation/`, managed with `uv`)
```sh
uv sync                                      # install deps (MuJoCo + Stable-Baselines3 + torch)
uv run python src/resources/build_model.py   # regenerate MuJoCo model.xml from firmware geometry
uv run python replay_gait.py                 # replay the classical firmware gait in MuJoCo (no RL)
uv run python train_mj.py --smoke            # RL training (SB3 PPO); drop --smoke for a full run
uv run python eval_policy.py --run <name>    # evaluate / visualize a trained policy
uv run python export_policy.py --run <name>  # export trained actor as a C++ header for the firmware
uv run python optimize_gait.py               # retune analytic command->gait coefficients
uv run pytest -q                             # firmware/sim gait parity tests (test_firmware_gait_parity.py)
uv run python check_animation.py             # run every bundled animation through the servo model
uv run python gen_animation_fixtures.py      # regenerate the animation parity fixtures
uv run python scripts/compile_protos.py      # regenerate src/platform_shared/animation_pb2.py (also runs on first import)
uv run python robot_animate.py --port <COMx> <command>  # upload/validate/play/stop/watch animations over native USB
```
See `simulation/README.md` for the full control-mode and training-flag reference.

## Firmware architecture

`docs/firmware.md` is the firmware architecture: tasks, EventBus, motion pipeline, transports, persistence, OTA, RAM budget and core-dump decoding, with the file for each component.
Read it before changing the firmware; what follows is only what an agent needs on top of it.

- Discarding the `Handle` that `EventBus<T>::subscribe()` returns unsubscribes on that line, silently; store it in a member, or use `consume()` for a program-lifetime subscription.
- EventBus workers run on PSRAM stacks, so a subscriber must never touch flash (no LittleFS or NVS read or write); hand such work to another task.
- New dynamic state for the app follows the observable-state pattern: the producer publishes on change, `observeStatus<T>()` in `main.cpp` bridges it, the app requests the current value once and then follows pushes.
  Never a timer push; settings stay request/response only.
- `MOTION_STATE`/`GaitType` cross the wire as enum positions, so firmware and app must be deployed together when those enums change.
  `simulation/test_wire_golden.py` fails on any renumbered enum value or field until `simulation/gen_wire_golden.py` regenerates `platform_shared/wire_golden.json`.
- Never hand-edit `firmware/include/gait_tuned.h` (from `simulation/export_gait.py`) or `firmware/include/policy/hexapod_policy.h` (from `simulation/export_policy.py`); regenerate them.
- Internal DMA RAM is the binding limit: measure with `logHeap()`, not `esp_get_free_heap_size()`, and delete the generated `sdkconfig.esp32-wroom-camera` after editing `sdkconfig.defaults`, or the change is silently ignored.
- The Arduino-only classic-ESP32 envs were removed with the IDF port; recover them from the `main` branch if a classic ESP32 is needed again.
  No TensorFlow Lite or ESP-DL component is part of the build (a local `firmware/components/esp-dl/` clone is gitignored and unreferenced).

## Web app architecture

SvelteKit (Svelte 5) + Tailwind/daisyUI, static-adapter SPA. Routes under `app/src/routes/` map to controller, `animations` (library and editor), connection, bluetooth, per-peripheral settings (servo/imu/camera/i2c), `wifi/*` (sta/ap/mdns) and `system/*` (status/filesystem/metrics/update) pages. `app/src/lib/` holds the shared logic: `transport/` (BLE, WebSocket and Web Serial byte-pipe implementations of `ITransport`, plus `databroker.ts` for tag-based pub/sub over the one active transport (serial > websocket > ble), `request()` correlation RPC and ping/pong), `proto-api.ts` (`resolveUrl` and IP helpers for the HTTP endpoints), `kinematic.ts` (the forward kinematics, which the firmware has none of, for drawing the robot from joint angles; tests check it against the firmware's inverse kinematics; the 3D view and the simulation view animate the robot with the firmware's own code, `simulation/virtual-robot.ts`), `sceneBuilder.ts` + `Visualization.svelte` (three.js URDF rendering), and `stores/` for app state. The controller sends `Command` messages and subscribes to telemetry by message tag.
The app's protobuf types are generated by ts-proto (`app/scripts/compile_protos.js`, output in `app/src/lib/platform_shared/`, gitignored) as a build pre-step; `request()` correlation calls time out after 15 s.
`app/src/lib/animation/` holds the animation document model (a TS port of `simulation/src/robot/animation.py`'s validator) and `engine.ts`, which runs the firmware's evaluator and player compiled to WebAssembly; it is tested against the shared fixtures in `animations/fixtures/` and backs the `/animations` route's library and editor.

## Further documentation

- `docs/connectivity.md` — how the app reaches the robot: the browser origin constraints that gate every transport, provisioning options, the `NET_STATUS`/`NET_COMMAND` topics, and the ranked plan. **Read before proposing any connection flow.**
- `docs/animation.md` — the animation system as implemented (file format, evaluator rules, firmware mode, messages); kept current.
- `docs/conventions.md` - the body frame, leg numbering and command signs, asserted on all three platforms through `platform_shared/sign_convention.json`.

## Connection constraint (summary)

Two browser rules collide and decide the whole transport story: Web Bluetooth needs a **secure context** (https/localhost), while mixed-content blocking forbids an https page from opening `ws://` or loading `http://` images.
So an https origin gets BLE but no WebSocket or camera; the robot-hosted http origin gets WebSocket and camera but no BLE.
An in-place BLE→WebSocket upgrade only works from `http://localhost`.

Provisioning APIs (DPP, SmartConfig, Unified Provisioning, NAN) address only *how the robot joins WiFi* — none of them change the above.
See `docs/connectivity.md` for the full matrix, options and decisions.

## Conventions

- **C++ formatting**: `firmware/.clang-format` (Google base, 4-space indent, 120 col, left pointer alignment). `SortIncludes: false` — include order is intentional.
- **TS/Svelte formatting**: Prettier + ESLint (run `pnpm lint` / `pnpm format` from `app/`).
- Commit messages use a leading emoji (✨ feature, ⚡ perf, 🎨 format, etc.).
</content>
