# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

A 6-legged (18-servo) hexapod robot built on an ESP32-S3. The repo has three independent but related parts:

- **`firmware/`** — ESP32 firmware (C++ on ESP-IDF 5.4, built with PlatformIO via the pioarduino platform fork). Runs the gait engine, kinematics, sensor reading, and the WiFi/BLE/WebSocket communication stack. This is the source of truth for gait and mode.
- **`app/`** — SvelteKit web controller (TypeScript). Deployed to GitHub Pages, also embeddable into the firmware. Talks to the robot over BLE or WebSocket using protobuf messages generated from `platform_shared/*.proto`.
- **`simulation/`** — Python MuJoCo simulation + RL training (`train_mj.py`) for sim-to-real transfer to the robot; managed with `uv`. A trained policy is exported to a dependency-free C++ header (`export_policy.py`) and embedded in the firmware (`WALK_NN` mode).

The root `animations/` directory is the bundled animation library (JSON in the `platform_shared/animation.proto` schema) shared by all three platforms.

The NumPy kinematics/gait (`simulation/src/robot/firmware_gait.py`) is a faithful port of the firmware kinematics/gait (`firmware/include/kinematics.h`, `gait.h`) and is the sim-to-real deploy target — when changing motion math, keep both in sync.

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

**Two FreeRTOS tasks** (`firmware/src/main.cpp`):
- *Control task* (core 1, prio 5, 5 ms loop): `robot->readSensors() → planMotion() → updateActuators()`. See `Hexapod` (`firmware/include/hexapod.h`), which owns `MotionService`, `Peripherals`, and `ServoController`.
- *Service task* (prio 2, 100 ms loop): brings up WiFi, AP, mDNS, the `esp_http_server` webserver, BLE (NimBLE NUS), and the WebSocket adapter; then services WiFi/AP.

**EventBus** (`firmware/include/event_bus.h`) is the central decoupling mechanism — a typed, lock-protected pub/sub built on a static FreeRTOS queue + an `evtbus` worker task per message type. Each message type (`CommandMsg`, `ModeMsg`, `GaitMsg`, `ServoAnglesMsg`, etc. from `firmware/include/message_types.h`) gets its own `EventBus<Msg>` specialization, with its own queue and worker, exposing `publish`/`subscribe`/`peek`. Subscriptions can be rate-limited and batched (`EmitMode::Latest`/`Batch`). This is how communication adapters, `MotionService`, and the control loop talk without direct coupling.

**Motion pipeline** (`firmware/include/motion.h`): `MotionService` subscribes to `CommandMsg`/`ModeMsg`/`GaitMsg`/`ServoAnglesMsg`. `MOTION_STATE` (DEACTIVATED/IDLE/POSE/STAND/WALK/WALK_NN) selects behavior. Per tick it lerps `body_state` toward `target_body_state`, runs `GaitController::step`, then `Kinematics::inverseKinematics` to produce 18 servo angles. A 2 s command timeout (`COMMAND_TIMEOUT_MS`) zeroes motion if commands stop arriving.

**Communication adapters** (`firmware/include/communication/`, `comm_base.hpp`): `CommAdapterBase` is the shared base for the `Websocket`, `BLE` (NimBLE NUS) and `SerialAdapter` (native USB Serial/JTAG, `USE_SERIAL_LINK`) transports — all carry the same protobuf `socket_message_Message` frames. Its `ProtoDecoder` dispatches decoded messages to handlers registered per type (`registerHandlers` in `main.cpp`), which republish onto the EventBus; `emitAll` fans EventBus events back out to every adapter. Tag subscriptions are tracked per client so `emit` skips encoding when nobody is listening, and ping/pong is handled in the decoder. Request/response calls (`SystemInformationRequest`, `I2CScanDataRequest`, `FeaturesDataRequest`) are correlated by id, with a 15 s timeout on the app side.

**Serialization**: protobuf everywhere. `platform_shared/*.proto` is the single source of truth, compiled to nanopb C for the firmware (`firmware/scripts/compile_protos.py`, output in `firmware/src/platform_shared/`, gitignored) and to TypeScript for the app (`app/scripts/compile_protos.js` via ts-proto, output in `app/src/lib/platform_shared/`, gitignored). Both run automatically as build pre-steps. `MOTION_STATE`/`GaitType` cross the wire as enum positions, so firmware and app must be deployed together when those enums change.

**Build configuration**: `platformio.ini` defines one board env, `esp32-wroom-camera` (the ESP32-S3 WROOM cam), plus a host `native` env for unit tests. The Arduino-only classic-ESP32 envs were removed with the IDF port — recover them from the `main` branch if a classic ESP32 is needed again. Hardware feature flags (`USE_CAMERA`, `USE_MPU6050`, pin assignments) live in env `build_flags` and `firmware/features.ini`; factory defaults (app name/version, WiFi, `NUM_SERVO`) live in `firmware/factory_settings.ini`; IDF sdkconfig overrides (PSRAM, 1 kHz tick, BLE/WiFi memory placement) live in `sdkconfig.defaults`. The learned policy runs as a hand-rolled float MLP header from `simulation/export_policy.py`; no TensorFlow Lite or ESP-DL component is part of the build (a local `firmware/components/esp-dl/` clone is gitignored and unreferenced). Every `pio run` also merges `firmware.factory.bin` (bootloader + partitions + OTA data + app at offset 0) via `firmware/scripts/merge_factory.py`; the release workflow publishes it and the Pages workflow serves it through the ESP Web Tools page in `flasher/`.

**Peripherals** (`firmware/include/peripherals/`): `Peripherals` owns the drivers behind the `USE_*` flags — MPU6050 DMP (`drivers/mpu6050.h`) and HMC5883 magnetometer (`drivers/hmc5883.h`), both over the shared `I2CBus` (`esp_driver_i2c`). Roll/pitch/yaw plus the compass heading are published together as `IMUAnglesMsg` every 25 ms. The bus pins and frequency come from `PeripheralSettingsService` (persisted, applied at boot — re-opening a live bus would invalidate every device handle).

**ESP-NOW controller** (`firmware/src/communication/espnow_adapter.cpp`): receives broadcasts from the handheld controller and republishes them as `CommandMsg`/`ModeMsg`/`GaitMsg`, so it drives `MotionService` exactly like the app does. Input-only. The radio must sit on `ESPNOW_WIFI_CHANNEL` (a STA join forces the router's channel instead — `begin()` logs this). The wire format in `communication/controller_packet.h` is shared with the separate controller firmware and with `simulation/controller_bridge.py`; keep the three in sync.

**OTA** (`firmware/include/ota_service.h`): `POST /api/firmware` streams a raw `.bin` request body into the inactive OTA slot; `POST /api/firmware/download` takes `{"download_url"}` (https only) and runs `esp_https_ota` against it using the mbedTLS root-CA bundle. Both publish `OtaStatusData` progress, which the app's update page feeds into the existing telemetry store. Rollback is enabled, and the service task calls `confirmRunningImage()` only after networking is up and the control loop has run 200 ticks (1 s), so a firmware that crashes before then reverts to the previous slot on the next reset.

## Web app architecture

SvelteKit (Svelte 5) + Tailwind/daisyUI, static-adapter SPA. Routes under `app/src/routes/` map to controller, `animations` (library and editor), connection, bluetooth, per-peripheral settings (servo/imu/camera/i2c), `wifi/*` (sta/ap/mdns) and `system/*` (status/filesystem/metrics/update) pages. `app/src/lib/` holds the shared logic: `transport/` (BLE, WebSocket and Web Serial byte-pipe implementations of `ITransport`, plus `databroker.ts` for tag-based pub/sub over the one active transport (serial > websocket > ble), `request()` correlation RPC and ping/pong), `proto-api.ts` (`resolveUrl` and IP helpers for the HTTP endpoints), `kinematic.ts`/`gait.ts`/`motion.ts` (a TS mirror of the robot math for the 3D visualization), `sceneBuilder.ts` + `Visualization.svelte` (three.js URDF rendering), and `stores/` for app state. The controller sends `Command` messages and subscribes to telemetry by message tag.
`app/src/lib/animation/` is the TS mirror of the animation reference (`simulation/src/robot/animation.py`), tested against the shared fixtures in `animations/fixtures/`, and backs the `/animations` route's library and editor.

## Further documentation

- `docs/connectivity.md` — how the app reaches the robot: the browser origin constraints that gate every transport, provisioning options, the `NET_STATUS`/`NET_COMMAND` topics, and the ranked plan. **Read before proposing any connection flow.**
- `docs/animation.md` — the animation system as implemented (file format, evaluator rules, firmware mode, messages); kept current.

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
