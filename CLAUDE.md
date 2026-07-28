# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

A 6-legged (18-servo) hexapod robot built on an ESP32-S3. The repo has three independent but related parts:

- **`firmware/`** — ESP32 firmware (C++/Arduino, built with PlatformIO). Runs the gait engine, kinematics, sensor reading, and the WiFi/BLE/WebSocket communication stack. This is the source of truth for gait and mode.
- **`app/`** — SvelteKit web controller (TypeScript). Deployed to GitHub Pages, also embeddable into the firmware. Talks to the robot over BLE or WebSocket using a shared binary protocol.
- **`simulation/`** — Python MuJoCo simulation + RL training (`train_mj.py`) for sim-to-real transfer to the robot; managed with `uv`. A trained policy is exported to a dependency-free C++ header (`export_policy.py`) and embedded in the firmware (`WALK_NN` mode).

The NumPy kinematics/gait (`simulation/src/robot/firmware_gait.py`) is a faithful port of the firmware kinematics/gait (`firmware/include/kinematics.h`, `gait.h`) and is the sim-to-real deploy target — when changing motion math, keep both in sync.

## Commands

### Firmware (run from repo root)
```sh
pio run                         # build default env (esp32-camera)
pio run -e esp32-wroom-camera   # build a specific env (esp32-camera | esp32-wroom-camera | esp32dev)
pio run -t upload               # build + flash firmware
pio run -t uploadfs             # build + flash the LittleFS filesystem image (firmware/data -> /config etc.)
pio test                        # run native/embedded unit tests (test_embedded is ignored by default)
pio device monitor              # serial monitor @ 115200 with esp32 exception decoder
```
The build runs `firmware/scripts/build_app.py` as a pre-script: when `EMBED_WEBAPP=1`, it builds the Svelte app and bakes it into `firmware/include/WWWData.h`. By default `EMBED_WEBAPP=0` (see `firmware/features.ini`), so the app is served separately and the firmware build does not require Node.

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
pnpm test:unit -- <file># run a single test file, e.g. src/index.test.ts
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
```
See `simulation/README.md` for the full control-mode and training-flag reference.

## Firmware architecture

**Two FreeRTOS tasks** (`firmware/src/main.cpp`):
- *Control task* (core 1, prio 5, 5 ms loop): `robot->readSensors() → planMotion() → updateActuators()`. See `Hexapod` (`firmware/include/hexapod.h`), which owns `MotionService`, `Peripherals`, and `ServoController`.
- *Service task* (prio 2, 100 ms loop): brings up WiFi, AP, mDNS, the PsychicHttp server, BLE, and the WebSocket adapter; then services WiFi/AP.

**EventBus** (`firmware/include/event_bus.h`) is the central decoupling mechanism — a typed, lock-protected pub/sub built on a static FreeRTOS queue + a dedicated `evtbus` worker task. Each message type (`CommandMsg`, `ModeMsg`, `GaitMsg`, `ServoAnglesMsg`, etc. from `firmware/include/message_types.h`) gets its own `EventBus<Msg>` specialization with `publish`/`subscribe`/`peek`/`take`. Subscriptions can be rate-limited and batched (`EmitMode::Latest`/`Batch`). This is how communication adapters, `MotionService`, and the control loop talk without direct coupling.

**Motion pipeline** (`firmware/include/motion.h`): `MotionService` subscribes to `CommandMsg`/`ModeMsg`/`GaitMsg`/`ServoAnglesMsg`. `MOTION_STATE` (DEACTIVATED/IDLE/POSE/STAND/WALK) selects behavior. Per tick it lerps `body_state` toward `target_body_state`, runs `GaitController::step`, then `Kinematics::inverseKinematics` to produce 18 servo angles. A 2 s command timeout (`COMMAND_TIMEOUT_MS`) zeroes motion if commands stop arriving.

**Communication adapters** (`firmware/include/communication/`, `comm_base.h`): `CommAdapterBase` is the shared base for `BLE`, `Websocket`, and `EventSource` adapters. Incoming messages are decoded (MsgPack or JSON, selected by build flag) into a `[MsgKind, topic, payload]` array and republished onto the EventBus; EventBus events for subscribed topics are re-emitted back to clients. `_incomingMessage` (thread-local) prevents echo loops. Topic subscriptions are reference-counted across clients so a bus subscription exists only while ≥1 client wants the topic.

**Serialization**: controlled by `firmware/features.ini` — exactly one of `USE_JSON`/`USE_MSGPACK` must be 1 (default MsgPack). The wire protocol and topic enum are mirrored in `app/src/lib/interfaces/transport.interface.ts` (`MessageType`/`MessageTopic`) — keep these enums aligned with `message_types.h`.

**Build configuration**: `platformio.ini` defines three envs differing by board/pins/partitions. Hardware feature flags (`USE_CAMERA`, `USE_MAG`, `USE_MPU6050`, pin assignments) live in env `build_flags` and `firmware/features.ini`; factory defaults (app name/version, WiFi, `NUM_SERVO`) live in `firmware/factory_settings.ini`. Vendored TensorFlow Lite Micro is in `firmware/lib/tfmicro/` and ESP-DL in `firmware/components/esp-dl/` — these are third-party, don't edit.

## Web app architecture

SvelteKit (Svelte 5) + Tailwind/daisyUI, static-adapter SPA. Routes under `app/src/routes/` map to controller, connection, bluetooth, and per-peripheral settings (servo/imu/camera/i2c) pages. `app/src/lib/` holds the shared logic: `transport/` (BLE + WebSocket implementations of `ITransport`), `kinematic.ts`/`gait.ts`/`motion.ts` (a TS mirror of the robot math for the 3D visualization), `sceneBuilder.ts` + `Visualization.svelte` (three.js URDF rendering), and `stores/` for app state. The controller sends `CommandMsg`-style joystick input and subscribes to telemetry topics.

## Further documentation

- `docs/connectivity.md` — how the app reaches the robot: the browser origin constraints that gate every transport, provisioning options, the `NET_STATUS`/`NET_COMMAND` topics, and the ranked plan. **Read before proposing any connection flow.**
- `docs/idf-migration.md` — the move from Arduino to ESP-IDF, using SpotMicroESP32-Leika as reference. Includes the open ordering decision and current session state.
- `docs/animation.md` — the procedural animation system (`ANIMATE` mode). Currently shelved; records the app/firmware sync contract, sign conventions, and known drift.

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
