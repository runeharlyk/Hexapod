# Firmware

The firmware runs the hexapod on an ESP32-S3 (WROOM module with camera and octal PSRAM).
It is C++20 on ESP-IDF 5.4, built with PlatformIO through the pioarduino platform; `platformio.ini` defines one board environment, `esp32-wroom-camera`, and a host `native` environment for unit tests.
The firmware is the source of truth for gait and mode: the app and the handheld controller only send commands and display what the robot reports.

This document describes the architecture as the code implements it.
Paths are relative to `firmware/` unless they start at the repository root.

## Components and their files

| Component | Files |
| --- | --- |
| Entry point, task creation, message handlers, transport bridges | `src/main.cpp` |
| Robot facade (owns motion, sensors, servos) | `include/hexapod.h` |
| EventBus and internal message types | `include/event_bus.h`, `include/message_types.h` |
| Motion state machine | `include/motion.h` (`MotionService`) |
| Gait generator and command ramp | `include/gait.h`, generated `include/gait_tuned.h` |
| Inverse kinematics | `include/kinematics.h` |
| Learned policy (WALK_NN) | `include/policy_runner.h`, generated `include/policy/hexapod_policy.h` |
| Animation player and mode arbitration | `include/animation/` (`animation.h`, `animation_codec.h`, `animation_runner.h`, `animation_store.h`, `mode_arbiter.h`) |
| Servo output (two PCA9685 boards) | `include/peripherals/servo_controller.h`, `include/peripherals/drivers/pca9685.h` |
| IMU and magnetometer | `include/peripherals/peripherals.h`, `include/peripherals/drivers/mpu6050.h`, `include/peripherals/drivers/hmc5883.h`, `include/peripherals/imu_calibration.h` |
| Shared I2C bus | `include/peripherals/i2c_bus.h` |
| I2C error counts and bus recovery | `include/peripherals/i2c_health.h` |
| Servo board write backoff and restart | `include/peripherals/servo_board.h` |
| Status LED | `include/peripherals/status_led.h`, `include/peripherals/status_led_pattern.h` |
| Transport base and protobuf dispatch | `include/communication/comm_base.hpp`, `include/communication/proto_helpers.h` |
| WebSocket transport | `include/communication/websocket.h`, `src/communication/websocket.cpp`, `include/communication/ws_frame.h`, `include/communication/stall_tracker.h` |
| BLE transport (Nordic UART Service) | `include/communication/ble.h`, `src/communication/ble.cpp` |
| USB serial transport | `include/communication/serial_adapter.h`, `src/communication/serial_adapter.cpp`, `include/communication/serial_frame.h` |
| ESP-NOW controller receiver | `include/communication/espnow_adapter.h`, `src/communication/espnow_adapter.cpp`, `include/communication/controller_packet.h` |
| HTTP server and origin check | `include/communication/webserver.h`, `src/communication/webserver.cpp`, `include/communication/origin_policy.h` |
| Embedded web app | `include/www_mount.hpp`, `src/www_mount.cpp` |
| Per-tick telemetry | `include/telemetry/telemetry.h`, `include/telemetry/spsc_ring.h` |
| Firmware update (OTA) | `include/ota_service.h`, `include/ota_session.h`, `include/ota_flash.h`, `include/ota_image.h` |
| Filesystem and file transfer | `include/filesystem.h`, `src/filesystem.cpp`, `include/filesystem_service.h`, `include/file_transfer.h` |
| Camera | `include/camera_service.h` |
| Persisted settings | `include/template/` (`stateful_service.h`, `stateful_persistence.h`), `include/settings/`, `include/*_settings_service.h`, `include/imu_calibration_service.h` |
| WiFi station, soft AP, mDNS, captive DNS | `include/wifi_service.h`, `src/wifi_service.cpp`, `include/ap_service.h`, `src/ap_service.cpp`, `include/mdns_service.h`, `src/mdns_service.cpp`, `include/wifi/` |
| CPU usage | `include/cpu_stats.h` |
| Feature flags | `include/feature_flags.h`, `src/features.cpp`, `features.ini` |

## Tasks

`app_main` in `src/main.cpp` initialises NVS and the LittleFS filesystem, verifies the baked-in policy against its golden vector, starts the persisted settings services and then creates two application tasks.

- The **control task** (`control`, core 1, priority 5) runs a fixed 5 ms loop: `readSensors()`, `planMotion()`, `updateActuators()` on the `Hexapod` object, then records a telemetry sample if anyone listens and publishes the servo angles every 20 ms.
  The period is held with `xTaskDelayUntil`; after an overrun the schedule restarts from the current tick instead of replaying the missed ones.
- The **service task** (`service`, priority 2) brings up WiFi, mDNS and the soft AP, then the HTTP server, the transports, ESP-NOW and the camera, in that order.
  It then loops every 100 ms: services WiFi and the AP, expires an abandoned OTA session, ships telemetry batches, samples CPU usage and emits analytics while subscribed, and confirms the running OTA image once the control loop has run 200 ticks.

Other tasks are created by the components that need them:

- one `evtbus` worker per EventBus message type (core 1, priority 6, 4 KB stack in PSRAM), started on the type's first publish or subscribe; a subscriber therefore must never touch flash, since the IDF asserts that no flash access runs on an external stack;
- `BLE_Process` (core 0, priority 6), which reassembles and decodes BLE frames;
- `serial_rx` (priority 4), which reads and decodes the USB serial link;
- `dns_server` for the soft AP's captive-portal resolver;
- `ota_download` while an HTTPS firmware download runs (the camera is released first to free its internal RAM); the restart after an update runs from a one-shot `esp_timer` created at boot, so it needs no memory when the update ends;
- the ESP-IDF tasks for WiFi, the event loop, NimBLE and `esp_http_server`.

The control loop owns `MotionService`, `Peripherals` and `ServoController` through `Hexapod` (`include/hexapod.h`).
Everything else reaches the motion through the EventBus or through the few `Hexapod` methods the message handlers call (animation play, stop and pose, IMU calibration, I2C scan).

## EventBus

`include/event_bus.h` is a typed publish/subscribe channel; each message type `Msg` is its own `EventBus<Msg>` specialisation with its own static FreeRTOS queue, subscriber table and worker task.
Template parameters set the queue depth, the subscriber limit and the batch size per type; the settings buses use a smaller configuration than the default.

- `publish(msg)` stores the message as the type's latest value and enqueues it without blocking; it returns false when the queue is full.
- `peek(out)` returns the latest published value, so a late subscriber can start from the current state.
- `subscribe(fn)` delivers every message on the worker task; `subscribe(ms, fn)` rate-limits a subscriber, batching (`EmitMode::Batch`) or keeping only the newest (`EmitMode::Latest`) between deliveries.
- `subscribe` returns an RAII `Handle` that unsubscribes when destroyed; an owner stores it in a member, and `consume(fn)` parks it in a function-local static for a subscription that lives as long as the program.
- `publish(msg, handle)` skips the subscriber the handle names, so a component does not hear its own echo.

The internal message structs (`CommandMsg`, `ModeMsg`, `GaitMsg`, `ServoAnglesMsg`, `ServoSignalMsg`, `ServoStateMsg`, `IMUAnglesMsg`, `PoseMsg`, `AnimationCommandMsg`) live in `include/message_types.h` and exist in RAM only.
Status messages that go to the app unchanged (`OtaStatusData`, `WifiStatus`, `APStatus`, `AnimationStatus`) travel on the bus as their generated protobuf structs.
Callbacks run on the type's worker task, so a subscriber that touches state shared with the control task has to synchronise; `MotionService` keeps that surface small by having the workers only write targets that the control task reads.

## Motion pipeline

`MotionService` (`include/motion.h`) subscribes to `CommandMsg`, `ModeMsg`, `GaitMsg` and `ServoAnglesMsg`, and `Hexapod::planMotion()` calls its `updateMotion()` once per control tick.
The mode is a `MOTION_STATE`: `DEACTIVATED`, `IDLE`, `POSE`, `STAND`, `WALK`, `WALK_NN` and `ANIMATE`.

- `DEACTIVATED` puts both PCA9685 boards to sleep; `IDLE` keeps them powered and produces no angles; `POSE` drives every servo to its centre pulse for calibration.
- `STAND` eases the body toward the commanded offset, height and tilt; the gait stays idle, except that a new stance width is reached by stepping the feet there rather than sliding them.
- `WALK` turns the stick into a locomotion command for `GaitController`; with `GaitType::TUNED` the stick is a velocity command mapped through the searched gait in `gait_tuned.h` instead.
- `WALK_NN` hands stride, lift, cadence and ride height to `PolicyRunner`, which runs the exported network every fourth tick on top of the analytic gait.
- `ANIMATE` plays a clip or a streamed puppeteer pose through `AnimationRunner`.

Each tick in a stepping mode follows the same order: the body state eases toward its target, the gait command ramps toward the commanded one (`approachGaitCommand`), `GaitController::step` advances the phase and writes the six foot targets (Bezier swing, sinusoidal stance), and `Kinematics::inverseKinematics` turns body pose and feet into 18 joint angles.
The IMU's roll and pitch are added to the commanded tilt (self-levelling), clamped to the stick's own tilt range.
`ServoController::setAngles` converts angles to PWM with the per-servo calibration, holds a servo on a non-finite command, clamps to joint travel and logs saturation per joint type.

Commands arrive asynchronously and only set targets.
If no command arrives within a timeout, `MotionService` applies a zero command; the timeout is shorter while a stick is steering, because the senders repeat their command while one is held.
Gait changes are deferred until the gait is not stepping, because swapping the phase schedule mid-stride drops a swinging foot.
`GaitType::AUTO` picks a gait from the commanded speed at the same safe moments.

Mode changes are decided by the pure function in `include/animation/mode_arbiter.h` and applied only on the `ModeMsg` worker, which makes that worker the one place the mode changes.
An animation play borrows `ANIMATE` from the current mode and the control task asks for the mode back when the clip ends; `DEACTIVATED`, `IDLE` and `POSE` always apply at once, which is the safety property.
`MotionService` republishes the mode it settled on as `ModeMsgKind::APPLIED`, so observers learn the outcome of a borrow or a hand-back.

The gait and kinematics math has two mirrors outside the firmware.
`simulation/src/robot/firmware_gait.py` is the sim-to-real port and must stay in step, enforced by `simulation/test_firmware_gait_parity.py`; the app's `app/src/lib/` copies drive only the 3D view.
`include/gait_tuned.h` is generated by `simulation/export_gait.py` and `include/policy/hexapod_policy.h` by `simulation/export_policy.py`; neither is edited by hand.

## Sensors

`Peripherals` (`include/peripherals/peripherals.h`) owns the MPU6050 (DMP orientation, gravity, gyro) and the HMC5883 magnetometer, each behind its `USE_*` flag, on the shared `I2CBus` (`include/peripherals/i2c_bus.h`, `esp_driver_i2c`).
The control task reads the IMU every 25 ms and publishes roll, pitch, yaw and compass heading together as one `IMUAnglesMsg`.
The bus takes its lock with a bounded wait, so a busy bus fails a transfer instead of stalling the 5 ms loop, and an I2C scan from the service side locks per address for the same reason.
Bus pins and frequency come from the persisted peripheral settings and are applied at boot only: re-opening a live bus would invalidate every device handle.
The two PCA9685 servo boards (addresses 0x40 and 0x41) sit on the same bus.
IMU calibration offsets are measured on request while the robot rests, persisted by `ImuCalibrationService`, and written to the chip at boot because it forgets them at power-off.

### I2C errors and recovery

The IDF 5.4 driver fails a NACK and a timeout alike, with `ESP_ERR_INVALID_STATE`, so `I2CBus` classifies each failed transfer itself (`include/peripherals/i2c_health.h`).
A failure that left SDA or SCL low, or that spent the transfer timeout (20 ms, less one tick), counts as a timeout; any other is a NACK.
The counts are kept per device address since boot, survive a restart of the bus, and reach the app in `AnalyticsData.i2c_errors` with `i2c_bus_recoveries`; the metrics page lists them.
A device cut off mid-read holds SDA low until it has clocked out its byte, and the driver's own bus clear does not free it on the ESP32-S3, because the peripheral reset that follows cuts the hardware's pulses short.
So after a failure with SDA low and SCL high the bus releases its pins, gives up to nine SCL pulses on them and a STOP, and starts again, at most once a second.
A held SCL is clock stretching or a device without power clamping the lines, which pulses cannot free.

Each PCA9685 is a `ServoBoard` with its own write backoff, so a board that stops answering neither stalls the control loop nor stops the other board's writes.
After three failed writes in a row a board is tried every 500 ms, and on every tick again once it answers.
A board that answers again without auto-increment in MODE1 lost power, so it is started again: asleep, prescale set, the current pose written into its channels, and woken only if the servos were active.
A board that did not answer at boot is started the same way when it first answers.

## Status LED

The board's WS2812 on `WS2812_PIN` (GPIO 48) shows the motion mode and whether a client is connected.

| Mode | Colour |
| --- | --- |
| `DEACTIVATED` | dim white |
| `IDLE` | blue |
| `POSE` | magenta |
| `STAND` | green |
| `WALK` | cyan |
| `WALK_NN` | yellow |
| `ANIMATE` | orange |

The colour is steady while a client is connected over WebSocket, BLE or the USB serial link, and blinks once a second, half on and half off, while none is.
A USB host counts as a client, as it does for stopping an animation when the last client goes; the ESP-NOW controller does not, since it only broadcasts.
The mode shown is the latest `ModeMsg` REQUEST or APPLIED, the mode the app is shown.
Red is kept out of the mode colours for the battery warning, which becomes one more input to the pattern in `include/peripherals/status_led_pattern.h` once a battery signal exists (roadmap 2.1).

The service task drives the LED every 100 ms and sends a colour only when it changes, through the IDF's RMT TX driver and its bytes encoder: one RMT channel, no DMA and no LED-strip component.
The driver's channel, queues and encoder take an estimated few hundred bytes of internal RAM (from the IDF 5.4 sources); `logHeap("status led")` logs the free internal RAM right after they are allocated.
The LED starts first in the service task: the WS2812 keeps its colour while powered, so a later start would leave the colour from before a reset showing through the bring-up.

## Transports

The app reaches the robot over three byte transports that carry the same protobuf frames, `socket_message_Message` from `platform_shared/message.proto` (repository root), compiled to nanopb C under `src/platform_shared/` by `scripts/pre_build.py` on every build.

- **WebSocket** at `/api/ws` on the HTTP server (`Websocket`).
  A socket that stays full for longer than a stall limit is treated as gone, since TCP alone notices a vanished peer only after minutes.
- **BLE**, the Nordic UART Service on NimBLE (`BLE`), with one connection; messages larger than the MTU travel as length-prefixed chunks and are reassembled on `BLE_Process`.
- **USB serial** on the native USB Serial/JTAG port (`SerialAdapter`, behind `USE_SERIAL_LINK`), framed as COBS with a CRC-16 so a reader can resynchronise after lost bytes (`include/communication/serial_frame.h`, test vectors in `platform_shared/serial_frame_vectors.json`).
  The log console stays on UART0, so log output and frames never interleave.

`CommAdapterBase` (`include/communication/comm_base.hpp`) is the shared base.
Its `ProtoDecoder` dispatches each decoded message to the handler registered for its type, answers pings and records per-client tag subscriptions.
`emit` encodes a message once and sends it to the clients subscribed to its tag, or to one client for a reply; with no subscriber it returns before encoding.

`registerHandlers` in `src/main.cpp` registers the same handlers on every adapter.
Commands, modes, gaits and raw servo signals are validated and republished onto the EventBus; animation requests call into `Hexapod`; a `CorrelationRequest` is answered with a `CorrelationResponse` carrying the same correlation id (system information, features, I2C scan, WiFi, AP, mDNS, servo and peripheral settings, IMU calibration, OTA chunks, file chunks, animation listing and validation).

Outgoing data takes one of four shapes, chosen by the kind of data:

- **Streams** (servo angles, IMU, analytics, per-tick telemetry) are emitted at their own rate while a client subscribes.
- **Observable state** (mode, gait, WiFi status, AP status, OTA progress, animation status) is published on the EventBus by its producer when it changes and pushed to every subscribed client; a client that joins late asks for the current value once with a request, then follows the pushes.
  State is never sent on a timer.
- **Settings** are read and written by request and response only, so a push can never overwrite a form being edited.
- **Commands** (sticks, mode, restart, sleep) are fire-and-forget.

Streams and observable state reach the transports through bridges in `setupComm()` (`addBridge`, `observeStatus<T>()`).
A bridge holds its EventBus subscription only while some client on some transport subscribes to its tag, so nothing is converted for nobody.
`emitAll` sends a message on every transport.
When the last client on every transport has gone, a playing animation stops, so nothing runs unattended.

Per-tick telemetry (`include/telemetry/telemetry.h`) is recorded by the control task into a lock-free single-producer ring, only while someone subscribes, and shipped in whole batches by the service task; a full ring drops samples and counts them rather than blocking the control loop.

The **ESP-NOW receiver** (`EspNowAdapter`) is input only: it turns the handheld controller's broadcasts into `CommandMsg`, `ModeMsg` and `GaitMsg` on the EventBus, so the controller drives `MotionService` exactly as the app does.
ESP-NOW hears only the radio's current channel, so the robot pins its radio to `ESPNOW_WIFI_CHANNEL` unless a station connection holds it elsewhere, and it broadcasts a beacon on whatever channel it is on so the controller can find it.
The packet format in `include/communication/controller_packet.h` is shared with the controller firmware and `simulation/controller_bridge.py`, so the three change together.

Which transport a browser may use depends on the page's origin, not on the firmware; `docs/connectivity.md` covers that.

## HTTP server

`WebServer` (`include/communication/webserver.h`) wraps `esp_http_server`.
It serves the WebSocket, `/api/features`, the file API (`/api/files*`, `include/filesystem_service.h`), firmware upload and download (`/api/firmware*`), the camera's MJPEG stream and settings (`/api/camera/*`), and, when built with `EMBED_WEBAPP=1`, the web app itself (`include/www_mount.hpp`).
The origin check in `include/communication/origin_policy.h` allows pages from the local network, the hosted app, and clients that send no `Origin`.
A WebSocket opened by any other page is closed before it carries a frame, and an HTTP answer to one carries no CORS header, so the browser withholds it and refuses its preflighted requests.

## Settings and persistence

Settings services derive from `StatefulService<T>` (`include/template/stateful_service.h`), which guards the state with a mutex and runs update and hook handlers on change.
`FSPersistencePB` (`include/template/stateful_persistence.h`) stores each one as binary protobuf under `/littlefs/config/` (paths in `include/filesystem.h`; the files keep their legacy `.json` names), writing beside the file and renaming over it so a reset mid-write keeps the old settings, and keeping an unloadable file as `.bak` before defaults replace it.
Servo calibration, peripheral settings and IMU calibration are also published on their own EventBus types, so the servo and sensor code picks them up with `peek` at start and by subscription afterwards.
Factory defaults (app name and version, WiFi, AP, `NUM_SERVO`) come from `factory_settings.ini`.

## Flash layout and firmware update

The 16 MB flash holds NVS, OTA data, two 6.25 MB app slots, a LittleFS data partition (labelled `spiffs`) and a core-dump partition (`partition_table/default_16MB.csv`).

An update is written into the slot the robot is not running from, and the boot partition changes only after the whole image has arrived and validated.
It arrives by one of three paths: a raw `.bin` body on `POST /api/firmware`, an HTTPS URL on `POST /api/firmware/download` (fetched by `esp_https_ota` against the mbedTLS root-CA bundle), or chunked `OtaStart`/`OtaChunk`/`OtaFinish` requests over any protobuf transport (`OtaSession`, `include/ota_session.h`).
One update runs at a time, the motion is deactivated while it runs, and progress is published as `OtaStatusData`.
Bootloader rollback is enabled: the service task marks a new image valid only after networking is up and the control loop has run 200 ticks, so an image that crashes before that boots back into the previous one.

## Decoding a core dump

A crash writes an ELF core dump to the `coredump` partition.
A core dump from release firmware decodes only against the ELF of the build that produced it.
Each GitHub release attaches that ELF as `esp32-wroom-camera.elf`, beside the `.bin` images.

Download the dump from the robot as a file, then run:

```sh
esp-coredump info_corefile -t raw -c <dump.bin> esp32-wroom-camera.elf
```

`esp-coredump` ships in the ESP-IDF Python environment; `uv tool install esp-coredump` also provides it.
The command prints the crashed task's registers, the call stack, the task list and the memory regions.

## RAM budget

Internal DMA-capable RAM, not total heap, is the limit on this board.
`esp_get_free_heap_size()` reports megabytes because the PSRAM dominates it, while WiFi, the soft AP, the BLE controller, the HTTP server, the camera's DMA buffer and every task stack need internal RAM.

The project's 2026-08-29 measurement put internal RAM at about 260 KB, of which about 225 KB was spoken for at boot.
The same measurement attributed roughly 64 KB to WiFi initialisation, 64 KB to the BLE controller before its trimming in `sdkconfig.defaults`, 25 KB to the HTTP server, 26 KB to the transports and WebSocket, 26 KB to the camera and 9 KB to ESP-NOW.
These are one measurement, not datasheet figures; `docs/ram-budget.md` (roadmap 1.5) will replace this section with a per-stage table.
On 2026-10-15 `logHeap("camera")` read 32.4 KB of internal RAM free, largest block 23.5 KB, on `v0.1.1`; before the bus workers moved to PSRAM it read 5.1 KB, and some workers or the camera failed to start (`docs/hardware-acceptance.md`).

`logHeap()` in `src/main.cpp` logs free internal RAM, free DMA RAM and the largest free block of each at the tight points of the service task's bring-up.
The bring-up order follows from the budget: the soft AP claims its buffers first, because its driver does not null-check its allocations and panics when starved, while BLE, the camera and ESP-NOW degrade cleanly and come later.
`sdkconfig.defaults` (repository root) moves what it can out of internal RAM: WiFi, LWIP, NimBLE host and TLS buffers into PSRAM, WiFi and BT controller code into flash, smaller WiFi buffer pools and a peripheral-only BLE controller, and software AES and SHA so the crypto drivers allocate no DMA buffers.
Large one-off buffers in the firmware (telemetry ring, animation clips) are allocated so they land in PSRAM.

An edit to `sdkconfig.defaults` does nothing once a build has run.
PlatformIO generates `sdkconfig.esp32-wroom-camera` (gitignored) from it on the first build and treats that file as the configuration afterwards, silently ignoring new defaults.
Delete the generated file before rebuilding, and check that the option arrived with `grep <OPTION> sdkconfig.esp32-wroom-camera`.

## Build configuration

- `platformio.ini` (repository root): the board environment, pins and hardware flags such as `USE_CAMERA`, the partition table, and the build scripts.
- `features.ini`: feature flags (`USE_SERIAL_LINK`, `USE_POLICY`, `USE_MPU6050`, `USE_MAG`, `USE_ESPNOW`, `ESPNOW_WIFI_CHANNEL`, `EMBED_WEBAPP`); `include/feature_flags.h` supplies the defaults and `FT_ENABLED`.
- `factory_settings.ini`: factory defaults for settings.
- `sdkconfig.defaults` (repository root): ESP-IDF options (1 kHz FreeRTOS tick for the 5 ms loop, PSRAM, BLE and WiFi memory placement, rollback, core dumps).

Every build runs `scripts/pre_build.py` (nanopb generation), `scripts/pack_animations.py` (root `animations/*.json` into `data/animations/*.pb` for the filesystem image) and `scripts/build_app.py` (bakes the web app into `include/WWWData.h` when `EMBED_WEBAPP=1`), and afterwards `scripts/merge_factory.py`, which writes `firmware.factory.bin` (bootloader, partition table, OTA data and app from offset 0) for flashing a blank board.
The release workflow publishes that image, and the GitHub Pages site serves it through the ESP Web Tools page in `flasher/` (repository root).

## Host tests

Header-only logic (gait, mode arbiter, animation, serial framing, OTA session, telemetry ring, origin policy and others) is unit-tested on the host with Unity in the `native` environment: one suite per directory in `test/test_<name>/`, fakes in `test/fakes/`.
Run them with `pio test -e native`; the suites need a C++20 host compiler.
