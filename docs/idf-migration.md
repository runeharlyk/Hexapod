# ESP-IDF migration: plan and reference

The firmware is moving from Arduino (`framework = arduino`, Arduino-ESP32 2.0.14 / IDF 4.4.6) to ESP-IDF.
[`runeharlyk/SpotMicroESP32-Leika`](https://github.com/runeharlyk/SpotMicroESP32-Leika) has already made this move and is the reference — take from it rather than rediscovering.

Note: clone Spot fresh when exploring. The local working copy carries unrelated local diffs.

## What Spot did

Still PlatformIO, not bare idf.py: `framework = espidf`, `platform = espressif32 @ 6.8.1`.
That keeps the familiar env-per-board layout while getting IDF.

- Same directory shape — `esp32/include`, `esp32/src`, `esp32/lib` — which maps one-to-one onto this repo's `firmware/include`, `firmware/src`, `firmware/lib`.
- `sdkconfig.defaults` at the root, plus per-board partition tables under `esp32/partition_table/`.
- Almost no shim layer: `esp32/include/compat/` contains only `pgmspace.h`. It is a rewrite, not an Arduino emulation.
- `esp32/include/motion_states/` — motion split into per-state files rather than one large switch.
- An `esp32-p4` env pointing at the pioarduino platform fork, for newer silicon.

### Replacements to copy

| Arduino dependency here | Spot's IDF replacement |
| --- | --- |
| `hoeken/PsychicHttp` | own `communication/webserver.{h,cpp}` + `websocket.{h,cpp}` over `esp_http_server`, with `-D CONFIG_HTTPD_WS_SUPPORT=1` |
| `WiFi`, `ESPmDNS` | `wifi/wifi_idf.{h,cpp}` |
| `DNSServer` | own `wifi/dns_server.h` |
| MsgPack / JSON wire format | protobuf via nanopb — see below |
| `FastLED` | `led_strip` (RMT) |
| `NimBLE-Arduino` | **nothing — see the BLE gap** |

Still-Arduino here and needing attention: eleven Wire-based sensor/display libraries (SSD1306, GFX, BusIO, ST7735, HMC5883, BMP085, ADS1X15, Unified Sensor, BNO055, the PWM-servo-driver fork, I2Cdevlib-MPU6050) plus `NewPing` (`pulseIn`-based).
That is the bulk of the port.
ArduinoJson needs no change — it has no Arduino dependency — though the proto pipeline may retire it.

Already IDF-shaped: `firmware/components/esp-dl/` is in components layout and ESP-DL is IDF-native, so the `WALK_NN` path improves.
`firmware/lib/tfmicro` would move to `components/`.
`EMBED_WEBAPP` maps onto CMake `EMBED_FILES`.

## The protobuf pipeline — take this

The most valuable thing to lift. A single source of truth compiled for both sides:

- `platform_shared/*.proto` (`api.proto`, `filesystem.proto`, `message.proto`)
- firmware: `esp32/scripts/compile_protos.py` → nanopb (vendored as a submodule, `-I submodules/nanopb`)
- app: `app/scripts/compile_protos.js` → `ts-proto` + `@bufbuild/protobuf`, wired into `pnpm build` as a `pnpm proto` prestep
- `template/stateful_proto_endpoint.h` adapts the existing `StatefulService` pattern to protobuf endpoints

This is what makes "use a proper protocol" cheap, and it is the reason WiFi provisioning should go in *our* schema rather than adopting Espressif's protocomm (see `connectivity.md`).

## The BLE gap

**Spot has no BLE at all** — no NimBLE, no GATT. It was dropped in the migration.

This matters because BLE is on the critical path for both near-term goals:

- WiFi provisioning over BLE (ranked #1)
- WebRTC signalling over BLE (ranked #3)

So the port must re-add BLE on `esp_nimble` directly. There is no reference for it in Spot.
This cost was not priced into the original ranking.

## platformio.ini corrections for this repo

`default_envs = esp32-camera` is the classic ESP32 (`board = esp32cam`), but the actual robot is the **ESP32-S3 WROOM cam** — that is the `esp32-wroom-camera` env (`board = esp32-s3-devkitc-1`).
Default should point at the S3 env.

Note the flash difference: Spot pins its S3 env to 8 MB, this repo to 16 MB with `qio_opi` memory type.

CLAUDE.md's overview says "ESP32-S3", which matches the real robot but not the default build target.

## Open decision — start here next session

Two orderings, not yet chosen:

**A. Port first.** IDF port with Spot's structure and proto pipeline → re-add BLE on `esp_nimble` → provisioning message → WebRTC.
Nothing gets written twice, but the hotspot demo waits for the whole port.

**B. Demo first.** Build BLE WiFi provisioning on the current working Arduino build so the hotspot demo exists now, accepting that it is rewritten during the port.

B costs duplicated work on the provisioning message; A costs delay. The BLE gap makes A's total larger than first estimated, which slightly favours B if the demo has a date attached.

## Migration status

The platform migration is complete on the `idf-migration` branch. Scope excludes policy
runner, net status/command, and animation (future features, deliberately out).

**Done and building green (firmware `pio run` SUCCESS; app `pnpm build` SUCCESS):**

- Stages 1-8: IDF build scaffold, event bus / littlefs, nanopb protobuf pipeline, wifi/AP/mDNS + settings HTTP, protobuf websocket comm model, motion pipeline, BLE (NUS on esp-nimble-cpp), EMBED_WEBAPP.
- Real peripherals: dual PCA9685 servo control over `esp_driver_i2c`; MPU6050 DMP IMU (body fusion + telemetry).
- Dual transport: WebSocket + BLE both carry the same protobuf `Message` frames, sharing comm handlers and telemetry (`registerHandlers` / `emitAll` in `main.cpp`).
- App transport rewritten to protobuf: `ts-proto` codegen from `platform_shared/*.proto`, byte-pipe transports, tag-based pub/sub + ping/pong in the databroker. Controller (motion/servo/IMU) drives the firmware end-to-end.
- **Settings pages over proto-HTTP.** `proto-api.ts` encodes `api.Request` / decodes `api.Response`; AP/STA wifi, scan, and mDNS pages rewired with endpoint-path alignment and proto-shape adaptation (IPs are `uint32` little-endian in proto, strings in the UI).
- **Correlation RPC** (firmware dispatch in `main.cpp` + app databroker `request()`): `SystemInformationRequest`, `I2CScanDataRequest`, `FeaturesDataRequest` → matching responses, correlated by id with a 15 s timeout. System-status, system-metrics, and i2c pages wired to it.
- **System metrics**: `AnalyticsData` streamed on a 2 s tick and fed to the charts store (CPU-usage chart dropped — no per-core runtime stats in the migration scope).
- `/api/features` served as JSON (adds `sleep`/`analytics` true, `ota`/`*_firmware` false) so the app menu gates correctly.
- Config polish: AP SSID rebranded `Hexapod-#{unique_id}` with real MAC-suffix substitution, mDNS hostname `hexapod`.

Hardware-verified: full boot on the ESP32-S3 WROOM cam — BLE advertising, WiFi STA+AP+captive
portal, MPU6050, MotionService, and the 5 ms control loop all run, with flat heap (no leak).

Two hardware-only bugs surfaced on first flash and were fixed (see `sdkconfig.defaults`):
- BLE `esp_nimble_hci_init()` failed — the controller/HCI ran out of contiguous internal DMA RAM
  once WiFi took its buffers. `CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y` offloads WiFi/LWIP to PSRAM,
  restoring the ~19 KB the HCI needs (now ~31 KB free; a `Before BLE:` heap log watches the margin).
- The 5 ms control loop asserted in `vTaskDelayUntil`: ESP-IDF defaults to a 100 Hz tick, so
  `pdMS_TO_TICKS(5)` rounded to 0. `CONFIG_FREERTOS_HZ=1000` restores the Arduino-era 1 kHz tick.

Note: persisted AP SSID / mDNS hostname override the rebranded factory defaults until a factory
reset (`/api/system/reset`).

**Remaining — a feature-migration tail (new subsystems, not platform), deferred per scope:**

- **Camera service** (esp32-camera driver + streaming) and camera settings page.
- **Servo settings / calibration** persistence (`ServoTable` page still a TODO(proto) stub) + firmware servo-settings service.
- **Filesystem** ops + file transfer pages; **OTA / firmware update** page.
- `bluetooth` debug page still a TODO(proto) stub.

**Gotchas that persist:**

- `MOTION_STATE` / `MotionModes` are serialized by enum position — flash firmware and deploy app together.
- BLE NUS frames are unfragmented (≤ MTU ~247 B); large telemetry could truncate. Fine for control.
- `EMBED_WEBAPP=1` compiles a ~7 MB C array (the app blob) → very slow link; optimize later via CMake binary embedding. Default is `0` (app served separately).
- Generated code is gitignored and regenerated by build: firmware `firmware/src/platform_shared/*.pb.*`, app `app/src/lib/platform_shared/*.ts`, `firmware/include/WWWData.h`.
