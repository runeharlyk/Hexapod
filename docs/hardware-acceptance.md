# Hardware acceptance

What has been run on the real robot, on which firmware, over which transport, and with what outcome.
A row is added whenever a feature is exercised on hardware; unit tests do not count here.

## Board

Checked on 2026-10-15 with `esptool.py flash_id` over the board's CH343 UART port.

| Property | Value |
|---|---|
| Chip | ESP32-S3 (QFN56) revision v0.2 |
| Flash | 16 MB, quad, 3.3 V (matches `default_16MB.csv`) |
| PSRAM | 8 MB embedded |
| Module label | not yet read off the module |

## 2026-10-15, branch `leika-port-candidate`

Firmware ELF hash `2f877aeb6ee3`, flashed over the CH343 UART port (COM3); the robot on the bench, servos not driven.
The robot joined the home network at 192.168.1.151; WebSocket checks were run from a PC on the same network.
The native USB port was not connected, so nothing over the USB serial link was tested.

| Feature | Transport | Outcome |
|---|---|---|
| Boot without panic, watchdog or stack-canary hit, settings kept across the flash | UART console | pass |
| Station joins at 20 MHz bandwidth | UART console | pass (`BW20` in the association log) |
| WebSocket upgrade and greeting pong | WebSocket | fail on the first image (404: the web app's `GET /*` was registered before `/api/ws`), pass after c09ed3d |
| WebSocket ping answered | WebSocket | pass |
| Uncalibrated IMU settings load without a backup on every boot | UART console | fail on the first image (a zero-byte file read as invalid), pass after 9543aa7 |
| Reported ELF hash equals the built image's | HTTP `/api/features` | pass |
| Origin policy: a foreign page refused, the Pages app and a LAN dev server accepted | WebSocket | pass |
| Telemetry recording and report (`record.py`, 6 s standing) | WebSocket | pass: period mean 5018 us, p99 5566 us, no overruns, none dropped on the robot; 26 of 226 batches skipped for a busy socket at -74 dBm; compute p99 5380 us, above the 5 ms period |
| Firmware update over the protobuf link, same image | WebSocket | pass: 1972400 bytes in 152 s, booted from `app1`, `running image marked valid (app1)` |
| Internal RAM after camera init | UART console | 5159 B internal, 1555 B DMA free; the base branch `feature/animation-app` leaves 4347 B and 743 B, so no regression |

Not yet run on hardware: OTA over BLE and USB, a deliberate rollback, the 600 ms steering dead-man, servos staying still at boot, WiFi network cycling, IMU calibration, USB setup, and the stick direction on the floor.
