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

Later the same day, same image, servo power on and the robot lying flat on the bench:

| Feature | Transport | Outcome |
|---|---|---|
| No servo moves when the board resets while deactivated | UART reset, observed | pass |
| IMU calibration on a robot lying still | WebSocket `imu_calibrate` | pass: status 200; gyro bias at rest from (-0.121, -0.022, -0.022) to (-0.002, -0.001, -0.001) rad/s |
| Calibration kept across a reboot | UART console, WebSocket | pass: `MPU6050 calibration applied` at boot, bias (-0.003, -0.001, -0.001) rad/s after it |
| Rollback from an image that crashes at boot | WebSocket update, UART console | pass: an image calling `abort()` first in `app_main` (built for the test, not committed) was accepted and selected, booted from `app0`, aborted, and the bootloader went back to `app1`; the robot then reported the previous ELF hash `2f877aeb6ee3` |
| The robot downloading the `v0.1.0` release itself (`POST /api/firmware/download`) | HTTP, HTTPS to GitHub | fail on `v0.1.0` and `main`: first no internal RAM for the download task's stack, then a crash in `sendOk` (2bb8f9a), then `HTTP_CLIENT: Out of buffer` on GitHub's 925-character redirect; pass after 2bb8f9a, 48ed3bf and 933062c: 1972400 bytes in about 40 s, booted from `app1`, reporting the release's ELF hash `357a0b75a0c5` |

Not yet run on hardware: OTA over BLE and USB, the 600 ms steering dead-man, a reset while the servos hold a pose, WiFi network cycling, USB setup, and the stick direction on the floor.

## 2026-10-15, branch `hardware-acceptance-2`

Firmware ELF hash `1e3cb6e87623`, flashed over the CH343 UART port; the robot lying flat on the bench, deactivated, at -73 to -93 dBm from the access point.

Two failures on the earlier images of this branch led here.
Once, after an HTTP upload restarted the robot, the boot stopped after `MotionService: Subscribing to event buses...`; 13 later restarts did not repeat it.
Once, after a flash, the camera found no internal RAM for its 16 KB DMA buffer, the station dropped with reason 4 (association expired), and the robot neither rejoined nor printed anything for about two minutes.
The log line added in e4ad372 then showed internal RAM running out at boot: each of the 14 event bus types started its own worker with a 4 KB internal stack, and the workers and the camera took turns failing.
Before e4ad372 a worker that failed to start still marked itself started, so its bus silently never delivered for the rest of the session.
Neither the hung boot nor the missing rejoin is proven to follow from this, and neither has been seen since.

| Feature | Transport | Outcome |
|---|---|---|
| Event bus workers start on every boot | UART console | pass after 856aef1 (PSRAM stacks): no `No memory for a bus worker` and the camera up in 15 boots; of the two boots before it, one lost bus workers and the other the camera |
| Internal RAM after camera init | UART console | 32.4 KB internal free, largest block 23.5 KB, 24.7 KB DMA, the same within 44 B over 15 boots; 5.1 KB, 3.5 KB and 1.5 KB before |
| Restart over the WebSocket, boot to an IP | WebSocket, UART console | pass: 10 of 10 |
| Restart after an HTTP upload | HTTP, UART console | fail before 32f18c3: the reply was 200 but the restart task found no memory, so the robot kept running the old image and answered later uploads with 409; pass after it: 8 of 8, and 4 of 4 on this image |
| Gait and mode requests echoed through the buses, IMU angles pushed | WebSocket | pass |
| Idle on WiFi, 15 min | HTTP every 10 s, UART console | pass: 77 of 77 polls over 13 min answered, nothing on the console; no disconnect happened, so the rejoin after one is still untested |
