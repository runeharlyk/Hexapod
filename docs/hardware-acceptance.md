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

## Open checks

Checks that wait for the robot, with the procedure that settles each.

- The task watchdog restarts a board whose control task hangs, with the watchdog's message in the core dump (roadmap 10.14): the `esp32-wroom-camera-hang` env, which sets `DEBUG_HANG_CONTROL_S`, as "Checking it on the robot" in `docs/firmware.md` describes and the Session checklist below runs.
  Note the time from the hang to limp servos, which `docs/firmware.md` only estimates.
- The status page reads the core dump that hang leaves (roadmap 2.5): after the restart, the Last Crash card names the task `control`, the watchdog's message and a hash that matches the running build.
  The Download button over Bluetooth and `curl.exe -o coredump-wifi.bin http://<ip>/api/coredump` over WiFi give the same bytes, which `esp-coredump info_corefile -t raw` decodes against `.pio/build/esp32-wroom-camera-hang/firmware.elf` (section 4 of the Session checklist has the commands); Erase is refused while the robot stands and, once it is deactivated, leaves "No crash stored".

## Session checklist

One session at the robot that closes the open checks above and those the roadmap leaves for the robot (1.1, 1.2, 1.3, 1.4, 1.6, 1.7, 2.4, 2.5, 3.2, 5.3, 5.4, 9.5, 10.12, 10.14 with 9.11, 11.1, the goal moves, and 6.11's record), and takes the measurements the policy retrain needs.
The order is cheapest and safest first: nothing walks on the floor before the safety checks have passed.
Every command runs in PowerShell, the firmware's and the core dump's from the repository root, the Python tools from `simulation/` with `uv run python`.
PlatformIO is the hexapod-pio-env's 6.1.19, `& C:\Users\Rune\.cache\hexapod-pio-env\Scripts\python.exe -m platformio`, written out in full in each command: the `pio` on PowerShell's PATH is 6.1.13, which the pioarduino platform rejects, and `~\.platformio\penv`'s is 6.2.0, which fails the ESP32 build with `No module named 'SCons.Tool.FortranCommon'` (checked on 2026-10-20).
The core dump is decoded with the penv's `esp-coredump` 1.17.2 and PlatformIO's Xtensa gdb, which is not on the PATH, so the command names both.
Windows opens a COM port for one program at a time: close the serial monitor with Ctrl-C before any upload or core-dump read on its port, or they fail with "Access is denied".
`<uart>` is the board's CH343 UART port (flashing and the console; COM3 on 2026-10-15), `<usb>` its native USB port (the protobuf link), and `<ip>` its address on the network.
One driver during the measurements: before sections 6 and 8 close every tab of the app, on every phone and PC, and switch the handheld off.
An open app re-sends its last command every 200 ms while the robot stands or walks, centred sticks when nobody touches it, and the firmware takes the last command from any client, so it walks a velocity run at about half its stick; `bench_floor.py` marks such a run INVALID and reports no speed or rate for it.
Write every outcome, failures included, into a new section of this file headed with the date and the firmware's ELF hash, in the format of the tables above, unless an item names another place.

### Session sheet

The commands in order; the numbered sections below say what each must show.

1. `& C:\Users\Rune\.cache\hexapod-pio-env\Scripts\python.exe -m platformio run -e esp32-wroom-camera -t upload --upload-port <uart>`, then `& C:\Users\Rune\.cache\hexapod-pio-env\Scripts\python.exe -m platformio device monitor -e esp32-wroom-camera -p <uart>`, left open until step 5.
2. App, System, Metrics: Refresh the task table twice, 10 s apart, while `uv run python record.py --robot <ip> --seconds 30 --out snapshot.hexrec` runs; then `uv run python report_recording.py snapshot.hexrec`.
3. App, robot on its stand: IDLE, the eased exit, the handheld, the dead-man, the stop; then the stick directions on the floor.
4. App: reconnects over WiFi, USB and BLE; the BLE release; the mDNS scan while driving, with `uv run python record.py --robot <ip> --seconds 20 --out mdns.hexrec`; 20 I2C scans; the SDA short; OTA over BLE and USB.
5. Close the monitor (Ctrl-C), then `& C:\Users\Rune\.cache\hexapod-pio-env\Scripts\python.exe -m platformio run -e esp32-wroom-camera-hang -t upload --upload-port <uart>`, then `& C:\Users\Rune\.cache\hexapod-pio-env\Scripts\python.exe -m platformio device monitor -e esp32-wroom-camera-hang -p <uart>`: the hang, the status page, `curl.exe -o coredump-wifi.bin http://<ip>/api/coredump`, `Get-FileHash -Algorithm SHA256 coredump-ble.bin, coredump-wifi.bin`.
6. Close the monitor (Ctrl-C), then `& C:\Users\Rune\.platformio\penv\Scripts\esp-coredump.exe --chip esp32s3 --port <uart> info_corefile --gdb C:\Users\Rune\.platformio\packages\tool-xtensa-esp-elf-gdb\bin\xtensa-esp32s3-elf-gdb.exe .pio\build\esp32-wroom-camera-hang\firmware.elf`.
7. `& C:\Users\Rune\.cache\hexapod-pio-env\Scripts\python.exe -m platformio run -e esp32-wroom-camera -t upload --upload-port <uart>`: `main` back; never `uploadfs`.
8. Scope on the 6 V and 5 V rails: rest, stand-up, stalled femur, full-stick walk; fit the 8 capacitors; the same four again.
9. Robot stood on the floor with the app, then every app tab closed and the handheld off, as for steps 10 and 12: `uv run python measure_pwm_per_degree.py <ip>`.
10. Robot on its stand: `uv run python servo_envelope.py --host <ip> --out envelope-<date> --scale <counts per degree> move travel`.
11. `uv run python robot_animate.py --port <usb> list`, then the animation acceptance of `docs/animation.md`, whose app steps open the app; close it again before step 12.
12. Camera, if used, in a second PowerShell window in `simulation/`, running through the session: `uv run python ground_truth.py course.json --camera 0 --out track.csv`; then in the first `uv run python bench_floor.py run --host <ip> --out floor-<date>`.
13. `uv run python bench_floor.py report floor-<date> --track track.csv`, and the tables into their documents; then the Contention run of section 3 with the app open.

### 1. Flash and boot

- **Flash:** `& C:\Users\Rune\.cache\hexapod-pio-env\Scripts\python.exe -m platformio run -e esp32-wroom-camera -t upload --upload-port <uart>` with the robot on its stand and the servos powered, then `& C:\Users\Rune\.cache\hexapod-pio-env\Scripts\python.exe -m platformio device monitor -e esp32-wroom-camera -p <uart>`.
  Expected: the boot ends without a panic, watchdog or stack-canary line, and the robot comes up DEACTIVATED with every servo limp.
  Record the ELF hash the status page shows as the session's firmware.
- **Core-dump line:** The boot log's `Core dump: <n> bytes, readable|unreadable, checked in <t> us`.
  Expected: with no dump stored, 0 bytes; the line is read again after the hang (section 4), where `docs/firmware.md` estimates 20 to 60 ms for a full 64 KB dump.
  Record: both lines here, and the measured time in place of that estimate in `docs/firmware.md` (Decoding a core dump).
- **Module and flash (roadmap 1.4):** Read the label off the module; the status page shows `flash_chip_size`.
  Expected: 16 MB, matching `default_16MB.csv`.
  Record: the Board table above.
- **Task table (roadmap 10.12):** App, System, Metrics: press Refresh, wait 10 s (the robot snapshots at most once per 10 s) and press it again, while `uv run python record.py --robot <ip> --seconds 30 --out snapshot.hexrec` runs.
  Expected: the line under the table reads `snapshot <x> ms, <n> B of free stack scanned`, against the estimate of 2 to 4 ms over 70 to 90 KB; the second table's CPU shares add up to 200 %; `uv run python report_recording.py snapshot.hexrec` shows the period's p99 and overruns, a refresh costing at most one tick late by about the snapshot.
  Record: `snapshot_us`, the scanned bytes, the task count and the tightest stack, here and in place of the Open line of `docs/firmware.md` (Task table).

### 2. Safety

The robot on its stand, legs free, and the app connected over WiFi.

- **IDLE from DEACTIVATED (roadmap 6.11):** Press IDLE.
  Expected: nothing moves; every servo stays limp until a joint of the 3D model is dragged, and then only that servo moves.
- **The eased exit from IDLE:** In IDLE drag one femur well away from its stance, then press STAND.
  Expected: every joint eases to the stance, at most 1.875 deg a tick (`docs/firmware.md`, Motion pipeline), with no snap, and the mode shows STAND once the ease has ended.
  The owner confirmed both on 2026-10-20 (roadmap 6.11) without a record here; record them on this build.
- **The handheld's priority, release and badge (roadmap 11.1):** In STAND move the app's stick, then the handheld's.
  Expected: while the handheld moves, the status bar shows "Handheld driving", the app's sticks do nothing and a goal is refused (423); 600 ms after the handheld's sticks centre the badge goes, the robot stays stopped, and the app drives again once its stick has been centred (`docs/firmware.md`, Handheld priority).
- **The dead-man (roadmap 1.3):** Walk with the handheld's stick held, then switch the handheld off.
  Expected: the legs stop about 600 ms after its last frame.
  The same over BLE from the phone, switching its Bluetooth off mid-walk.
- **The stop (roadmap 9.5):** Walk from the app and press DEACTIVATED.
  Expected: the legs go limp at once, with no further step.
- **Stick directions from behind (roadmap 1.3):** The first item on the floor, behind the robot, at a small stick: left stick forward walks it away, left stick right strafes it to its right, right stick right turns it clockwise seen from above (`docs/conventions.md`).

### 3. Links

- **Reconnects with immediate state:** Over WiFi (reload the page), USB (pull and reseat the cable to `<usb>`) and BLE (disconnect and reconnect in the app), each after changing the mode with the handheld while the app was away.
  Expected: within a second of the link coming up the status bar shows the mode and the gait the robot is in, with nothing pressed (the snapshot answering the subscribe).
- **BLE stop on release:** Drive over BLE at full stick on the stand, then let go.
  Expected: the legs stop on the release, within a step and well before the 600 ms dead-man: the last frame of a released stick always goes out (`app/src/lib/transport/latest-frames.ts`).
- **Driving during an mDNS scan:** With `uv run python record.py --robot <ip> --seconds 20 --out mdns.hexrec` running, hold the app's stick to walk on the stand and press Scan on WiFi, mDNS.
  Expected: the scan answers after about 3 s and the gait never pauses; `uv run python report_recording.py mdns.hexrec` shows no dead-man stop and no overrun.
- **20 I2C scans in a row:** Peripherals, I2C: press Scan 20 times, each after the last has answered.
  Expected: each lists exactly 0x1E (HMC5883), 0x40 and 0x41 (PCA9685) and 0x68 (MPU6050), never a phantom address (the core-0 race of `docs/firmware.md`, I2C scan), and the I2C error counts on the metrics page do not rise.
- **SDA short (roadmap 2.4):** In STAND, short SDA (GPIO47) to ground with a jumper for about a second.
  Expected: the servos hold their pose, the metrics page counts timeouts and at least one bus recovery, the robot answers within a second of the release, and the console shows no reboot.
- **OTA over BLE and USB (roadmap 1.3):** System, Update: upload `.pio/build/esp32-wroom-camera/firmware.bin` over BLE, then over USB.
  Expected: each completes, the robot boots from the other slot and logs `running image marked valid`, and the status page shows the same ELF hash; time each (WebSocket took 152 s on 2026-10-15).
- **Contention (roadmap 9.5):** After the floor session of section 8, not during it: with the app on BLE and its metrics page open, one more goal run over WiFi, `uv run python bench_floor.py run --host <ip> --out floor-<date>-contention square`.
  A running goal ignores centred commands, so the app's heartbeat does not drive it, where it would halve a velocity run's speed.
  Expected: the outcome `done`, the square closing as in section 8, and the summary's overruns, dead-man stops and failed writes 0.

### 4. The task watchdog's hang (roadmaps 10.14, 9.11, 2.5)

The `esp32-wroom-camera-hang` env in `platformio.ini` builds the robot's firmware with `DEBUG_HANG_CONTROL_S=20` (`docs/firmware.md`, Task watchdog), so no file is edited; it is not a default env, and CI builds `esp32-wroom-camera` alone.

1. Close the monitor of section 1 (Ctrl-C), then `& C:\Users\Rune\.cache\hexapod-pio-env\Scripts\python.exe -m platformio run -e esp32-wroom-camera-hang -t upload --upload-port <uart>`, then `& C:\Users\Rune\.cache\hexapod-pio-env\Scripts\python.exe -m platformio device monitor -e esp32-wroom-camera-hang -p <uart>`.
   Expected: the boot logs `debug build: the control task hangs 20 s after boot`.
2. Within those 20 s press STAND, the robot on its stand, so the servos hold a pose when the task hangs; film the legs and the console together.
   Expected: the legs freeze at 20 s; about 5 s later the console shows `Task watchdog got triggered.` with `control (CPU 1)`, the core dump is written, and the board restarts.
   Record: the time from the watchdog's line to the legs going limp, from the video, against the estimate of `docs/firmware.md` (What the restart leaves).
3. After the restart and before the next hang, as the board restarts about every 25 s: the status page shows the reset reason `task watchdog`, and the Last crash card names the task `control`, the watchdog's message and a hash that matches the running build.
   Download the dump with the card's button over BLE, saved as `coredump-ble.bin` in the repository root, and over WiFi with `curl.exe -o coredump-wifi.bin http://<ip>/api/coredump`; `Get-FileHash -Algorithm SHA256 coredump-ble.bin, coredump-wifi.bin` gives both files the same hash.
4. Close the monitor (Ctrl-C), then `& C:\Users\Rune\.platformio\penv\Scripts\esp-coredump.exe --chip esp32s3 --port <uart> info_corefile --gdb C:\Users\Rune\.platformio\packages\tool-xtensa-esp-elf-gdb\bin\xtensa-esp32s3-elf-gdb.exe .pio\build\esp32-wroom-camera-hang\firmware.elf`, or from the downloaded file `& C:\Users\Rune\.platformio\penv\Scripts\esp-coredump.exe --chip esp32s3 info_corefile --gdb C:\Users\Rune\.platformio\packages\tool-xtensa-esp-elf-gdb\bin\xtensa-esp32s3-elf-gdb.exe -t raw -c coredump-wifi.bin .pio\build\esp32-wroom-camera-hang\firmware.elf`.
   Expected: `Panic reason:` repeats the watchdog's message, and the task list shows `control` suspended in `controlLoop`.
5. Flash `main` back: `& C:\Users\Rune\.cache\hexapod-pio-env\Scripts\python.exe -m platformio run -e esp32-wroom-camera -t upload --upload-port <uart>`.
   Never `uploadfs` here: its image would replace the LittleFS partition, and with it the stored settings and calibration.
   Expected: the boot's core-dump line shows the dump and its check time (section 1), the card says the dump is not the running build's, Erase is refused while the robot stands (409), and once it is deactivated Erase leaves "No crash stored".
   Record: here, and both Open checks above as done.

### 5. The servo rail and the bulk capacitors (roadmaps 5.3, 5.4)

Scope channel 1 on a servo connector's V+ (the 6 V rail), channel 2 on the 5 V logic rail, both referenced to the board's ground.
DC coupling for the level and the sag, AC coupling at 50 to 100 mV/div for the ripple; trigger on a falling edge 0.3 V under the resting level; 2 ms/div for a step, 50 ms/div for a walk.

1. Without the added capacitors capture: STAND at rest, a stand-up from DEACTIVATED, one femur stalled (in IDLE on the stand, hold leg 0's foot still and drag its femur 20 deg into the hand in the app's 3D model, for about a second), and a full-stick TRI walk on the floor.
   Expected: no reset; the status page's reset reason stays as it was (a brownout shows as `brownout`).
2. Power off, and fit the 8 capacitors: 470 uF 16 V low-ESR on each buck's output, 470 uF 25 V on each board's VBAT input.
3. The same four captures again.
   Record, per capture and rail, before and after: the resting level, the lowest sag and the peak-to-peak ripple, with the battery voltage at the pack, in `docs/electronics.md` (roadmap 5.3's power tree), and the fitted values in the KiCad schematic (roadmap 5.4).
   The 6 V rail under load is also a figure of `docs/servo-envelope.md` (section 6).

### 6. The servo envelope (roadmap 1.7)

- **Counts per degree:** Robot stood on a flat, level floor with the app, then every app tab closed and the handheld off: `uv run python measure_pwm_per_degree.py <ip>`.
  Expected: a fit whose residual is under 0.25 deg; the robot ends in STAND.
  It now runs on the robot: the real backend reads the roll from the DMP's pitch, the calibration from the servo settings, and writes raw PWMs.
- **Move time and travel:** Robot switched off on its stand, legs free, no app open and the handheld off, a slow-motion camera (240 fps or more) on femur 1 with marks at -30 and +30 deg: `uv run python servo_envelope.py --host <ip> --out envelope-<date> --scale <counts per degree> move travel`.
  It stands the robot and switches it to IDLE, swings the femur 60 deg five times with 1 s dwells for the video, then through a sweep of shorter dwells, asking after each whether the leg reached both marks; then it steps each joint of leg 0 from the servo's centre pulse both ways, 5 deg at a time, within the servo's calibrated pulse range (90 deg either side of its centre, read from the robot's servo calibration) and the firmware's joint limit, so the tibia's + side ends at 0 deg.
  At each step answer `f` if the joint moved freely, `s` on its end stop, `b` against the body or a leg, or `q` to end the joint; Enter stops the side, so a key pressed without looking never steps further.
  From the telemetry it measures each command's tick, that the board acknowledged it, the dwell the robot really held, and the IMU's response at 40 Hz.
  It cannot measure the move itself, since the servo reports no position: that comes from the video (the frames from the first motion to the stop, x 1000 / fps) and from the sweep's bounds; a stopwatch cannot time an 80 ms move, and a 30 fps video resolves it only to 33 ms.
  For the move under the robot's own weight, hang the leg's share of it from the foot (about a third of its mass in a tripod stance, with the femur horizontal) and pass `--load-g`.
  Record: `envelope-<date>/envelope.md` into `docs/servo-envelope.md`, with the rail of section 5.

### 7. Animation acceptance (roadmap 1.2)

On the native USB port: `uv run python robot_animate.py --port <usb> list`.
Upload any of the seven bundled animations it does not list with `uv run python robot_animate.py --port <usb> upload <name>`, not `uploadfs`.
Then the 11 bench steps and the 7 app steps of `docs/animation.md`, Acceptance on hardware, each recorded there.

### 8. The floor (roadmaps 1.6, 3.2 and the goal moves)

A clear floor of at least 2.5 x 2.5 m, a start mark, a tape measure, every app tab closed and the handheld off, and the overhead camera if used: `uv run python ground_truth.py course.json --camera 0 --out track.csv` in a second PowerShell window in `simulation/`, started before the first run and stopped after the last.
`uv run python bench_floor.py run --host <ip> --out floor-<date>`, or `--port <usb>`; a run over each closes 9.5's stand-then-goal over WiFi and serial.
It refuses a robot that is not DEACTIVATED; before each run it asks for the robot on its start mark and for the battery voltage at the pack, stands the robot, and refuses to go on unless it reports STAND, its body rolled less than 10 deg, with the handheld idle; after each run it lowers the robot, switches it off and asks for the figures measured on the floor.
A velocity run (`turn`, `tri`) whose recording shows any other command between the first and the last tick of the script's stick, as an open app's heartbeat does, is marked INVALID in the summary, with no speed or rate; run it again with the app closed.
Ctrl-C at any time cancels a goal, lowers the robot, switches it off and writes the summary of the runs so far.
The runs, in order:

- `rest`, 20 s standing still: the control period, and the IMU at rest against the domain randomisation's ranges.
- `turn`, TRI rx -0.5 through 360 deg in place: the DMP's yaw rate against the speed model's; give the heading change on the floor.
- `tri`, TRI forward at ly 0.3, 0.6 and 1.0 over the model's 2 m: give each distance between the marks.
- `metre`, three 1 m goal walks: give each distance; roadmap 3.2 asks the camera to agree with the tape within 2 cm.
- `square`, 1 x 1 m of goal walks and 90 deg turns, and `circle`, a 0.5 m radius arc through 360 deg: give the distance from the start mark and the heading error.

Expected: every outcome `done`; in the simulation (`simulation/README.md`, Goals) the square closes within 4 cm and 2.6 deg, and the speeds are the model's within the feet's slip.
Record: `floor-<date>/summary.md`, written again with `uv run python bench_floor.py report floor-<date> --track track.csv`, into `simulation/docs/sim-to-real.md` (roadmap 1.6); each table carries the firmware's hash and the battery voltage.
Each run's `.hexrec` is also an input for `identify.py` (roadmap 10.3), for example `uv run python identify.py floor-<date>/tri-ly100.hexrec`.

### 9. Last

- **The flasher page (roadmap 1.1):** Optional, as it replaces the session's firmware: install `v0.1.1` from the web flasher without erasing the device, check that it boots and reports the release's ELF hash, then flash `main` back as in section 4.
