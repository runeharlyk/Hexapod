# Changelog

All notable changes to this project are documented in this file, following [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Added

- Adds a configuration page that saves the servo calibration and peripheral settings to a file and loads them back, refusing a foreign or half-valid file.
- Adds recording a `.hexrec` telemetry file from the metrics page, in the format `simulation/record.py` writes.
- Adds the compass hard-iron fit: `simulation/calibrate_compass.py` fits the offset from a recording and stores it, the firmware applies it live, and the telemetry carries the raw compass reading.
- Adds the app as an installable offline web app (manifest and service worker), not registered in the embedded build.
- Adds `simulation/identify.py`, which fits the servo, latency and IMU terms to a recording, and `train_mj.py --dr-centre`, which re-centres the training randomization on them.
- Adds the floor benchmark tools: `ground_truth.py` (overhead camera and ArUco markers), `score_track.py` and `measure_pwm_per_degree.py`.
- Adds `simulation/leg_clearance.py`, which checks adjacent legs for collision over the command envelope.
- Adds a USB serial backend to the Python `Robot` API.
- Adds a sign-convention fixture, `platform_shared/sign_convention.json`, asserted by the firmware, simulation and app tests, and `docs/conventions.md`.
- Adds `docs/firmware.md` (architecture) and `docs/releasing.md`.
- Adds the release's ELF as a release asset, for decoding core dumps.
- Shows each saved robot's firmware version, build and servo calibration state on the start page; a robot that cannot be asked shows its last report, labelled with its age, and `/api/features` now reports `servos_calibrated`.

### Changed

- Eases the feet home over 0.5 s on a minimum-jerk path when a walk stops, and carries any remaining offset into a walk that resumes during the ease.
- Models the simulated servo as non-back-drivable: gear friction holds well above the stall torque, while driving stays at stall.
- Shapes both sticks with a round dead zone and an axis snap.
- Turns the newer-release indicator into a link to the update page.
- Splits the hosted app's bundle so each page loads its code when opened; the embedded build keeps one bundle.

### Fixed

- Fixes a walk entered from STAND moving a joint up to 15.6 degrees in one tick: legs found mid-swing at the start of the cycle now replay their swing from the floor.
- Fixes the servo settings update answering 200 to an update it rejected, and the servo page reporting such an update as saved.
- Fixes the animation editor: a stale autosave could overwrite a newer draft, and storage failures went unreported.

## [0.1.1] - 2026-10-15

Fixes found running 0.1.0 on the robot, and WiFi from the hosted app.

### Added

- Adds WiFi from the hosted https app in Chromium 147 and later: with a robot address saved and local network access granted, it opens the robot's WebSocket and camera.

### Fixed

- Fixes the robot crashing after every successful firmware upload over HTTP, so the app reported a failed update that had in fact landed.
- Fixes the robot's own download of a release, which could not start its task with the camera running, crashed before answering, and could not follow GitHub's redirect.
- Fixes the restart after an update, which could find no memory for its task, so the robot kept the old image and refused later uploads as busy.
- Fixes event buses whose worker task found no internal RAM at boot and then never delivered a message; the workers now run on PSRAM stacks, which leaves about 27 KB more internal RAM after the camera starts.

## [0.1.0] - 2026-10-15

The first tagged release: the animation system and the ports from the SpotMicro firmware.

### Added

- Adds firmware updates over Bluetooth and USB: the https app sends the image over the protobuf link when it cannot reach the robot's HTTP endpoint.
- Adds a check of the chosen firmware file before upload, which refuses a factory image or an image for another chip.
- Adds a report after an update's restart that tells the new firmware running from a rollback, from the ELF hash the robot now reports.
- Adds a setup page for configuring the robot's WiFi over USB, which the web flasher links to after an install.
- Adds reuse of a previously granted USB port, and automatic reconnection after the board resets.
- Adds IMU calibration from the IMU page: with the robot lying still and level it zeroes the gyro bias and levels the tilt, and keeps the result across restarts.
- Adds a compass dial to the IMU page.
- Adds a drawing of each leg's reference pose to the servo calibration page.
- Adds a per-tick telemetry stream, `simulation/record.py` to record it and `simulation/report_recording.py` to compare it with the simulation's assumptions.
- Adds a Python `Robot` API that drives the simulation and the real robot over its WebSocket.
- Adds `firmware/include/secrets.h` for the WiFi and access point credentials, created on the first build and ignored by git.
- Adds a CI job for the simulation, parity and framing tests.
- Adds stack canaries to the firmware.

### Changed

- Publishes the plain app image beside the factory image in each release, and updates over the air from it.
- Frames the USB serial link with COBS and a CRC, so a lost or cut frame no longer desynchronises the stream.
- Stops walking after 600 ms without input while a stick is deflected, instead of 2 s, and keeps the learned walk alive with the app's heartbeat.
- Sends a neutral command when the app's tab is hidden, rather than only releasing the keys.
- Holds the self-levelling attitude within the body's tilt range, in the firmware and the simulation.
- Joins WiFi at 20 MHz bandwidth, replies to a WiFi settings change before reconnecting, and cycles through the saved networks, trying the last good one first.
- Sends small WebSocket frames as one TCP segment with Nagle off, skips stream frames a client cannot take, and closes clients that stopped reading or vanished.
- Writes firmware updates sector by sector, so an update no longer blocks the web server for seconds.
- Confirms a robot during discovery by its greeting, probes subnets over HTTP first, and reduces a pasted URL to its host.
- Shows a connect prompt on settings pages while no robot is linked, instead of waiting out a request timeout.
- Answers CORS for allowed origins only, and refuses WebSockets from other pages.
- Reports the IMU as present only when it was detected at boot.
- Runs the I2C bus at 400 kHz at most, the rating of the MPU6050 and HMC5883L, also on robots that stored a faster clock.
- Serves the embedded app with `no-cache` and an ETag, so the browser loads the new app after a firmware update and an unchanged file costs an empty 304.

### Fixed

- Fixes every WebSocket upgrade being answered with 404 on firmware built without the embedded app, as the web app's catch-all route was registered before the socket's.
- Fixes over-the-air updates from a release, which picked the factory image for the app slot.
- Fixes a swing foot jumping to the middle of its stride at the end of about one swing in a hundred, from a float underflow in the Bezier curve.
- Fixes the servo board waking with the last pose across a reset before the firmware turned its channels off.
- Fixes the control loop replaying missed ticks back to back after an overrun.
- Fixes an interrupted upload or settings write destroying the previous file, and an unloadable settings file being overwritten instead of kept as a backup.
- Fixes the file API reading and writing the settings under `/config`, which exposed the stored WiFi and access point passwords and bypassed the settings checks.
- Fixes WebSocket pings going unanswered and close frames going unechoed, which made clients disable their keep-alive and wait out their close timeout.
- Fixes the I2C transfer timeout being given in ticks where the driver takes milliseconds.
- Fixes a stance foot on a diagonal stride jumping by the whole stance depth at lift-off, as its press was measured along x + y instead of along the stroke.

[Unreleased]: https://github.com/runeharlyk/Hexapod/compare/v0.1.1...HEAD
[0.1.1]: https://github.com/runeharlyk/Hexapod/compare/v0.1.0...v0.1.1
[0.1.0]: https://github.com/runeharlyk/Hexapod/releases/tag/v0.1.0
