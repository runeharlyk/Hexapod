# Changelog

All notable changes to this project are documented in this file, following [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).
No version has been tagged yet; everything so far is unreleased.

## [Unreleased]

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

- Fixes over-the-air updates from a release, which picked the factory image for the app slot.
- Fixes a swing foot jumping to the middle of its stride at the end of about one swing in a hundred, from a float underflow in the Bezier curve.
- Fixes the servo board waking with the last pose across a reset before the firmware turned its channels off.
- Fixes the control loop replaying missed ticks back to back after an overrun.
- Fixes an interrupted upload or settings write destroying the previous file, and an unloadable settings file being overwritten instead of kept as a backup.
- Fixes the file API reading and writing the settings under `/config`, which exposed the stored WiFi and access point passwords and bypassed the settings checks.
- Fixes WebSocket pings going unanswered and close frames going unechoed, which made clients disable their keep-alive and wait out their close timeout.
- Fixes the I2C transfer timeout being given in ticks where the driver takes milliseconds.
- Fixes a stance foot on a diagonal stride jumping by the whole stance depth at lift-off, as its press was measured along x + y instead of along the stroke.
