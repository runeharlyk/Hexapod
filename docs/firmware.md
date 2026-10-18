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
| Classic gait speed envelope (stick to stroke and cadence, posture clamps) | `include/gait_speed.h`, generated `include/gait_envelope.h` |
| Inverse kinematics | `include/kinematics.h` |
| Odometry (stance-feet fit, gyro fusion) | `include/odometry.h` |
| Goal controller (distance, turn and arc moves on the odometry) | `include/goal_controller.h`, the goal parts of `include/motion.h` |
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
| ESP-NOW controller receiver | `include/communication/espnow_adapter.h`, `src/communication/espnow_adapter.cpp`, `include/communication/controller_packet.h`, `include/communication/handheld_input.h` |
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
- `camstream` (core 0, priority 5, 4 KB stack in PSRAM), which takes each camera frame once and sends it to every MJPEG viewer, so a stream never holds the HTTP server's task and its WebSocket; a fourth viewer is refused with 503;
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
Status messages that go to the app unchanged (`OtaStatusData`, `WifiStatus`, `APStatus`, `AnimationStatus`, `GoalStatus`) travel on the bus as their generated protobuf structs.
Callbacks run on the type's worker task, so a subscriber that touches state shared with the control task has to synchronise.
`MotionService`'s subscribers write no motion state: they post into its inbox (`include/motion_inbox.h`), which the control task takes at the start of its next tick.

## Motion pipeline

`MotionService` (`include/motion.h`) subscribes to `CommandMsg`, `ModeMsg` and `GaitMsg`, and `Hexapod::planMotion()` calls its `updateMotion()` once per control tick.
Only the control task changes the motion.
The subscribers and the goal and odometry requests post into `MotionInbox` under its one mutex, and `updateMotion()` starts by taking everything posted since the last tick, so nothing changes in the middle of one.
It applies the modes in the order they arrived, then the newest gait, then the newest command, then the goal's mail.
Commands and gaits coalesce, since each is a whole snapshot; modes queue, eight deep and dropping the oldest when full, since a borrow and a hand-back are decided against the mode they find.
What a goal depends on is decided on arrival, under the mutex: its takeover by a stick off centre, the end a mode or a gait implies, and the refusal of an odometry reset while a goal drives.
So is the handheld's priority over the other sources (below).
The end a mode implies is the arbiter's decision from the mode applied last.
No task waits for the control task: the mode, the goal status and the pose the replies carry are the ones the control task noted last.
A goal's 409 and a play judge the mode applied, the one the robot reports, not a mode still queued.
A stop, a puppet pose and the loss of the last client judge it too, but also take a play's borrow still queued as ANIMATE, so they reach a play that has not been applied yet.
A goal accepted while a `DEACTIVATED` is queued is answered 202, since the mode applied still allows it, and is refused at the take, which applies the mode first: its status is `REFUSED`.
A script that asks for a stand waits for the robot to report it before it sends a goal (`RealBackend.set_mode`), and that goal is accepted.
The log lines of what a tick took come after its servo write (`Hexapod::updateActuators`), so a slow console does not delay the write.
The mode is a `MOTION_STATE`: `DEACTIVATED`, `IDLE`, `POSE`, `STAND`, `WALK`, `WALK_NN` and `ANIMATE`.

- `DEACTIVATED` puts both PCA9685 boards to sleep; `IDLE` keeps them powered and produces no angles; `POSE` drives every servo to its centre pulse for calibration.
- `STAND` eases the body toward the commanded offset, height and tilt; the gait stays idle, except that a new stance width is reached by stepping the feet there rather than sliding them.
  A posture change is clamped to what the legs can stand in (`clampStandPosture`) and taken only once the feet are at rest.
- `WALK` turns the stick into a locomotion command for `GaitController`; with `GaitType::TUNED` the stick is a velocity command mapped through the searched gait in `gait_tuned.h` instead.
  A classical gait (`TRI_GATE`, `BI_GATE`, `WAVE`, `RIPPLE`) takes its stroke and cadence from `classicWalkTarget` in `include/gait_speed.h`.
  The `stride` slider picks the stroke per axis: 0 is each gait's speed-optimal stroke at the posture, +1 the longest feasible stroke and -1 0.3 of the optimum, linear in between.
  A positive stride changes nothing where the speed-optimal stroke is already the longest: BI at the default posture and in most yaw postures, TRI in most yaw postures; at the default posture TRI, WAVE and RIPPLE have room (forward 65 to 82 mm).
  The stick's share (L1-normalised, times `speedFraction(s)`: -1, 0 and +1 give 0, 60 % and 100 %) sets the cadence as a fraction of that stroke's servo-bound top cadence, passed as an explicit `phase_rate`, with the stroke held, so slow walking takes long, slow steps.
  A mixed command runs at the smallest top cadence of the axes it uses.
  The generated `gait_envelope.h` holds the walkable postures (`WALKABLE`), the gait-independent geometry (`STROKE_MAX` and swing and stance kappa samples per axis and posture) and the constants the runtime needs (`STAND_FRAC` per gait, `SERVO_RAD_S`, `CADENCE_MAX`, `TRAVEL_MARGIN_DEG`, `DEFAULT_SPEED_FRACTION`); the firmware computes the top cadences at runtime, reading kappa as the running maximum of the samples.
  `MotionService` slews a stride change at 1.0 per second while walking and takes it at once at rest; a stride change is not a posture change and does not pause the walk.
  The posture sliders (`s1` lift, `h` body height, `fd` feet distance) are first clamped along a ray from neutral to the walkable region (`clampPosture`).
  A posture change while walking pauses the walk and settles the posture in stages: the lift comes down, the stance and body height move at the lower height, and the lift goes back up, after which the walk resumes.
  A walk entered from a stand at rest in the same posture starts at once; a rising lift needs no settling, a falling one does.
  A classic walk ignores the tilt stick and eases the stand's shift and tilt out, since the envelope was measured on a level, centred body.
  Self-levelling stays on, and the tilt it adds on a slope is not covered by the envelope.
  A stop eases the stroke out from rest and slows the cadence with it (`GaitCommandRamp` in `include/gait.h`), so it passes only through walks a slower stick would take.
  The dead-man's neutral posture is taken like any other, once the walk has stopped; in `STAND` it waits until the feet are still.
- `WALK_NN` hands stride, lift, cadence and ride height to `PolicyRunner`, which runs the exported network every fourth tick on top of the analytic gait.
  Its observation and action decoding is `PolicyCore` (`include/policy/policy_core.h`), which the training environment runs through `host/policy_api.cpp`, so the network trains on the code that deploys it.
  Its `h` and `fd` are clamped to the classic gaits' walkable region (`clampPosture`).
- `ANIMATE` plays a clip or a streamed puppeteer pose through `AnimationRunner`.

Each tick in a stepping mode follows the same order: the body state eases toward its target, the gait command ramps toward the commanded one (`approachGaitCommand`), `GaitController::step` advances the phase and writes the six foot targets (Bezier swing, sinusoidal stance), and `Kinematics::inverseKinematics` turns body pose and feet into 18 joint angles.
The IMU's roll and pitch, turned into the body frame (`include/imu_frame.h`), are added to the commanded tilt (self-levelling), clamped to the stick's own tilt range.
`ServoController::setAngles` converts angles to PWM with the per-servo calibration, holds a servo on a non-finite command, clamps to joint travel and logs saturation per joint type.

Commands arrive asynchronously and only set targets.
If no command arrives within a timeout, `MotionService` applies a zero command; the timeout is shorter while a stick is steering, because the senders repeat their command while one is held.
A command the inbox drops is no command to this dead-man: its window runs from the arrival of the last command taken.
Gait changes are deferred until the gait is not commanded, not repositioning and not settling a posture, because swapping the phase schedule mid-stride drops a swinging foot.
`GaitType::AUTO` picks a gait from the commanded speed at the same safe moments.
That speed is the stick's share of BI's top speed at the posture and stride, whichever gait walks it, and AUTO picks the slowest of `RIPPLE`, `TRI_GATE` and `BI_GATE` whose top speed covers it.
A stick pushed past the top of the gait in effect therefore walks at that top until the next stop.

Mode changes are decided by the pure function in `include/animation/mode_arbiter.h` and applied only by the control task, in the order the modes arrived, which makes it the one place the mode changes.
An animation play borrows `ANIMATE` from the current mode, and the control task asks for the mode back when the clip ends by posting the hand-back into its own inbox, behind every mode that arrived before it.
`DEACTIVATED`, `IDLE` and `POSE` always apply at the next tick, even mid-play, which is the safety property.
Waking or sleeping the two PCA9685 boards waits 5 ms per board (`include/peripherals/drivers/pca9685.h`), so the tick that enters or leaves `DEACTIVATED` overruns its 5 ms period by about 10 ms, two ticks, on which the robot is not being driven.
`deactivateServos()` in `main.cpp` publishes `DEACTIVATED` and waits 50 ms before a restart, a reset or a deep sleep.
The control task takes the mode within the rest of its period and the next tick's sensor read, and writes the second board's sleep bit about 5 ms after the first, so both are asleep at most about 12 ms after the publish; the 50 ms leaves room for one I2C transfer timing out on the way (20 ms), not two.
`MotionService` publishes every mode it applies, and the mode a refused borrow keeps, as `ModeMsgKind::APPLIED`, after it is in force and noted for the replies.
The robot reports a mode only then: the app bridge and the ESP-NOW adapter's mode mirror take `APPLIED` alone, so a client never sees a mode the robot is not in, and one that waits for the report finds it in force.
The status LED shows the latest mode message only if it is `APPLIED`, so it can lag during a graceful leave.

The gait and kinematics math has two mirrors outside the firmware.
`simulation/src/robot/firmware_gait.py` is the port the simulation's research modes run on and must stay in step, enforced by `simulation/test_firmware_gait_parity.py`; the app draws the robot with its own forward kinematics, which the firmware has none of.
`include/gait_tuned.h` is generated by `simulation/export_gait.py`, `include/gait_envelope.h` by `simulation/export_envelope.py` and `include/policy/hexapod_policy.h` by `simulation/export_policy.py`; none is edited by hand.

### Handheld priority

The ESP-NOW adapter tags each command with its origin (`CommandOrigin`, `include/message_types.h`): `HANDHELD_MOVING` when any axis of the frame is off centre, the `STAND` height trim included, and `HANDHELD` otherwise (`include/communication/handheld_input.h`); every other command is a `CLIENT`'s.
The handheld drives for `MotionInbox::HANDHELD_PRIORITY_MS` (600 ms, equal to `STEERING_TIMEOUT_MS`) after its last moving frame.
Meanwhile the inbox drops every client's command, centred or not, so the app's 200 ms heartbeat and the facade's keep-alive no longer stop the handheld's walk for a tick and move its posture; the handheld's own centred frame is its stop and is taken.
A handheld whose link drops mid-walk is therefore stopped by the dead-man 600 ms after its last frame, with `link_lost` set, though the app's heartbeat goes on arriving.
A goal asked for meanwhile is refused with 423, and a running goal ends `CANCELLED` on any moving frame of the handheld, its tilt stick and its height trim included.
After the window the robot stays stopped: a client's stick off centre is dropped until a client has sent a centred command, so neither a script's keep-alive nor a stick the app held through the drive walks the robot again by itself.
Commands carry no client's identity, so one centred command from any client ends that wait for all of them; the app's heartbeat, centred while its sticks are untouched, ends it within 200 ms.
Modes, gaits, animation plays, poses, servo writes and settings from any source are not gated, so the stops always pass: `DEACTIVATED`, `goal_cancel` and `AnimationStop`.
Whether the handheld drives is reported with the mode, as `ModeData.handheld`: the control task reports the mode applied again, as `APPLIED`, on each change of the flag, so the mode's other observers see an unchanged mode, and `mode_get` (request 134, reply 132) reads both for a client that connects between pushes.
Outside the window an idle handheld's frames still carry its own posture against the app's sliders (roadmap 11.1b).

## Odometry and goal moves

`include/odometry.h` dead-reckons the robot's pose from the gait it commands, and `include/goal_controller.h` closes a move on that pose.
Both are ports of SpotMicroESP32-Leika's, to six legs and the body frame of `docs/conventions.md`: x right, y forward, yaw counter-clockwise, in metres and radians, in the frame the robot faced at the last reset.

### Odometry

`KinematicOdometry` takes the six foot targets the gait placed, before the body's tilt and shift, since those move the body over the planted feet and the robot travels only as far as the gait strokes them.
A foot is on the floor when its gait phase is in stance and it is within `LIFTED_MM` (3 mm) of the lowest foot, which keeps a foot a stopped walk is lowering from counting.
The robot's step each tick is the inverse of the least-squares rigid motion of the feet that were on the floor in both this tick and the last; the yaw comes from the fit only when at least two feet were down in both.
The stop's ease home is credited tick by tick through the same fit, as the feet that stay carry the body while the lifted ones come down; `restStep` predicts where that ease will leave the robot, for the goal's rest pose only.
It knows only what the gait commanded: servo lag, slip and short steps make the robot travel otherwise.

`FusedOdometry` corrects the heading with the gyro's yaw rate, taken from the DMP's yaw: its change between two packets over the time between them, in the body frame through `imu_frame.h`'s `YAW_SIGN`.
The chip integrates the gyro at 200 Hz, so the walk's yaw rocking does not alias into that mean as it did into one raw gyro sample held over the read period; the raw gyro is still read for the policy.
The IMU is read every 25 ms and the control runs every 5 ms, so a packet's rate is fused on the tick that read it, over the time since the last one, and only while the packet before it is at most `GYRO_STALE_S` (50 ms) old and no FIFO reset came between them; the heading follows the feet in between and across a stall.
The two rates are weighted by their variances, which puts the gyro's weight up while walking and the feet's while standing.
The gyro's bias is learned by a scalar Kalman filter on the difference of the two rates, and only while the robot is still: every foot down and no gait cycling, the body's yaw over the planted feet still, and the feet's own rate quiet, all kept up for `SETTLE_S` (0.2 s).
A stance change, a body tilt or an easing body shift is therefore taken neither for bias nor for travel.
The constants are Leika's (gyro noise measured on its Pico, the feet's error model fitted in its MuJoCo model); neither is measured on this robot.
The feet's own yaw is kept beside the fused one as `kinematicYaw`, for diagnosis.
The odometry runs in every mode that steps the gait, is paused when the motion leaves it (so no sample or stillness spans the gap), and holds its value otherwise.

The pose is in every telemetry tick (`TickSample` fields 16 to 20: `odom_x`, `odom_y`, `odom_yaw`, `odom_kinematic_yaw`, `odom_distance`), and `odometry_get` (request 130, reply 130) reads it and optionally resets it.
A reset must be asked of a robot that is still: a walk under way would be counted into the new frame.
A reset asked while a goal drives is refused with 409 and resets nothing, since it would move the goal's frame.

### Goal moves

A goal is a correlation request (`goal_move`, 131) carrying a kind and the walking style, accepted by `MotionService::startGoal` on the adapter's task and walked by the control task, through the motion inbox as every other input.
The kinds are `FORWARD` (m, negative is backwards), `SIDEWAYS` (m, positive is right in the firmware's frame, which the facade maps), `TURN` (rad, counter-clockwise positive) and `ARC` (radius in m above 0 and an angle, positive curving left; a circle is an arc of 360 deg).
The style carries `gait`, `speed`, `stride`, `s1`, `h` and `fd`.
Speed is a share in (0, 1] of the gait's top speed at the posture (`goalTops` over `gait_speed.h`'s envelope), and AUTO's is a share of BI's top.
The reply is 202 with the goal's status when accepted, 400 when malformed, 409 when the mode the robot last reported, not one requested and not yet applied, is other than `STAND` or `WALK` (deactivated, animating, or `WALK_NN`, which has no stance flags for the odometry) or a firmware update is running, and 422 for a style it cannot walk: TUNED (no speed envelope), or a posture none of whose cells walks the goal's axes.
A goal asked for while the ESP-NOW handheld drives is answered 423 (Handheld priority).
A goal accepted while a `DEACTIVATED` is queued is answered 202, and is then refused at the take with the status `REFUSED`, since the robot is deactivated by then.
A goal accepted while another runs replaces it.
A posture that is not walkable is clamped as the sliders' is.
On acceptance the goal takes the style's posture and gait through the existing WALK posture paths (the settle) and the gait swap, enters `WALK` from `STAND`, and then writes the walk's sticks itself every tick, through the path the app's commands take.
An AUTO swap at rest is not a gait change and does not cancel.

Each tick `GoalController` compares the odometry with the target in the frame the goal started in:

- **Distance:** the remaining distance along the axis sets the stick: the asked speed, slowed in proportion over the last `SLOW_DOWN_M` (50 mm), or over the distance two stops' coasts cover at that speed when longer, and never below `MIN_FRACTION` (0.2) of the asked speed.
  The start heading is held with a turn stick of `HEADING_GAIN` (2.0 per rad of fused heading error) clipped to 0.3, and the start line with a stick across it.
- **Turn:** the remaining angle on the unwrapped fused yaw, slowed over `SLOW_DOWN_RAD` (30 deg) to the same floor.
- **Arc:** a forward stick with the turn stick its radius implies, slowed on the remaining angle; the same heading gain steers the heading toward the arc's ideal heading at the distance travelled.
  That drift correction changes the arc's length as well as its end heading.

Once the remainder is within the coast the stop is predicted to take (the live command ramp stepped ahead, with `STOP_TAIL` for a foot that sets down off its stance), the sticks centre and the controller waits until the gait is not commanded and the feet are at rest, then judges the settled pose.
The tolerances are 10 mm and 3 deg, an arc's end within 30 mm and 3 deg, and a straight walk's cross-track within 20 mm.
Outside them it runs a corrective pass on the remainder, `MAX_PASSES` (3) passes in all, and then ends `TIMED_OUT`; so does a pass longer than twice its expected time plus 2 s.

A goal ends early, as `CANCELLED`, on:

- a `goal_cancel` request (132);
- a stick off centre (lx, ly or rx) on the `CommandMsg` bus from the app, a gamepad or a script, unless the inbox drops it (Handheld priority);
- the ESP-NOW handheld moving any of its axes;
- a mode change, or a gait change requested over the wire;
- the requesting client leaving, found by its adapter and id, so another client's disconnect does not end it while the app stays connected;
- 2 s (`OWNER_SILENCE_MS`) without a frame from the requesting client, for a link that went away without saying so.

While a goal runs, a command with every stick centred is ignored, posture fields included, so the app's heartbeat, the Python keep-alive and the ESP-NOW controller's centred packets do not disturb it, and the dead-man stop does not stop it.
A goal taken after the dead-man stop fired ends that stop, as a command would, and once the goal ends the stop watches the link again.
When a goal ends the robot stands at rest with no cadence left.

The status (`GoalStatus`: id, state, kind, target, progress, x, y, yaw, passes) is an observable: `observeStatus` pushes it as `Message` 300 on every state change and every 5 % of progress, never on a timer, and `goal_status_get` (133, reply 131) reads the current one.
The states are `IDLE`, `SETTLING`, `RUNNING`, `REACHED`, `CANCELLED`, `TIMED_OUT` and `REFUSED`; `SETTLING` is both the start (posture and gait) and the rest before each judging.

## Sensors

`Peripherals` (`include/peripherals/peripherals.h`) owns the MPU6050 (DMP orientation, gravity, gyro) and the HMC5883 magnetometer, each behind its `USE_*` flag, on the shared `I2CBus` (`include/peripherals/i2c_bus.h`, `esp_driver_i2c`).
The control task reads the IMU every 25 ms and publishes roll, pitch, yaw and compass heading together as one `IMUAnglesMsg`.
The bus takes its lock with a bounded wait, so a busy bus fails a transfer instead of stalling the 5 ms loop, and an I2C scan from the service side locks per address for the same reason.
Bus pins and frequency come from the persisted peripheral settings and are applied at boot only: re-opening a live bus would invalidate every device handle.
The two PCA9685 servo boards sit on the same bus: 0x41 drives the right legs (servos 0-8) and 0x40 the left (servos 9-17).
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
The LED shows the latest mode message on the bus only if that message is `APPLIED`, so it can lag during a graceful leave.
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
Each `TickSample` also carries the tick's servo write: the 18 pulses written or meant to be written (`pwm`, in PCA9685 counts), the time the write took (`servo_write_us`) and each board's result (`servo_write_right`, `servo_write_left`).
A board's result is `OK`, `FAILED` for a write it did not acknowledge, `SKIPPED` for one left out while the board backs off, which never reached the bus, `RESTARTED` for one it took after being started again, and `NONE` for a tick that wrote nothing: a mode that drives no servo, a `WALK_NN` tick on which the policy produced no angles, or a tick on which a servo calibration signal took over the pulses.
`link_lost` is set from the tick the dead-man stop zeroed the command until a command arrives or a goal takes the walk, so the tick it rises on is the tick the stop fired.
`simulation/report_recording.py` lists the failed writes and the dead-man stops of a recording.

The **ESP-NOW receiver** (`EspNowAdapter`) is input only: it turns the handheld controller's broadcasts into `CommandMsg`, `ModeMsg` and `GaitMsg` on the EventBus, so the controller drives `MotionService` as the app does, with priority over the other sources' sticks and goals while it moves a stick (Handheld priority).
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

## Virtual robot

`firmware/host/virtual_robot.h` runs the firmware's own `Hexapod` off the robot: the real `MotionService`, gait, kinematics, policy and animation runner, built against host stand-ins in `firmware/host/` for the clock, logging, FreeRTOS, the EventBus, `Peripherals` and `ServoController`.
The EventBus stand-in queues each publish with the real queue depth and delivers on `host_bus::drainAll()`, and a tick is one pass of `main.cpp`'s 5 ms control loop.
`test/test_virtual_robot/` exercises the motion pipeline through it natively, the steering dead-man's 600 ms among it.
`firmware/host/virtual_robot_api.cpp` exposes it as a C interface, which `app/scripts/build_virtual_robot.js` compiles with Emscripten (`pnpm virtual-robot`) for the app's simulation view, with the flags in `features.ini` and the packed animations embedded, and `simulation/src/sim/virtual_robot.py` compiles as a native library for the Python simulation's `FirmwareLoop`.
Anything the motion code includes must therefore build against `firmware/host/` as well: a new ESP-IDF header in that path needs a stand-in there.
