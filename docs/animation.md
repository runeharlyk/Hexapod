# Animation system

Named full-body animations (wave, crouch, play dead, ...) that pose the body and feet over time.
The firmware evaluates an animation every control tick into a body pose and foot targets, then runs the same inverse kinematics as the gait.
Animations are offsets from the standing pose at the current feet distance; the runner adds a base ride height to the body `z` before IK (see [Ride height](#ride-height)).
The design rationale is in `docs/superpowers/specs/2026-09-29-animation-system-design.md`; this document describes what is built.

## The animation file

The schema is `platform_shared/animation.proto` (package `animation`).
An `Animation` holds a name (at most 32 characters of `[a-z0-9_-]`, equal to the file stem), a description, `schema = 1`, `loop`, `hold_end`, `entry_time`, `exit_time`, up to 32 keyframes, up to 8 overlays, up to 10 declared parameters and an optional `ride_height` (mm, finite).
A keyframe has a time in seconds (strictly increasing, the first at 0), an ease, a body pose and 0 or 6 leg targets.
The ease shapes the segment that ends at that keyframe.
A leg target is either a foot offset (mm from the standing foot) or three joint angles (degrees, IK output convention: coxa yaw, absolute femur, tibia relative to femur).
An overlay is an additive sine on one body axis or one foot channel (`leg * 3 + axis`) inside a time window.
Parameters are a fixed set of multipliers: `SPEED`, `BODY_X` to `BODY_YAW`, `FOOT_LIFT`, `OVERLAY_AMPLITUDE`, `REPEAT`.
An animation exposes only the ids it declares, with a minimum, default and maximum.

Bundled animations are proto3 JSON in `animations/<name>.json` at the repository root: `wave`, `crouch`, `wiggle`, `stretch`, `spooked`, `play_dead`, `body_roll_test`.
The robot stores the binary encoding in `/littlefs/animations/<name>.pb`.
`firmware/scripts/pack_animations.py` converts the JSON into `firmware/data/animations/*.pb` on every firmware build, and `pio run -t uploadfs` ships them.

## Sign and coordinate conventions

- Positive body `z` crouches the robot; negative body `z` raises it (the firmware `zm` convention).
- Positive body `x` and `y` move the body toward negative `x` and `y`, because the feet are what the IK places.
- Positive foot `z` lifts a foot.
  Never push a foot below the ground: a negative foot `z` inflates the ground-contact term and launches the robot.
- Body axes follow the gait: X lateral, Y forward, Z up, and the robot faces +Y.
- Body angles are radians, lengths millimetres, joint angles degrees, time seconds.

## Evaluator and player rules

The reference is `simulation/src/robot/animation.py`; the firmware (`firmware/include/animation/`) and the app mirror it line for line.
A port must honour these rules.

- File values are float32.
  A port that parses JSON rounds every number to float32 before use.
- The evaluate order is: body interpolation, then body overlays and per-leg foot overlay accumulation, then the body multipliers, then the legs.
  A leg whose endpoints are both feet interpolates offsets, both joints interpolates angles, and mixed runs IK against the output body on the foot endpoint (with its overlay and lift applied) and interpolates in joint space with the raw joint endpoint.
- An overlay on a joint leg is ignored, and so are the multipliers.
- The joint clamp is symmetric `+-JOINT_LIMIT_DEG` in IK-output space.
  An unreachable foot sets that leg's femur and tibia bits in the clamp mask.
  The mask has bit `leg * 3 + joint`.
- `REPEAT` is `max(1, floor(x + 0.5))`.
  It is ignored when `loop` is set.
- Entry and exit default to 0.5 s and run on wall time; `SPEED` scales only the playing clock.
- The step arc is `45 mm * min(1, travel / 40 mm) * sin(pi * u)`, applied to foot legs whose horizontal travel exceeds 2 mm.
- `play` records the live pose as `lastPose`, and Exit starts from the final keyframe.
- Stance feet are an input (`default_feet_pos`), never a constant.
- The player runs Entry, Playing, then Hold (when `hold_end`) or Exit, then Done.
  Stop in any state enters Exit from the current pose; play in any state skips Exit and enters the new animation's Entry from the current pose.
- Structural validation is identical on every platform; the rules are listed in the spec section 1 and implemented by the validator in `animation.py` and in `firmware/include/animation/animation.h`.

## Firmware

### Mode, borrow and hand-back

`MOTION_STATE::ANIMATE` is mode 6 (`ModesEnum.ANIMATE`).
The `MotionService` ANIMATE branch advances the player with the measured `dt`, runs IK with the joint-leg overrides and clamps, and publishes the angles like any other mode.
IMU self-levelling is off in ANIMATE.
While the player is not idle, `AnimationRunner` logs at INFO, at most every 5 s, the worst tick cost (evaluation, IK and status) in microseconds: `tick max <n> us over the last 5 s of playing`.
An idle player restarts the window, so a play shorter than 5 s logs nothing; `wiggle` loops and is the one to read it with.
The command timeout that zeroes WALK does not apply, and it does not zero the ride-height slider while in ANIMATE: its window restarts every tick, so after a hand-back the app has a full window to resume its heartbeat.

There are two ways in.

- A play from an active mode (STAND, a walking mode, or ANIMATE) borrows ANIMATE.
  The clip is loaded before the mode changes, so a play naming a missing or invalid file leaves the robot in its mode and pose.
  When the runner is no longer busy (see below), the previous mode is handed back.
  A play from IDLE or DEACTIVATED is ignored.
- Setting the mode to ANIMATE explicitly is sticky: the robot holds stance, accepts puppeteer poses and plays, and stays until the mode changes.

Every mode decision is made on the `ModeMsg` worker, in the order the messages arrive, by `decideMode` in `firmware/include/animation/mode_arbiter.h`; `firmware/test/test_mode_arbiter` replays the interleavings.

- A borrow reaching the worker after a DEACTIVATED, IDLE or POSE is refused, the loaded play is dropped, and the standing mode is published again as `APPLIED`.
- The hand-back is only requested by the control task, as a `ModeMsg` of kind `HANDBACK`, at most once until the worker has handled a mode message.
  The worker applies it only while the robot is still in a borrowed ANIMATE and the runner is not busy, so an emergency stop or a sticky ANIMATE handled first wins, and a play chained after the request keeps the borrow; the control task asks again once the worker has cleared its flag and the runner is idle at stance.
- `AnimationRunner::busy()` is true while the player is not idle, a play is pending, or the pose is away from stance (any joint-angle leg, or a body or foot offset above 1 mm or 0.01 rad).
  A finished Exit is restated as zero foot offsets, so a leg that blended home in joint space does not keep the runner busy.
- An explicit STAND, WALK or WALK_NN while the runner is busy does not leave at once.
  A running player is stopped, and a pose held by an idle player (a puppeteer pose) eases home through the puppet path toward stance; the requested mode becomes the hand-back target, and the mode changes once every leg is home.
  Leaving at once would snap joint-angle legs to stance at full servo speed.
  The requested mode is published when it arrives, so a mode observer sees it about one exit time before the robot switches.
- DEACTIVATED, IDLE and POSE apply at once even mid-play, because cutting or centring the servos immediately is the safety property.
- Any other explicit mode ends a borrow, so an explicit ANIMATE mid-play makes the robot stay in ANIMATE afterwards.

A `ModeMsg` carries a `ModeMsgKind` (`firmware/include/message_types.h`):

- `REQUEST` is a mode asked for by a client, the ESP-NOW controller, the OTA service or the firmware itself; every `{MOTION_STATE::X}` initialiser is one.
- `BORROW` is a play asking for ANIMATE, and `HANDBACK` is the control task asking for the borrowed mode back; the worker decides both against the mode at delivery.
- `APPLIED` is `MotionService` reporting the mode it switched to, or kept, for a `BORROW` or `HANDBACK`; the worker ignores it.

The `ModeData` bridge to the clients and the ESP-NOW adapter's mode mirror pass only `REQUEST` and `APPLIED`.
A client therefore sees every requested mode when it arrives and every mode the animation borrow or hand-back actually produced, and never a borrow that was refused or a hand-back that was ignored.

Losing control stops the animation.
When a client goes (a WebSocket closes, the BLE central disconnects, or the USB host disappears) and no transport has a client left, `MotionService` asks a running or held animation to stop; a borrowed mode then hands back as usual, and a sticky ANIMATE stays in ANIMATE at stance.
Outside ANIMATE a play that is loaded but whose borrow has not landed yet is dropped, so it cannot start unattended.
A puppeteer pose is still held on a silent stream while a client remains connected.
Each adapter answers `hasClient()`, and `CommAdapterBase::onClientGone` fires after a client's subscriptions are dropped; `main.cpp` checks every adapter there.

Exit returns to zero offsets on the slider's ride height, not the body pose the other STAND sliders held before the animation.
A play while playing chains from the current pose.

### Ride height

The evaluator's body `z` is an offset; the runner (`AnimationRunner`) adds a base ride height before IK.
When the file has no `ride_height`, the base is the current ride-height slider (the STAND target `zm`), so an animation played on a tall-standing robot stays tall and Exit returns to that height.
When the file sets `ride_height`, the base is that value while the player is not idle, regardless of the slider, because some animations only work at one height; Exit still returns to the slider's height.
The base eases toward its target with the STAND smoothing factor, so the change between the two is never a step.
During Entry the base instead moves along the Entry blend (eased like the pose) from the base at the play to the base Entry converted its destination at, so it arrives exactly when Entry ends and the Entry-to-Playing seam is continuous however short the Entry; Exit keeps the base of the stop, at which it converted both of its ends.
Puppeteer poses use the slider base.
Entering ANIMATE starts the base at the slider and captures only the height beyond it as an offset, so the STAND height is not treated as part of the pose.
The base is added to the body by each platform's runner, not by the evaluator, so the evaluated body stays an offset and the parity fixtures are unaffected; the sim sandbox adds it the same way in Animate mode, without the lerp.
The base does enter every foot-to-joint conversion: `legJointsDeg` (`leg_joints_deg`), `evaluate` for a mixed leg, and the player's Entry and Exit blends and its Playing and Hold evaluation take the base, so a leg switching between a foot and joint angles is continuous at any base.
Entry converts its start at the live base and its target at the base held once Entry ends (the file's `ride_height` when set, else the live base); Exit converts both ends at the live base.

The stance itself has only about 5 mm of femur travel left at the crouched (`+50` mm) end of the slider, so no bundled animation fits the whole slider range, and all seven set `ride_height: 0`.
`simulation/test_animation_library.py` checks every keyframe and midpoint on each base the runner may add: the fixed `ride_height`, or both slider ends (`-50` and `+50` mm) for an animation that follows the slider.
The robot's validate request sweeps the same bases and reports the union of the clamped joints.

### Puppeteering

A `PoseData` message (a `BodyPose` plus 0 or 6 `LegTarget`s) is applied only in ANIMATE while the player is idle.
The firmware lerps toward it with the STAND smoothing factor and holds it if the stream stops.

### Messages

Wire tags as in `platform_shared/message.proto` and `api.proto`.

| Message | Tag | Direction |
| --- | --- | --- |
| `Message.animation_play` (`name`, repeated `AnimationParam { ParamId id; float value }`) | 280 | to robot |
| `Message.animation_stop` | 281 | to robot |
| `Message.pose` (`PoseData`) | 282 | to robot |
| `Message.animation_status` (`name`, `AnimationState state`, `t`, `clamped_mask`) | 283 | from robot, observable |
| `Message.mode` (`ModeData`) | 130 | both |
| `Message.sub_notif` | 20 | to robot |

`AnimationState` is `ANIM_IDLE`, `ANIM_ENTRY`, `ANIM_PLAYING`, `ANIM_HOLD`, `ANIM_EXIT`.
A status is pushed on every state change and at 5 Hz while the player is not idle; a client subscribes to tag 283.
Missing play parameters take their defaults, values are clamped to the declared range, and undeclared ids are ignored.

Correlation requests, answered on the same correlation id:

| Request | Tag | Response | Tag |
| --- | --- | --- | --- |
| `file_write_chunk` (`path`, `offset`, `total_size`, `content`) | 100 | status code only | |
| `file_read_chunk` (`path`, `offset`, `length`) | 101 | `file_chunk` (`content`, `total_size`) | 101 |
| `file_delete` | 102 | status code only | |
| `animation_validate` (`name`) | 110 | `animation_report` (`ok`, `error`, `clamped_mask`) | 110 |
| `animation_list_request` | 111 | `animation_list` (`entries` of `name`, `size`) | 111 |

The generic `empty` response is tag 5.
Chunks carry at most 512 bytes, which keeps a frame well under the 2048-byte serial ceiling.
A write is refused with 400 when the chunk is larger than 512 bytes, runs past `total_size`, or does not continue the file at its current size; the path must be absolute and free of `..`.
Offset 0 truncates.
Any status other than 200 on a chunk means the client restarts the file from offset 0, or reads the size back with `file_read_chunk` and continues from there.
A failed `fwrite` can leave a partial chunk in the file, so the size read back, not the last acknowledged offset, is where a continuation starts.
Validation decodes the file, checks the structural rules, and evaluates every keyframe and 32 intermediate times per segment on the file's `ride_height`, or on both slider ends (`-50` and `+50` mm) when it has none, returning the first structural error or the union of clamped joints.
An invalid file is answered with status 422 and the report filled in (`ok` false and the `error`).
A clamped joint is a warning, not a refusal.

### Storage

Files live at `/littlefs/animations/<name>.pb`.
Three clip buffers (the active clip, the pending clip a play loads beside it, and one for validation) and the encoded and decoded file buffers are allocated once in PSRAM.
Loads run under one lock, so one file is decoded at a time.

## Parity and fixtures

`animations/fixtures/` holds animations that exercise every path (mixed legs, overlays, parameters, single keyframe) and `expected.json`, generated from the Python reference by `simulation/gen_animation_fixtures.py`.
It contains poses at fixed times and player traces covering entry from a displaced pose, a stop during Playing, and a chained play.
Each platform checks itself against it.

- Simulation: `uv run pytest` regenerates the expectations in memory and fails while the committed file is stale.
- Firmware: `pio test -e native` runs the animation tests in `firmware/test/`, which compare the C++ evaluator and player with the fixtures state by state.
  The C++ test reads `expected.txt`, a plain-text dump of the same data written by the same script; `expected.json` is the Python side's file.
- App: a TypeScript port and its `pnpm test:unit` parity test are planned with the editor.

`uv run python check_animation.py` runs every bundled animation through the servo model and reports clamped joints, peak joint speed, tilt and falls.
Regenerate the fixtures after any behaviour change in `animation.py`.

## Bench tool

`simulation/robot_animate.py` drives the robot over the native USB Serial/JTAG port, before the app has an animation page.
The framing is `SerialAdapter`'s: a little-endian uint16 length followed by one `socket_message.Message`; a length of 0 or above 2048 makes the robot drop its buffer.
The robot only sends once it sees a host on the port.
A COM port is exclusive on Windows, so `watch` cannot run beside another command.
`play` and `stop` therefore subscribe to the status (tag 283) and mode (tag 130) topics themselves and print every status and mode change by name.
They return after a mode change that follows an idle status, which is the hand-back, or after 2 s without traffic, which covers a sticky ANIMATE, a refused play and a stop with nothing playing.
They also stop following, and say why, on the first `ANIM_HOLD` status and after 2 s of `ANIM_PLAYING` whose clock has wrapped (a loop, or a repeat), because neither settles on its own; Ctrl-C stops following too.
`shell` opens the port once and reads the same commands (`list`, `upload`, `validate`, `play`, `stop`, `mode`, `watch`, `quit`) from stdin, so a looping or holding animation can be stopped, and a play chained, while the port stays open.
In the shell, Ctrl-C ends the current command, including a `list`, `upload` or `validate` waiting on the robot, and returns to the prompt with the port still open.
The tool opens the port with DTR and RTS held low, so opening it should not reset the robot.

```sh
uv run python robot_animate.py --port COM5 list
uv run python robot_animate.py --port COM5 upload wave       # ../animations/wave.json -> /animations/wave.pb
uv run python robot_animate.py --port COM5 validate wave
uv run python robot_animate.py --port COM5 play wave SPEED=1.5
uv run python robot_animate.py --port COM5 stop
uv run python robot_animate.py --port COM5 mode ANIMATE
uv run python robot_animate.py --port COM5 watch
uv run python robot_animate.py --port COM5 shell
```

## Acceptance on hardware

Not yet performed.
Flash with `pio run -t upload`, then `pio run -t uploadfs`; the seven bundled `.pb` files land in `/littlefs/animations`.
Watch the robot the first time the tool opens the port.
A reboot shows as the servos going limp, and the next `play` from STAND prints no mode change because the robot came back up DEACTIVATED.
If that happens, record it, put the robot back in STAND with `mode STAND` before every step, since each command opens the port again, and report it so the port handling can be fixed.

Then, on the native USB port:

1. `list` shows the seven bundled animations with sizes.
2. `validate wave` reports ok with an empty clamp mask; `validate play_dead` reports ok.
3. Put the robot in STAND from the controller or `mode STAND`, then `play wave`.
   `play` prints the mode ANIMATE, the status running ENTRY, PLAYING, EXIT, IDLE, and the mode STAND.
   The robot leans left and back, raises the right front leg, flicks it twice, and steps home.
4. Open `shell` for steps 4 to 6.
   `play wave`, press Ctrl-C mid-play to return to the prompt, then `play crouch`: the second animation enters from wherever the first is, with no jump.
5. `play wiggle`: following ends after about 2.4 s with the looping message while the robot keeps wiggling.
   Wait for the `tick max` INFO line (every 5 s while playing) and record it; `stop` then eases the robot home and hands STAND back.
   The log is on the UART0 console, a separate port from the native USB Serial/JTAG port the shell holds, so read it with a second monitor on that port.
6. `play play_dead`: the robot lies down and holds, and following ends with the holding message; `stop` brings it back over 1.2 s.
7. `mode ANIMATE`, then `play crouch`: after the play the robot stays in ANIMATE (sticky) at stance.
8. Edit `animations/wave.json` into an invalid file (a keyframe with three legs) and `upload wave`: the upload succeeds, validation reports the error, and `play wave` does nothing while the robot stays in its mode.
   Restore the file and upload again.
9. `play spooked SPEED=2`: the checker predicted a peak of 10 rad/s at speed 1, so at speed 2 the servos lag; confirm nothing worse than a softened hop.
10. Control loss: with no app connected over WiFi or BLE, put the robot in STAND, `play wiggle` from the shell, and unplug the USB cable mid-wiggle with the robot on battery.
    The robot eases home and returns to STAND.
    Closing the tool alone is not a disconnect: the serial adapter sees the host through USB start-of-frame packets, which continue while the port is closed.
11. Ride height: with the app open on the robot-hosted page, put the robot in STAND and move the height slider to the end that raises the body.
    `play play_dead` and `stop` from the shell: the robot settles to the file's `ride_height` 0 while it plays, and Exit returns it to the raised height rather than to zero.

Record the outcome of each step, including failures, in `docs/superpowers/handoffs/2026-09-30-animation-firmware-acceptance.md`.

## Known gaps

- The app's `MotionModes` does not yet know `ANIMATE`, so the app cannot select it.
- The app evaluator, editor and `/animations` route are not built.
- Controller buttons are not mapped to animations.
- The serial adapter cannot tell a closed port from an open one, so only the end of the USB session counts as the serial host gone for the control-loss rule.
