# Animation system

Named full-body animations (wave, crouch, play dead, ...) that pose the body and feet over time.
The firmware evaluates an animation every control tick into a body pose and foot targets, then runs the same inverse kinematics as the gait.
Animations are offsets from the standing posture, so they compose with ride height and the feet-distance slider.
The design rationale is in `docs/superpowers/specs/2026-09-29-animation-system-design.md`; this document describes what is built.

## The animation file

The schema is `platform_shared/animation.proto` (package `animation`).
An `Animation` holds a name (at most 32 characters of `[a-z0-9_-]`, equal to the file stem), a description, `schema = 1`, `loop`, `hold_end`, `entry_time`, `exit_time`, up to 32 keyframes, up to 8 overlays and up to 10 declared parameters.
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
The command timeout that zeroes WALK does not apply.

There are two ways in.

- A play from an active mode (STAND, a walking mode, or ANIMATE) borrows ANIMATE.
  The clip is loaded before the mode changes, so a play naming a missing or invalid file leaves the robot in its mode and pose.
  When the player is idle with nothing pending, the previous mode is handed back.
  A play from IDLE or DEACTIVATED is ignored.
- Setting the mode to ANIMATE explicitly is sticky: the robot holds stance, accepts puppeteer poses and plays, and stays until the mode changes.

Every mode decision is made on the `ModeMsg` worker, in the order the messages arrive, by `decideMode` in `firmware/include/animation/mode_arbiter.h`; `firmware/test/test_mode_arbiter` replays the interleavings.

- A borrow reaching the worker after a DEACTIVATED, IDLE or POSE is refused, the loaded play is dropped, and the standing mode is published again.
- The hand-back is only requested by the control task, as a `ModeMsg` with `handback` set, at most once until the worker has handled a mode message.
  The worker applies it only while the robot is still in a borrowed ANIMATE, so an emergency stop or a sticky ANIMATE handled first wins.
- An explicit STAND, WALK or WALK_NN while ANIMATE is playing or has a play pending does not leave at once.
  The player is stopped, the requested mode becomes the hand-back target, and the mode changes when Exit has eased every leg home.
  Leaving at once would snap joint-angle legs to stance at full servo speed.
  The requested mode is published when it arrives, so a mode observer sees it about one exit time before the robot switches.
- DEACTIVATED, IDLE and POSE apply at once even mid-play, because cutting or centring the servos immediately is the safety property.
- Any other explicit mode ends a borrow, so an explicit ANIMATE mid-play makes the robot stay in ANIMATE afterwards.

Exit returns to zero offsets, which is the neutral stance, not the body pose the STAND sliders held before the animation.
A play while playing chains from the current pose.

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
A lost chunk therefore leaves a short file that the next write at the right offset continues.
Validation decodes the file, checks the structural rules, and evaluates every keyframe and 32 intermediate times per segment, returning the first structural error or the union of clamped joints.
A clamped joint is a warning, not a refusal.

### Storage

Files live at `/littlefs/animations/<name>.pb`.
The decode buffer is allocated once in PSRAM; one animation is loaded at a time.

## Parity and fixtures

`animations/fixtures/` holds animations that exercise every path (mixed legs, overlays, parameters, single keyframe) and `expected.json`, generated from the Python reference by `simulation/gen_animation_fixtures.py`.
It contains poses at fixed times and player traces covering entry from a displaced pose, a stop during Playing, and a chained play.
Each platform checks itself against it.

- Simulation: `uv run pytest` regenerates the expectations in memory and fails while the committed file is stale.
- Firmware: `pio test -e native` runs the animation tests in `firmware/test/`, which compare the C++ evaluator and player with the fixtures state by state.
- App: a TypeScript port and its `pnpm test:unit` parity test are planned with the editor.

`uv run python check_animation.py` runs every bundled animation through the servo model and reports clamped joints, peak joint speed, tilt and falls.
Regenerate the fixtures after any behaviour change in `animation.py`.

## Bench tool

`simulation/robot_animate.py` drives the robot over the native USB Serial/JTAG port, before the app has an animation page.
The framing is `SerialAdapter`'s: a little-endian uint16 length followed by one `socket_message.Message`; a length of 0 or above 2048 makes the robot drop its buffer.
The robot only sends once it sees a host on the port.

```sh
uv run python robot_animate.py --port COM5 list
uv run python robot_animate.py --port COM5 upload wave       # ../animations/wave.json -> /animations/wave.pb
uv run python robot_animate.py --port COM5 validate wave
uv run python robot_animate.py --port COM5 play wave SPEED=1.5
uv run python robot_animate.py --port COM5 stop
uv run python robot_animate.py --port COM5 mode ANIMATE
uv run python robot_animate.py --port COM5 watch
```

## Acceptance on hardware

Not yet performed.
Flash with `pio run -t upload`, then `pio run -t uploadfs`; the seven bundled `.pb` files land in `/littlefs/animations`.
Then, on the native USB port:

1. `list` shows the seven bundled animations with sizes.
2. `validate wave` reports ok with an empty clamp mask; `validate play_dead` reports ok.
3. Put the robot in STAND from the controller or `mode STAND`, then `play wave`.
   In `watch` the status runs ENTRY, PLAYING, EXIT, IDLE and the mode returns to STAND.
   The robot leans left and back, raises the right front leg, flicks it twice, and steps home.
4. `play crouch` while `wave` is playing: the second animation enters from wherever the first is, with no jump.
5. `play wiggle`, then `stop` mid-way: the robot eases home.
6. `play play_dead`: the robot lies down and holds; `stop` brings it back over 1.2 s.
7. `mode ANIMATE`, then `play crouch`: after the play the robot stays in ANIMATE (sticky) at stance.
8. Edit `animations/wave.json` into an invalid file (a keyframe with three legs) and `upload wave`: the upload succeeds, validation reports the error, and `play wave` does nothing while the robot stays in its mode.
   Restore the file and upload again.
9. `play spooked SPEED=2`: the checker predicted a peak of 10 rad/s at speed 1, so at speed 2 the servos lag; confirm nothing worse than a softened hop.

Record the outcome of each step, including failures, in `docs/superpowers/handoffs/2026-09-30-animation-firmware-acceptance.md`.

## Known gaps

- The app's `MotionModes` does not yet know `ANIMATE`, so the app cannot select it.
- The app evaluator, editor and `/animations` route are not built.
- Controller buttons are not mapped to animations.
