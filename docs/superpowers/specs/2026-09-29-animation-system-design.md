# Animation system

Date: 2026-09-29.
Status: draft for review.

## Goal

Give the hexapod a library of expressive full-body animations: waving, stretching, looking spooked, playing dead, rolling the body through its range, and more.
Animations are authored by posing the robot in the web app, previewed there, checked in the MuJoCo simulation, and played back by the firmware.
The animation file is the single source of truth.
Every platform reads the same file and evaluates it with the same rules, so an animation that looks right in the editor behaves the same on the robot.

Success means: a new animation can be authored in the editor without touching code, played on the robot from the app or by a later controller button, and checked for feasibility in the sim before it ever runs on hardware.

## Context

An earlier attempt lives only in the git stash `animation and nets` and is documented in `docs/animation.md`.
It mirrored a keyframe engine in TypeScript and C++ with the clips hard-coded on both sides.
It drifted within days: one clip's duration differed between the mirrors, another baked frozen forward-kinematics literals, and clips were addressed by array index with nothing checking the two catalogues agreed.
Its keyframe, overlay, easing and recovery ideas were sound and are kept.
Its decision to make clips code is the defect this design removes.

Boston Dynamics' Choreographer was used as a reference for the parts that transfer: an animation as a keyframe file with declared options and bounded parameters, per-leg choice between foot position and joint angles, the robot validating a file on upload, and the robot owning playback while the tool previews.
Musical time and multi-animation sequencing are deliberately not in this cut and nothing here prevents adding them later.

Relevant existing pieces:

- `MotionService` (`firmware/include/motion.h`) selects behaviour by `MOTION_STATE`, lerps body state toward a target, and runs `Kinematics::inverseKinematics` to produce eighteen servo angles.
  The firmware has inverse kinematics only; there is no forward kinematics.
  Joint limits are `JOINT_LIMIT_DEG` in `kinematics.h`.
- Protobuf is the wire and storage format everywhere.
  `platform_shared/*.proto` compiles to nanopb for the firmware and ts-proto for the app.
- LittleFS is mounted at `/littlefs`; file list, edit and delete exist as HTTP endpoints only.
  The app on the GitHub Pages origin reaches the robot over BLE and Web Serial and has no HTTP, so anything the editor needs must travel on the protobuf request channel.
- The serial framing caps a frame at 2048 bytes.
- The simulation has a Python port of the firmware gait and IK (`simulation/src/robot/firmware_gait.py`), a calibrated servo model, and `sim_sandbox.py` with Stand, Gait and Policy modes.

## Decisions

1. **The firmware plays, the app and sim preview.**
   The animation file is uploaded to the robot and evaluated there every control tick.
   The app and the sim each carry a port of the same evaluator for preview and feasibility, kept in parity by shared golden fixtures.
   The robot therefore animates without a phone attached, playback is smooth regardless of link jitter, and the robot validates a file on upload.
   Streaming poses from the app was rejected because it makes every animation depend on a connected app and exposes BLE jitter as motion.
2. **Protobuf schema, JSON in the repo, binary on the robot.**
   The schema is `platform_shared/animation.proto`.
   Bundled animations are committed as the proto3 JSON mapping in `animations/<name>.json` at the repository root, because all three platforms consume them and JSON diffs and hand-edits.
   The robot stores the binary encoding at `/littlefs/animations/<name>.pb`.
3. **Poses are offsets from the standing posture.**
   Body offsets and foot offsets compose with the feet-distance slider and ride height.
   Per leg and per keyframe the author chooses a foot offset or three joint angles.
4. **Tunable parameters are a fixed set of multipliers.**
   An animation declares which parameters it exposes and their ranges.
   The firmware evaluator stays a table of multiplies.
5. **Transitions are part of playback, not of the file.**
   The player blends from wherever the robot is into the first keyframe, and from wherever it stops back to stance.
   A keyframe is a target, never a jump.
6. **The system noun is "animation".**
   The mode is `ANIMATE`, the route is `/animations`, the messages are `AnimationPlay`, `AnimationStop` and `AnimationStatus`.

## 1. File schema

`platform_shared/animation.proto`, package `animation`, compiled like the existing files for the firmware and the app, and additionally to Python for the simulation.

```proto
syntax = "proto3";
package animation;

// Offsets from the current standing posture. Angles rad, lengths mm.
message BodyPose { float roll = 1; float pitch = 2; float yaw = 3;
                   float x = 4;    float y = 5;     float z = 6; }

message FootOffset  { float x = 1; float y = 2; float z = 3; }         // mm from the standing foot, +z lifts
message JointAngles { float coxa = 1; float femur = 2; float tibia = 3; } // deg, IK output convention

message LegTarget { oneof target { FootOffset foot = 1; JointAngles joints = 2; } }

enum Ease { LINEAR = 0; EASE_IN = 1; EASE_OUT = 2; EASE_IN_OUT = 3; }

message Keyframe {
  float time = 1;               // seconds of animation time, strictly increasing, first is 0
  Ease ease = 2;                // shapes the segment that ends at this keyframe
  BodyPose body = 3;            // absent = zero offset
  repeated LegTarget legs = 4;  // 0 entries = every foot holds stance, else exactly 6
}

message Overlay {               // additive sine on one channel
  oneof channel { uint32 body_axis = 1; uint32 foot_channel = 2; } // body_axis 0..5 in BodyPose order, foot_channel = leg*3 + axis
  float amplitude = 3;          // mm or rad
  float frequency = 4;          // Hz of animation time
  float phase = 5;              // rad
  float start = 6; float end = 7; // active window, seconds of animation time
}

enum ParamId { SPEED = 0; BODY_X = 1; BODY_Y = 2; BODY_Z = 3;
               BODY_ROLL = 4; BODY_PITCH = 5; BODY_YAW = 6;
               FOOT_LIFT = 7; OVERLAY_AMPLITUDE = 8; REPEAT = 9; }
message ParamSpec { ParamId id = 1; float min = 2; float default_value = 3; float max = 4; }

message Animation {
  string name = 1;              // max 32, [a-z0-9_-], unique on the robot, equals the file stem
  string description = 2;       // max 96
  uint32 schema = 3;            // 1; bumped on an incompatible change
  bool loop = 4;                // repeat until stopped
  bool hold_end = 5;            // freeze on the last keyframe until stopped, else exit to stance
  float entry_time = 6;         // seconds, blend from the live pose into keyframe 0; 0 means the default 0.5
  float exit_time = 7;          // seconds, blend from the current pose back to stance; 0 means the default 0.5
  repeated Keyframe keyframes = 8;   // max 32
  repeated Overlay overlays = 9;     // max 8
  repeated ParamSpec params = 10;    // max 10, only the ids listed are exposed
}
```

`platform_shared/animation.options` fixes the nanopb sizes: `name` 32, `description` 96, `keyframes` 32, `legs` 6, `overlays` 8, `params` 10.
A fully populated `Animation` decodes to roughly 5 KB, allocated once in PSRAM; one animation is loaded at a time.

Structural validity, checked identically on every platform:

- `schema` is 1.
- at least one keyframe, the first at time 0, times strictly increasing.
- `legs` has 0 or 6 entries in every keyframe.
- overlay windows lie within `[0, last keyframe time]` and `start < end`.
- parameter ids are unique and `min <= default_value <= max`; `SPEED` must have a positive `min`.
  The field is `default_value` because `default` is a C keyword and nanopb would emit it verbatim.
- `name` matches the character set above.
- `description` is at most 96 bytes of UTF-8, because the nanopb buffer is sized in bytes.
- every keyframe `ease` is in 0..3 and every parameter `id` is in 0..9, checked on the raw integer because the proto enums are open.
- every float is finite: keyframe times, body channels, foot offsets, joint angles, every overlay field, parameter `min`, `default_value` and `max`, `entry_time` and `exit_time`.
- `loop` and `hold_end` are not both set.
- a declared `REPEAT` has `min >= 1`.

Parameter semantics:

| Id | Effect |
| --- | --- |
| `SPEED` | multiplies the animation clock: animation time advances by `dt * speed` |
| `BODY_X` .. `BODY_YAW` | multiplies that body channel after interpolation and overlays |
| `FOOT_LIFT` | multiplies every foot offset `z` |
| `OVERLAY_AMPLITUDE` | multiplies every overlay amplitude |
| `REPEAT` | number of plays of a non-looping animation, rounded to an integer; ignored when `loop` |

A parameter the animation does not declare takes its default: 1 for every multiplier and `REPEAT`, and is not shown in any UI.

JSON files use the canonical proto3 JSON mapping: lowerCamelCase field names, enums as strings, the `oneof` as whichever field is present.

## 2. Evaluator and player

### Evaluator

`evaluate(animation, params, t)` is a pure function of animation time.
It returns a pose: six body offsets and six leg targets, each a foot offset or joint angles.

1. Find the keyframe pair bracketing `t`; clamp `t` to the last keyframe's time.
2. Normalise within the segment and apply the end keyframe's easing.
3. Interpolate the body channels.
4. Per leg: if both endpoints are foot offsets, interpolate offsets; if both are joint angles, interpolate angles; if mixed, take the foot endpoint with its foot overlay and `FOOT_LIFT` applied, run IK on it against the output body of step 6, and interpolate in joint space with the raw joint endpoint, so the leg is continuous across the keyframe where it switches.
   A missing `legs` array counts as six zero foot offsets.
5. Add every overlay whose window contains `t`: `amplitude * sin(2*pi*frequency*t + phase)`.
6. Apply the multipliers: each `BODY_*` multiplies its body channel, giving the output body, and `FOOT_LIFT` multiplies each foot offset's `z` after its foot overlay; a joint-to-joint leg ignores overlays and multipliers.

Easing curves are the stash's: linear, `t*t`, `t*(2-t)`, and the piecewise ease-in-out.

### Output into the pipeline

Body offsets add to the zero body state; foot offsets add to the standing feet from `default_feet_pos`.
The result runs through the ordinary IK.
For a joint-angle leg, the three IK results for that leg are replaced by the interpolated angles.
All eighteen angles are clamped to `JOINT_LIMIT_DEG`, and the evaluator reports an 18-bit mask of which joints clamped.
The mask feeds upload validation and the editor's warnings.

IMU self-levelling is off in `ANIMATE`; a body roll would otherwise fight the levelling loop.

### Player

A state machine around the evaluator: `Entry -> Playing -> Hold | Exit -> Done`.

- **Entry.** On play, capture the live pose (current body state and feet, expressed as offsets from stance) and blend to `evaluate(animation, params, 0)` over `entry_time` with ease-in-out.
  A foot whose horizontal travel is more than a few millimetres gets a sine lift arc scaled by `min(1, travel / 40 mm)` up to 45 mm, so it steps instead of dragging.
  Blending toward a joint-angle leg happens in joint space with IK on the captured foot.
- **Playing.** Animation time advances by `dt * speed`, where `dt` is measured.
  A looping animation wraps at the last keyframe time; a non-looping one plays `REPEAT` times.
- **Hold.** When `hold_end` is set, freeze on the final pose until stopped.
- **Exit.** Blend from the current pose to zero offsets over `exit_time`, with the same stepping arc.
  Stop during any state enters Exit from the current pose.
  Play during any state skips Exit and enters the new animation's Entry from the current pose.
- **Done.** The player is idle and the firmware hands the mode back if it borrowed it.

Because the evaluator and the player are pure and time-based, the app at display rate, the sim at its control step and the firmware at 200 Hz produce the same pose for the same inputs.

### Puppeteer pose

A puppeteer pose is a `BodyPose` plus six `LegTarget`s with no timing.
The firmware lerps toward it with the STAND smoothing factor, in joint space for a joint-angle leg, and otherwise applies it through the same output path.

## 3. Firmware

### Mode

`MOTION_STATE::ANIMATE` is appended after `WALK_NN` and mirrored in `ModesEnum` and the app's `MotionModes`.
`MotionService` owns an `AnimationPlayer` and the PSRAM decode buffer.
The per-tick `ANIMATE` branch advances the player with the measured `dt` from `esp_timer`, produces the pose, runs it through IK with the joint-angle overrides and clamps, and publishes the angles like any other mode.

Two ways in:

- **A play command from any active mode** remembers the previous mode, switches to `ANIMATE`, and hands the mode back when the player reaches Done.
- **Setting the mode to `ANIMATE` explicitly** is sticky: the robot holds stance, accepts puppeteer poses and play commands, and stays until the mode is changed.

Exit returns to zero offsets, which is the neutral stance, not the body pose the STAND sliders held before the animation.
The command timeout that zeroes motion in WALK does not apply; a puppeteer pose is held if the stream stops, as STAND holds its body pose.

### Messages

Added to the `Message` oneof in `message.proto`, carried on every transport:

| Message | Direction | Semantics |
| --- | --- | --- |
| `AnimationPlay { string name; repeated AnimationParam { ParamId id; float value; } params; }` | to robot | loads `/littlefs/animations/<name>.pb`, validates, enters Entry. Missing params take defaults, values are clamped to the declared range, undeclared ids are ignored. A play while playing chains. |
| `AnimationStop {}` | to robot | enters Exit from the current pose. |
| `PoseData { BodyPose body; repeated LegTarget legs; }` | to robot | puppeteer target, applied only in `ANIMATE` when the player is idle. |
| `AnimationStatus { string name; AnimationState state; float t; uint32 clamped_mask; }` | from robot, observable | emitted on every state change and at 5 Hz while the player is not idle, following the emit-on-change pattern. `AnimationState` is `IDLE, ENTRY, PLAYING, HOLD, EXIT`. |

Added to `CorrelationRequest` and `CorrelationResponse`:

| Request | Response |
| --- | --- |
| `FileWriteChunk { string path; uint32 offset; uint32 total_size; bytes content; }` | status. Offset 0 truncates, the chunk whose end equals `total_size` closes the file. Content at most 1024 bytes so a frame stays under the 2048 byte ceiling. Generic, any path. |
| `FileReadChunk { string path; uint32 offset; uint32 length; }` | `FileChunk { bytes content; uint32 total_size; }`, `length` at most 1024. |
| `FileDeleteRequest` (existing) | status. |
| `AnimationValidate { string name; }` | `AnimationReport { bool ok; string error; uint32 clamped_mask; }`. Decodes the file, checks the structural rules, evaluates every keyframe and 32 evenly spaced intermediate times per segment, and returns the first structural error or the union of clamped joints. |
| `AnimationListRequest {}` | `AnimationList { repeated { string name; uint32 size; } }`. |

The play handler runs the same validation and refuses a file that fails a structural rule; a clamped joint is a warning in the status, not a refusal.

### Storage and build

Animations live at `/littlefs/animations/<name>.pb`, the nanopb encoding, one file per animation, the file stem equal to the `name` field.
Bundled animations are `animations/<name>.json` in the repository.
`firmware/scripts/pack_animations.py`, run as a PlatformIO pre-script, converts them into `firmware/data/animations/<name>.pb` (gitignored) so `pio run -t uploadfs` ships them.

### Controller

The ESP-NOW packet has three auxiliary buttons.
A later settings entry maps each to an animation name.
Not in this cut.

## 4. App editor

Route `/animations` with two views over the same parts.

**Library.** Lists built-in animations (the JSON files imported at build time), animations on the connected robot (`AnimationListRequest`), and drafts in browser storage.
Each row has a play button, a stop button, and the animation's declared parameter sliders.

**Editor** for one animation.

- **3D view** on the left, reusing `SceneBuilder` and the URDF model, driven by the TypeScript evaluator and kinematics.
  A leg whose IK clamps is drawn red and its clamped joint is named.
- **Pose panel** for the selected keyframe: six body sliders; per leg a foot/joints toggle with three offset fields plus a drag handle in the 3D view in foot mode, or three angle sliders in joint mode.
  Flipping a leg from joints to foot converts through forward kinematics as a convenience; the stored value is whatever mode the leg is in.
- **Timeline strip** along the bottom: keyframes as markers on a time axis, drag to retime, click to select, duplicate, delete, ease per keyframe, a scrub head, play, pause, loop, and the speed slider.
  Overlays are a list under the strip with channel, amplitude, frequency, phase and window.
- **Animation panel**: name, description, loop, hold at end, entry and exit times, the exposed parameters with min, default and max, and the file actions.

**"Show on robot" toggle.** When on and linked, the editor sets the mode to `ANIMATE` and streams a `PoseData` through the existing throttler at 20 Hz on every pose change, including scrubbing and preview playback.
When unlinked the flag does nothing.
**"Play on robot"** uploads the current animation in chunks, requests validation, shows the report, and sends `AnimationPlay` with the current parameter values; the 3D view keeps showing the local preview.

**Files.** New from a blank stance; open from a JSON file, the built-in library, browser storage, or the robot via `FileReadChunk`.
Save downloads the JSON.
Browser storage keeps an autosaved draft as a per-browser convenience only, wrapped in try/catch and never relied on.
Upload writes chunks, validates, then refreshes the robot list.

**Code.** `app/src/lib/animation/evaluator.ts` and `player.ts` mirror the firmware line for line; `transfer.ts` does chunked upload and download; `stores/animation-editor.ts` holds the document and selection.
The route is thin Svelte over those.
The stash's `legFromAngles` helper is the forward-kinematics convenience above.

## 5. Simulation

`simulation/src/robot/animation.py` holds the Python evaluator and player, ported from the firmware like the gait.
The simulation reads the JSON files through the generated Python protobuf classes and `json_format`, so it parses exactly what the other platforms parse.

`sim_sandbox.py` gains an **Animate** mode beside Stand, Gait and Policy: a picker over `animations/*.json`, the exposed parameter sliders, play, stop and scrub, with the pose fed through the Python IK into the calibrated servo model.
This is where dynamic feasibility is learned: whether play-dead lowers the body without a leg pushing the robot over, whether a fast wave tips it, whether the servos keep up at the authored speed.

`simulation/check_animation.py` runs the same headless over every file in `animations/` and prints, per animation, the clamped joints, the peak joint speed against the servo no-load speed, the peak body tilt, and whether the body fell.
It exits non-zero on a fall or a structural error so it can run in CI.

## 6. Testing

**Parity fixtures.** `animations/fixtures/` holds animations chosen to exercise every path: mixed foot and joint targets on one leg, every ease, an overlay, each parameter, and `expected.json` with poses at fixed animation times plus a player trace covering entry from a displaced pose, a stop during Playing, and a chained play.
`simulation/gen_animation_fixtures.py` writes `expected.json` from the Python implementation.

- The firmware native test (`firmware/test/test_animation/`) loads the fixtures and asserts the C++ evaluator and player match within `1e-4`.
- `pnpm test:unit` asserts the same for the TypeScript evaluator and player.
- `uv run pytest` regenerates the expectations in memory and compares them with the committed file, so a stale fixture fails.

**Firmware, beyond parity**: nanopb round trip of an `Animation` at the size limits; chunked write and read on the host filesystem; the validator's report for a keyframe with an unreachable foot and for each structural rule.

**App**: evaluator and player parity; chunking against a mock transport including a lost chunk; JSON round trip of a document through the editor store; the throttled puppeteer stream sends at most 20 messages per second.

**Simulation**: parity generation, `check_animation.py` over the bundled library passes.

**Hardware acceptance**, manual:

1. Upload each bundled animation from the library page and play it.
2. Turn on "Show on robot" and drag a foot; the robot follows.
3. Play on robot from the editor; the status shows Entry, Playing, Exit, Done and the mode returns to STAND.
4. Chain two animations; the second enters from the first's current pose without a jump.
5. Stop one mid-way; the robot steps home.

## Bundled animations

The first library, authored in the editor and committed as JSON:

- `wave`, `crouch`, `wiggle`: ported from the stash's presets, with `wiggle` as a looping overlay-only animation.
- `stretch`: front legs reach forward and the body sinks, then rear legs, then back to stance.
- `spooked`: a fast body jump up and back with a small roll, then a slow settle.
- `play_dead`: body to the ground with the legs raised by joint angles, `hold_end` set.
- `body_roll_test`: roll, pitch and yaw swept one at a time to their usable limits, for checking the mechanics.

The stash's `slam` is not ported until the sim says it is safe at the servo's real speed.
Clip values are recovered from `git stash show -p stash@{0}`.

## Out of scope

- Sequencing several animations on a timeline, musical time, and sound.
- Mapping controller buttons to animations.
- Re-enabling self-levelling during Hold.
- Per-channel easing, spline interpolation, and custom parameter bindings to individual channels.
- Merging the stash; it remains a reference only.

## Compatibility

`ANIMATE` shifts no existing enum value, but it crosses the wire by position, so the app and the firmware ship together.
`docs/animation.md` is superseded by this design and is rewritten once the implementation lands.
