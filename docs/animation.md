# Animation system

Procedural gesture playback: named clips ("wave", "slam") that pose the body and feet over time, evaluated into a body state that flows through the *same* IK as the gait.
Because animations reuse the gait's IK and foot layout, they compose with body translation, rotation, and the feet-distance slider rather than fighting them.

This document exists because the feature is being shelved.
It records the parts that are not obvious from the code, and the one place the two implementations have already drifted.

## Where it lives

| Concern | Firmware | App |
| --- | --- | --- |
| Engine | `firmware/include/animation.h` (`anim::Animator`) | `app/src/lib/animation.ts` (`Animator`) |
| Clip catalogue | same file, `anim::CLIPS[]` | `app/src/lib/animations/presets.ts` |
| Mode plumbing | `firmware/include/motion.h`, `MOTION_STATE::ANIMATE` | `app/src/lib/motion.ts`, `MotionModes.ANIMATE` |
| Wire message | `AnimationMsg` (`message_types.h`), topic `ANIMATION = 15` | `requestAnimation` / `stopAnimation` in `app/src/lib/control.ts` |

The two engines are deliberate mirrors, like the kinematics and gait: the app renders a local preview with no robot attached, the firmware runs the real thing.

## The sync contract

**Clips are addressed by array index, not by name.**
`AnimationMsg.index` is an index into `anim::CLIPS[]`, and the app sends `presets.indexOf(clip)`.
A negative index means *stop*.

So `anim::CLIPS[]` and `presets` must stay in the same order.
Reordering one array silently plays the wrong gesture, and there is no handshake that would detect it.

## Data model

A **Pose** is a 6-DOF body offset plus one 3-vector foot offset per leg:

- body: `omega, phi, psi` (roll/pitch/yaw, **radians**) and `xm, ym, zm` (translation, **mm**)
- feet: `[x, y, z]` per leg, **mm offsets from that leg's default standing position**

Offsets, not absolute positions — that is what makes a clip independent of the current posture and of the feet-distance slider.

A **Keyframe** is `{ t, ease, pose }` with `t` normalized to `[0, 1]`.
Note the easing convention: **`ease` shapes the segment that *ends* at that keyframe**, not the one that starts there.

An **Overlay** is a procedural sine/cosine added *on top of* the interpolated keyframe pose, within a normalized time window: `{ target: feet|body, leg, axis, fn, amp, freq, phase, window }`.
`freq` counts cycles over the whole clip duration.
Overlays are how "wiggle" works with only two (empty) keyframes.

A **clip** is `{ name, duration (ms), loop, keyframes, overlays }`.

### Evaluation, per tick

1. Advance elapsed time; `t = elapsed / duration`, wrapped with `fmod` when `loop`, clamped to 1 otherwise.
2. Find the bracketing keyframe pair, normalize `t` within that span, apply the *end* keyframe's easing.
3. Lerp all six body channels and all eighteen foot channels.
4. Add every overlay whose window contains `t`.
5. Write body channels into the body state, and each foot as `defaultFeet[leg] + offset`.

A non-looping clip returns "finished" once elapsed exceeds duration, which triggers recovery.

## Sign and coordinate conventions

These are the traps. All of them are load-bearing:

- Leg indices: `0,1,2` = right front/mid/rear, `3,4,5` = left front/mid/rear.
- **`+z` lifts a foot.** Never push a foot down. A negative foot `z` inflates the ground-contact term and launches the whole robot upward, so clips only ever lift.
- **Negative `zm` moves the body UP; positive `zm` crouches it DOWN.** The visualization renders height as `-zm/12`. This inverts the intuition for every clip that changes ride height.
- **Forward is negative `ym`.**
- Body angles are radians, everything positional is mm.

## Authoring a pose by joint angles

Guessing an xyz foot offset for "raise the leg with the knee bent 90°" is painful, so the app has `legFromAngles(leg, coxa, femur, tibia)` in `presets.ts`.
It runs forward kinematics on one leg and returns the resulting **offset from the standing pose** — the same offset the rest of the pipeline consumes, so an angle-authored leg still goes through IK and still composes with body motion.

**The firmware has no equivalent.**
It cannot run the authoring helper, so those values are baked in as literals — see `slamRaise()` in `animation.h`, where `{-77.73, 57.27, 120.72}` is the precomputed result of `legFromAngles(0, 45, 90, -90)`.

Consequence: **if the kinematics config or the default posture (`genPosture(60, 75)`) changes, those literals are stale and must be regenerated** from the app side. Nothing checks this.

## Mode plumbing and recovery

Starting a clip (`handleAnimation` in firmware, `requestAnimation` in the app):

1. Remember the current mode in `previousMode` (only if not already animating).
2. Switch to `ANIMATE`, activate servos, `animator.play(clip)`.

Each tick in `ANIMATE`: advance the animator, and when the clip reports finished, begin **recovery**.

Recovery eases the body and feet back to the standing pose over `RECOVER_DURATION` (0.7 s) with an `easeInOut` curve, and adds a `sin(πt)` arc of up to `RECOVER_LIFT` (45 mm) so displaced feet **step home instead of dragging**.
The lift is scaled by how far each foot actually moved (`min(1, horiz / 40)`), so planted feet stay planted.
When recovery completes, the mode is handed back to `previousMode` — animations are a temporary excursion, not a destination.

The app additionally short-circuits: if the body and feet are already within a small tolerance of standing, it finishes recovery immediately instead of running a pointless 0.7 s ease.

Stopping mid-clip (negative index) also routes through recovery rather than snapping.

## Preset catalogue

| Clip | Duration | Loop | What it does |
| --- | --- | --- | --- |
| `wave` | 2400 ms | no | Leans away from the front-right leg (`xm` shift, `omega`/`phi` tilt) with the other five feet counter-offset so they stay planted, lifts leg 0, then two up/down winks |
| `slam` | see drift below | no | Rears up and cocks both front legs with a ~90° knee bend, then lunges the body forward and drops flat, accelerating in with `easeIn` |
| `crouch` | 600 ms | no | Squat down and back up (`zm: 26`) |
| `wiggle` | 2400 ms | **yes** | Two empty keyframes; all motion comes from three body overlays (roll sine, yaw cosine, height sine offset by π/2) |

## Known drift and open issues

1. **`slam` duration disagrees between the mirrors** — 750 ms in `presets.ts`, 1100 ms in `animation.h`. The robot performs the gesture noticeably slower than the on-screen preview. Pick one before resuming work.
2. **`slamRaise()` literals are frozen FK output** and will silently go wrong if the leg geometry or default posture changes.
3. **Index coupling has no guard.** Adding, removing, or reordering a clip in one place and not the other misplays silently. A name field exists in both but is never checked over the wire.
4. Overlays are only additive on top of keyframes; there is no way to have an overlay replace or scale a channel.
5. Easing is per-segment and fixed to four curves; no per-channel easing and no spline interpolation.

## Adding a clip

1. Author it in `app/src/lib/animations/presets.ts`, using `legFromAngles` for articulated poses.
2. Append to the `presets` array — **append**, never insert.
3. Mirror it in `firmware/include/animation.h`: keyframe array, any overlay array, and an entry appended to `CLIPS[]` at the same index.
4. Bump `CLIP_COUNT`.
5. Verify duration, loop flag, and every literal match between the two.
6. Because the app and firmware are index-coupled, ship both together.
