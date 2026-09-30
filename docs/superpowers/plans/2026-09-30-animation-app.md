# Animation Web App Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give the web app an animation library and editor: a TypeScript port of the reference evaluator and player proven against the parity fixtures, chunked transfer and the play, stop, pose and status messages over the existing data broker, a library page that plays animations on the robot with their parameter sliders, and an editor that poses the robot in the 3D view, keeps keyframes on a timeline, previews locally, mirrors to the robot when asked, and saves the file.

**Architecture:** The generated `Animation` type from `platform_shared/animation.proto` is the document model everywhere in the app; it already carries `fromJSON`, `toJSON`, `encode` and `decode`.
`app/src/lib/animation/evaluator.ts` and `player.ts` are line-for-line ports of `simulation/src/robot/animation.py` on top of the app's own `Kinematics` mirror, checked against `animations/fixtures/expected.json`.
`transfer.ts` wraps the data broker's `request()` for chunked upload, download, list, validate and delete.
A small `preview` store lets the editor drive the existing 3D view instead of the gait.
The route `/animations` holds the library and the editor as two views over the same stores.

**Tech Stack:** SvelteKit with Svelte 5 runes, Tailwind 4 and daisyUI 5, three.js 0.162 with urdf-loader, ts-proto generated types, vitest with jsdom, Playwright, pnpm.

**Spec:** `docs/superpowers/specs/2026-09-29-animation-system-design.md`, sections 1, 2, 4 and 6.
The reference `simulation/src/robot/animation.py` and its docstrings win where the spec's section 2 wording lags.

This is plan 3 of 3.
Plan 1 (foundation) and plan 2 (firmware, branch `feature/animation-firmware`, unmerged) produced the schema, the fixtures and the robot side this plan talks to.
This branch, `feature/animation-app`, is cut from the firmware branch so the generated protos carry the animation messages.

## Global Constraints

- All code, comments, docs and commit messages in English and ASCII.
- Commit messages are one line, `[Gitmoji] [Verb] ...`, no body and no trailers of any kind.
- App work runs from `app/` with pnpm: `pnpm proto`, `pnpm check`, `pnpm lint`, `pnpm format`, `pnpm test:unit`, `pnpm test:integration`, `pnpm build`.
  Every task ends with `pnpm check` and `pnpm lint` clean.
- Svelte 5 runes (`$state`, `$derived`, `$effect`, `$props`, snippets), no legacy `export let` in new components; stores for cross-component state as the app does today.
- Prettier and ESLint as configured; daisyUI classes for controls; icons from `$lib/components/icons`.
- No dead code; comments only for genuine documentation or a non-obvious why.
- The port mirrors the reference line for line.
  Rules a porter must honour, stated in the reference docstrings: file values are float32 (`Math.fround` every number after `fromJSON`); the evaluate order is body interpolation with the end keyframe's easing, body overlays and per-leg foot-overlay accumulation, `BODY_*` multipliers, then legs, with mixed legs running IK against the output body with the foot endpoint's overlay and lift applied and the base height `baseZ` added to the IK body only; an overlay on a joint leg is ignored; joint clamp symmetric `+-[31.5, 90, 149]` degrees in IK-output space; an unreachable foot sets that leg's femur and tibia mask bits; `REPEAT` is `max(1, floor(x + 0.5))`; entry and exit default to 0.5 s and run on wall time while `SPEED` scales only the playing clock; step arc `45 * min(1, travel / 40) * sin(pi u)` for foot legs whose horizontal travel exceeds 2 mm; `play` records the live pose as the last pose; Exit starts from the final keyframe; Entry converts its destination at the clip's `rideHeight` when set, else the current base; stance feet and the base are inputs.
- Units: body angles radians, lengths mm, joint angles degrees in the IK output convention (the app's `Kinematics.inverseKinematics` returns radians; convert), time seconds.
- Body offset sign conventions, from `platform_shared/animation.proto`: positive body `z` crouches; positive body `x` and `y` move the body toward negative x and y; `+z` lifts a foot; forward is `+y`.
- Wire facts from the firmware: `Message` fields `animationPlay`, `animationStop`, `pose`, `animationStatus`; correlation `fileWriteChunk`, `fileReadChunk`, `fileDelete`, `animationValidate`, `animationListRequest`; responses `empty`, `fileChunk`, `animationReport`, `animationList`; chunks at most 512 bytes; any non-200 chunk means restart the file from offset 0; `animationValidate` answers 422 with the report filled in for an invalid file; `ModesEnum.ANIMATE = 6`; mode messages the app receives are only applied ones; a play from IDLE or DEACTIVATED is ignored by the robot.
- The connected robot may be in use by another session: subagents never connect the dev app to a robot, never flash, never open a serial port.
  Builds and tests only.
  Hardware acceptance is the owner's.
- Markdown: each sentence on its own line.

## Review Focus

1. A JSON file that parses but fails validation (three legs, a NaN, an unknown enum name): the editor shows the error and does not load a broken document.
   Test in Task 3 (validate) and Task 6 (open path).
2. An upload whose chunk gets a non-200 or times out: the uploader restarts from offset 0 once and reports failure after that, never leaving the robot with a half file it believes is whole; validate always follows the last chunk.
   Test in Task 4.
3. Dragging a foot handle to an unreachable spot: the leg turns red and the numeric field still updates; nothing throws.
   Test in Task 5 (handle math) and Task 6 (component).
4. "Show on robot" toggled on while unlinked, or the link dropping mid-drag: no error, no stuck mode request, and the toggle reflects reality.
   Test in Task 4 (pose sender guards) and Task 6.
5. The parity trace at float32: the TypeScript player must match every step; `Math.fround` on file values and a float64 clock are the contract.
   Test in Task 3 across all 819 steps.

---

## File Structure

| Path | Responsibility |
| --- | --- |
| `animations/wave.json`, `spooked.json`, `stretch.json` | sign corrections from the owner's review of plan 1 |
| `app/src/lib/animation/model.ts` | constants, `froundAnimation`, `validate`, `LegTarget` helpers, `Pose` |
| `app/src/lib/animation/evaluator.ts` | `easeValue`, `resolveParams`, `legJointsDeg`, `evaluate`, `poseToAngles`, `capturePose` |
| `app/src/lib/animation/player.ts` | `Player` |
| `app/src/lib/animation/transfer.ts` | upload, download, list, validate, delete over `dataBroker.request` |
| `app/src/lib/animation/library.ts` | built-in animations (JSON import), browser drafts |
| `app/src/lib/control.ts` | `playAnimation`, `stopAnimation`, `PoseSender` |
| `app/src/lib/motion.ts` | `MotionModes.ANIMATE` |
| `app/src/lib/stores/animation.ts` | `animationStatus`, `animationPreview`, editor document store |
| `app/src/lib/kinematic.ts` | `footReachable` |
| `app/src/lib/components/Visualization.svelte`, `sceneBuilder.ts` | preview override, foot handles |
| `app/src/routes/animations/` | `+page.ts`, `+page.svelte`, `Library.svelte`, `Editor.svelte`, `PosePanel.svelte`, `Timeline.svelte`, `AnimationPanel.svelte` |
| `app/src/lib/components/menu/Menu.svelte` | the route in the menu |
| `app/vite.config.ts` | `server.fs.allow` for the root `animations/` directory |
| `app/tests/unit/animation-*.spec.ts` | parity, transfer, editor store, handle math |
| `app/tests/integration/test.ts` | the route renders |
| `docs/animation.md`, `CLAUDE.md` | the app side as built |

---

### Task 1: Library sign corrections

**Files:**
- Modify: `animations/wave.json`, `animations/spooked.json`, `animations/stretch.json`

**Interfaces:**
- Produces: corrected bundled animations; nothing else consumes them differently.

The owner reviewed plan 1's library in the sandbox: wave leaned toward the lifted leg instead of over the supporting five, and stretch did not extend the reaching legs.
The cause is the body offset sign: a positive body offset moves the feet in that direction relative to the body, so the body moves the opposite way (confirmed against the IK on 2026-09-29: body `x = +25` moves the body toward the left side).

- [ ] **Step 1: Apply the corrections**

`animations/wave.json`: on every keyframe that carries `"body": {"x": -25, "y": -15, ...}` change it to `"x": 25, "y": 15` (the body moves left and back, over the five planted feet).
`animations/spooked.json`: change `"y": -20` to `"y": 20` and `"y": -25` to `"y": 25` (the hop goes backward).
`animations/stretch.json`: change the front feet's `"y": 45` to `"y": 85` and the rear feet's `"y": -45` to `"y": -85` (measured reach limit 90 mm at the keyframe's body offset; 85 keeps a margin).

- [ ] **Step 2: Verify**

From `simulation/`: `uv run pytest test_animation_library.py -q` (every keyframe inside joint travel at the default parameters and at the declared extremes; if stretch clamps at `BODY_PITCH` max 1.5, lower that maximum to 1.2 in the file rather than the reach) and `uv run python check_animation.py` (`7/7 passed`).
Then `uv run python sim_sandbox.py`, Animate mode, play `wave`: the body visibly moves away from the raised right front leg; play `stretch`: the front legs reach far forward; play `spooked`: the hop goes backward.

- [ ] **Step 3: Commit**

```bash
git add animations/wave.json animations/spooked.json animations/stretch.json
git commit -m "🐛 Leans wave and spooked over the support polygon and extends the stretch"
```

---

### Task 2: Data model, float32 rounding and validation

**Files:**
- Create: `app/src/lib/animation/model.ts`
- Test: `app/tests/unit/animation-model.spec.ts`

**Interfaces:**
- Produces: constants `KEYFRAME_MAX 32`, `OVERLAY_MAX 8`, `PARAM_MAX 10`, `NAME_LEN_MAX 32`, `DESCRIPTION_LEN_MAX 96`, `SCHEMA_VERSION 1`, `DEFAULT_ENTRY_S 0.5`, `DEFAULT_EXIT_S 0.5`, `STEP_ARC_MM 45`, `STEP_ARC_FULL_TRAVEL_MM 40`, `STEP_ARC_MIN_TRAVEL_MM 2`, `JOINT_LIMIT_DEG [31.5, 90, 149]`, `BODY_PARAM_FOR_AXIS`, `BodyAxis` enum `{ROLL, PITCH, YAW, X, Y, Z}`; type `Leg = { joints: false; v: [number, number, number] } | { joints: true; v: [number, number, number] }`; `legOf(target: LegTarget): Leg`, `stanceLeg(): Leg`; `Pose { body: number[6]; legs: Leg[6] }`, `stancePose()`, `clonePose(p)`; `duration(a)`, `entrySeconds(a)`, `exitSeconds(a)`, `legTarget(k: Keyframe, leg): Leg`; `froundAnimation(a: Animation): Animation`; `validate(a: Animation): string | null`; `loadAnimationJson(text: string): { animation: Animation } | { error: string }` (parse, `Animation.fromJSON`, `froundAnimation`, `validate`).

- [ ] **Step 1: Write the failing tests**

`app/tests/unit/animation-model.spec.ts`:

```ts
import { describe, expect, it } from 'vitest'
import { Animation, Ease, ParamId } from '$lib/platform_shared/animation'
import {
  froundAnimation,
  legOf,
  legTarget,
  loadAnimationJson,
  stanceLeg,
  validate
} from '$lib/animation/model'

const two = (extra: Partial<Animation> = {}): Animation =>
  Animation.fromPartial({
    name: 't',
    schema: 1,
    keyframes: [{ time: 0 }, { time: 1 }],
    ...extra
  })

describe('validate', () => {
  it('accepts a minimal animation', () => {
    expect(validate(two())).toBeNull()
  })
  it.each<[string, (a: Animation) => void, string]>([
    ['schema', a => void (a.schema = 2), 'schema'],
    ['name charset', a => void (a.name = 'Bad Name'), 'name'],
    ['name length', a => void (a.name = 'x'.repeat(33)), 'name'],
    ['description bytes', a => void (a.description = 'e'.repeat(97)), 'description'],
    ['no keyframes', a => void (a.keyframes = []), 'keyframe'],
    ['first time', a => void (a.keyframes[0].time = 0.1), 'time 0'],
    ['increasing', a => void (a.keyframes[1].time = 0), 'increase'],
    ['legs count', a => void (a.keyframes[1].legs = [{}, {}, {}]), '0 or 6'],
    ['ease range', a => void (a.keyframes[1].ease = 4 as Ease), 'ease'],
    ['nan time', a => void (a.keyframes[1].time = NaN), 'finite'],
    ['loop and hold', a => void ((a.loop = true), (a.holdEnd = true)), 'loop'],
    ['overlay channel', a => a.overlays.push({ amplitude: 1, frequency: 1, phase: 0, start: 0, end: 1 }), 'channel'],
    ['overlay body axis', a => a.overlays.push({ bodyAxis: 6, amplitude: 1, frequency: 1, phase: 0, start: 0, end: 1 }), 'body_axis'],
    ['overlay window', a => a.overlays.push({ bodyAxis: 0, amplitude: 1, frequency: 1, phase: 0, start: 0.5, end: 0.5 }), 'start'],
    ['overlay end', a => a.overlays.push({ bodyAxis: 0, amplitude: 1, frequency: 1, phase: 0, start: 0, end: 1.5 }), 'end'],
    ['param unique', a => a.params.push({ id: ParamId.SPEED, min: 0.5, defaultValue: 1, max: 2 }, { id: ParamId.SPEED, min: 0.5, defaultValue: 1, max: 2 }), 'unique'],
    ['param order', a => a.params.push({ id: ParamId.BODY_Z, min: 0.5, defaultValue: 3, max: 2 }), 'min <= default_value <= max'],
    ['speed min', a => a.params.push({ id: ParamId.SPEED, min: 0, defaultValue: 1, max: 2 }), 'SPEED'],
    ['repeat min', a => a.params.push({ id: ParamId.REPEAT, min: 0, defaultValue: 1, max: 2 }), 'REPEAT'],
    ['param id', a => a.params.push({ id: 10 as ParamId, min: 0.5, defaultValue: 1, max: 2 }), 'param id'],
    ['ride height', a => void (a.rideHeight = Infinity), 'ride_height']
  ])('reports %s', (_, mutate, fragment) => {
    const a = two()
    mutate(a)
    expect(validate(a)).toContain(fragment)
  })
  it('bounds the counts', () => {
    expect(validate(two({ keyframes: Array.from({ length: 33 }, (_, i) => ({ time: i })) }))).toContain('32')
    expect(validate(two({ overlays: Array(9).fill({ bodyAxis: 0, amplitude: 1, frequency: 1, phase: 0, start: 0, end: 1 }) }))).toContain('8')
    expect(validate(two({ params: Array.from({ length: 11 }, (_, i) => ({ id: i % 10, min: 1, defaultValue: 1, max: 1 })) }))).toContain('10')
  })
})

describe('legs and rounding', () => {
  it('reads a leg target as foot or joints and an empty target as stance', () => {
    expect(legOf({ joints: { coxa: 1, femur: 2, tibia: 3 } })).toEqual({ joints: true, v: [1, 2, 3] })
    expect(legOf({ foot: { x: 0, y: 0, z: 5 } })).toEqual({ joints: false, v: [0, 0, 5] })
    expect(legOf({})).toEqual(stanceLeg())
    expect(legTarget({ time: 0, ease: Ease.LINEAR, body: undefined, legs: [] }, 4)).toEqual(stanceLeg())
  })
  it('rounds every number to float32', () => {
    const a = froundAnimation(two({ entryTime: 0.4, keyframes: [{ time: 0 }, { time: 0.53, body: { roll: 0.1, pitch: 0, yaw: 0, x: 0, y: 0, z: 15 } }] }))
    expect(a.entryTime).toBe(Math.fround(0.4))
    expect(a.keyframes[1].time).toBe(Math.fround(0.53))
    expect(a.keyframes[1].body?.roll).toBe(Math.fround(0.1))
  })
  it('loads JSON and reports a validation error instead of a document', () => {
    const ok = loadAnimationJson('{"name":"x","schema":1,"keyframes":[{"time":0}]}')
    expect('animation' in ok && ok.animation.name).toBe('x')
    const bad = loadAnimationJson('{"name":"x","schema":1,"keyframes":[{"time":0,"legs":[{},{},{}]}]}')
    expect('error' in bad && bad.error).toContain('0 or 6')
    const junk = loadAnimationJson('{not json')
    expect('error' in junk && junk.error).toMatch(/JSON/)
  })
})
```

- [ ] **Step 2: Run to verify failure**

From `app/`: `pnpm test:unit -- tests/unit/animation-model.spec.ts`
Expected: module not found.

- [ ] **Step 3: Write the model**

`app/src/lib/animation/model.ts`:

```ts
// Port of the constants, validator and pose types in simulation/src/robot/animation.py. The
// generated Animation type is the document; this file adds what the codec does not carry.
import { Animation, Ease, ParamId, type Keyframe, type LegTarget } from '$lib/platform_shared/animation'

export const KEYFRAME_MAX = 32
export const OVERLAY_MAX = 8
export const PARAM_MAX = 10
export const NAME_LEN_MAX = 32
export const DESCRIPTION_LEN_MAX = 96
export const SCHEMA_VERSION = 1
export const DEFAULT_ENTRY_S = 0.5
export const DEFAULT_EXIT_S = 0.5
export const STEP_ARC_MM = 45
export const STEP_ARC_FULL_TRAVEL_MM = 40
export const STEP_ARC_MIN_TRAVEL_MM = 2
export const JOINT_LIMIT_DEG = [31.5, 90, 149] as const
export const PARAM_COUNT = 10

export enum BodyAxis { ROLL = 0, PITCH = 1, YAW = 2, X = 3, Y = 4, Z = 5 }
export const BODY_PARAM_FOR_AXIS = [ParamId.BODY_ROLL, ParamId.BODY_PITCH, ParamId.BODY_YAW, ParamId.BODY_X, ParamId.BODY_Y, ParamId.BODY_Z] as const

export type Vec3 = [number, number, number]
export type Leg = { joints: boolean; v: Vec3 }
export interface Pose { body: number[]; legs: Leg[] }

export const stanceLeg = (): Leg => ({ joints: false, v: [0, 0, 0] })
export const stancePose = (): Pose => ({ body: [0, 0, 0, 0, 0, 0], legs: Array.from({ length: 6 }, stanceLeg) })
export const clonePose = (p: Pose): Pose => ({ body: [...p.body], legs: p.legs.map(l => ({ joints: l.joints, v: [...l.v] as Vec3 })) })

export const legOf = (t: LegTarget): Leg =>
  t.joints ? { joints: true, v: [t.joints.coxa, t.joints.femur, t.joints.tibia] }
           : { joints: false, v: [t.foot?.x ?? 0, t.foot?.y ?? 0, t.foot?.z ?? 0] }
export const legTarget = (k: Keyframe, leg: number): Leg => (k.legs.length ? legOf(k.legs[leg]) : stanceLeg())
export const bodyOf = (k: Keyframe): number[] => [k.body?.roll ?? 0, k.body?.pitch ?? 0, k.body?.yaw ?? 0, k.body?.x ?? 0, k.body?.y ?? 0, k.body?.z ?? 0]

export const duration = (a: Animation) => (a.keyframes.length ? a.keyframes[a.keyframes.length - 1].time : 0)
export const entrySeconds = (a: Animation) => (a.entryTime > 0 ? a.entryTime : DEFAULT_ENTRY_S)
export const exitSeconds = (a: Animation) => (a.exitTime > 0 ? a.exitTime : DEFAULT_EXIT_S)

// File values are float32; a document parsed from JSON must be rounded before it is evaluated,
// or the app diverges from the firmware and the fixtures.
export const froundAnimation = (a: Animation): Animation => {
  const f = Math.fround
  return {
    ...a,
    entryTime: f(a.entryTime), exitTime: f(a.exitTime),
    rideHeight: a.rideHeight === undefined ? undefined : f(a.rideHeight),
    keyframes: a.keyframes.map(k => ({
      ...k, time: f(k.time),
      body: k.body && { roll: f(k.body.roll), pitch: f(k.body.pitch), yaw: f(k.body.yaw), x: f(k.body.x), y: f(k.body.y), z: f(k.body.z) },
      legs: k.legs.map(l => l.joints ? { joints: { coxa: f(l.joints.coxa), femur: f(l.joints.femur), tibia: f(l.joints.tibia) } }
                                     : { foot: { x: f(l.foot?.x ?? 0), y: f(l.foot?.y ?? 0), z: f(l.foot?.z ?? 0) } })
    })),
    overlays: a.overlays.map(o => ({ ...o, amplitude: f(o.amplitude), frequency: f(o.frequency), phase: f(o.phase), start: f(o.start), end: f(o.end) })),
    params: a.params.map(p => ({ ...p, min: f(p.min), defaultValue: f(p.defaultValue), max: f(p.max) }))
  }
}

const NAME_RE = /^[a-z0-9_-]{1,32}$/
const finite = Number.isFinite

// Same rules, order and wording as the reference validate(); the first failure is returned.
export const validate = (a: Animation): string | null => {
  if (a.schema !== SCHEMA_VERSION) return `schema ${a.schema} is not ${SCHEMA_VERSION}`
  if (!NAME_RE.test(a.name)) return 'name must be 1-32 characters of [a-z0-9_-]'
  if (new TextEncoder().encode(a.description).length > DESCRIPTION_LEN_MAX) return 'description longer than 96 bytes'
  if (a.loop && a.holdEnd) return 'loop and hold_end cannot both be set'
  if (a.keyframes.length < 1) return 'at least one keyframe is required'
  if (a.keyframes.length > KEYFRAME_MAX) return `more than ${KEYFRAME_MAX} keyframes`
  if (a.overlays.length > OVERLAY_MAX) return `more than ${OVERLAY_MAX} overlays`
  if (a.params.length > PARAM_MAX) return `more than ${PARAM_MAX} params`
  for (const [i, k] of a.keyframes.entries()) {
    if (k.legs.length !== 0 && k.legs.length !== 6) return `keyframe ${i} must have 0 or 6 legs`
    if (k.ease < Ease.LINEAR || k.ease > Ease.EASE_IN_OUT) return `keyframe ${i} ease out of range`
  }
  const nonFinite = nonFiniteField(a)
  if (nonFinite) return nonFinite
  if (a.keyframes[0].time !== 0) return 'first keyframe must be at time 0'
  for (let i = 1; i < a.keyframes.length; i++)
    if (a.keyframes[i].time <= a.keyframes[i - 1].time) return `keyframe ${i} time must increase`
  for (const [i, o] of a.overlays.entries()) {
    if ((o.bodyAxis === undefined) === (o.footChannel === undefined)) return `overlay ${i} needs exactly one channel`
    if (o.bodyAxis !== undefined && (o.bodyAxis < 0 || o.bodyAxis > 5)) return `overlay ${i} body_axis out of range`
    if (o.footChannel !== undefined && (o.footChannel < 0 || o.footChannel > 17)) return `overlay ${i} foot_channel out of range`
    if (o.start < 0 || o.start >= o.end) return `overlay ${i} window must have 0 <= start < end`
    if (o.end > duration(a)) return `overlay ${i} end is after the last keyframe`
  }
  const seen = new Set<number>()
  for (const p of a.params) {
    if (p.id < 0 || p.id > 9) return `param id ${p.id} out of range`
    if (seen.has(p.id)) return `param ${ParamId[p.id]} is not unique`
    seen.add(p.id)
    if (!(p.min <= p.defaultValue && p.defaultValue <= p.max)) return `param ${ParamId[p.id]} needs min <= default_value <= max`
    if (p.id === ParamId.SPEED && p.min <= 0) return 'param SPEED needs a positive min'
    if (p.id === ParamId.REPEAT && p.min < 1) return 'param REPEAT needs min >= 1'
  }
  return null
}

const nonFiniteField = (a: Animation): string | null => {
  if (!finite(a.entryTime) || !finite(a.exitTime)) return 'entry_time and exit_time must be finite'
  if (a.rideHeight !== undefined && !finite(a.rideHeight)) return 'ride_height must be finite'
  for (const [i, k] of a.keyframes.entries()) {
    if (!finite(k.time)) return `keyframe ${i} time must be finite`
    if (!bodyOf(k).every(finite)) return `keyframe ${i} body must be finite`
    if (!k.legs.every(l => legOf(l).v.every(finite))) return `keyframe ${i} legs must be finite`
  }
  for (const [i, o] of a.overlays.entries())
    if (![o.amplitude, o.frequency, o.phase, o.start, o.end].every(finite)) return `overlay ${i} must be finite`
  for (const p of a.params)
    if (![p.min, p.defaultValue, p.max].every(finite)) return `param ${p.id} must be finite`
  return null
}

export const loadAnimationJson = (text: string): { animation: Animation } | { error: string } => {
  let parsed: unknown
  try { parsed = JSON.parse(text) } catch (e) { return { error: `not JSON: ${(e as Error).message}` } }
  let animation: Animation
  try { animation = froundAnimation(Animation.fromJSON(parsed)) } catch (e) { return { error: `not an animation: ${(e as Error).message}` } }
  const error = validate(animation)
  return error ? { error } : { animation }
}
```

Before running, open `simulation/src/robot/animation.py` `validate` and `_non_finite` and align the rule order and messages exactly (the reference checks counts, then per-keyframe legs and ease, then finiteness, then times and overlays, then params); the test fragments above are chosen to hold under that order.
Note `Animation.fromJSON` throws on an unknown enum name; the loader turns that into an error string.

- [ ] **Step 4: Run, lint, commit**

`pnpm test:unit -- tests/unit/animation-model.spec.ts` all pass; `pnpm check`; `pnpm lint` (run `pnpm format` first).

```bash
git add app/src/lib/animation/model.ts app/tests/unit/animation-model.spec.ts
git commit -m "✨ Adds the animation document model, float32 rounding and validator to the app"
```

---

### Task 3: Evaluator and player port with fixture parity

**Files:**
- Modify: `app/src/lib/kinematic.ts` (`footReachable`)
- Create: `app/src/lib/animation/evaluator.ts`, `app/src/lib/animation/player.ts`
- Test: `app/tests/unit/animation-parity.spec.ts`, `app/tests/unit/animation-player.spec.ts`

**Interfaces:**
- Produces: `Kinematics.footReachable(bodyState: body_state_t, leg: number): boolean`; in `evaluator.ts`: `DEFAULT_FEET` (the 6x4 stance from `genPosture(60 deg, 75 deg)` as used by `Motion`, but taken from the same numbers the firmware uses: `[[122,152,-66,1],[171,0,-66,1],[122,-152,-66,1],[-122,152,-66,1],[-171,0,-66,1],[-122,-152,-66,1]]`), `easeValue(kind, t)`, `resolveParams(a, values: Map<ParamId, number> | undefined): number[10]`, `legJointsDeg(kin, body6, foot, leg, stance, baseZ = 0): Vec3`, `evaluate(a, params, t, kin, stance, baseZ = 0): Pose`, `poseToAngles(pose, kin, stance): { angles: number[18]; mask: number }` (degrees, IK order), `capturePose(bodyState, stance): Pose`; in `player.ts`: `enum State { IDLE, ENTRY, PLAYING, HOLD, EXIT }`, `class Player` with `constructor(kin, stance)`, `play(a, values, live, baseZ = 0)`, `stop(baseZ = 0)`, `update(dt, baseZ = 0): Pose`, `state`, `t`, `lastPose`, `clip`, `params`, `blendFraction()`, `entryBase`.

- [ ] **Step 1: Write the failing parity test**

`app/tests/unit/animation-parity.spec.ts`:

```ts
import { readFileSync } from 'node:fs'
import { resolve } from 'node:path'
import { describe, expect, it } from 'vitest'
import { ParamId } from '$lib/platform_shared/animation'
import Kinematics from '$lib/kinematic'
import { config } from '$lib/components/config'
import { loadAnimationJson } from '$lib/animation/model'
import { DEFAULT_FEET, capturePose, evaluate, poseToAngles, resolveParams } from '$lib/animation/evaluator'
import { Player, State } from '$lib/animation/player'

const FIXTURES = resolve(__dirname, '../../../animations/fixtures')
const expected = JSON.parse(readFileSync(resolve(FIXTURES, 'expected.json'), 'utf8'))
const clip = (name: string) => {
  const r = loadAnimationJson(readFileSync(resolve(FIXTURES, `${name}.json`), 'utf8'))
  if ('error' in r) throw new Error(r.error)
  return r.animation
}
const params = (values: Record<string, number>) =>
  new Map(Object.entries(values).map(([k, v]) => [ParamId[k as keyof typeof ParamId], v]))
const kin = new Kinematics(config)
const TOL = 1e-3 // servo resolution is about 0.1 deg; float32 agreement within 1e-3 is parity

describe('parity with the reference fixtures', () => {
  it('matches every evaluate sample', () => {
    let checked = 0
    for (const c of expected.evaluate) {
      const a = clip(c.animation)
      const p = resolveParams(a, params(c.params))
      for (const s of c.samples) {
        const pose = evaluate(a, p, s.t, kin, DEFAULT_FEET)
        const { angles, mask } = poseToAngles(pose, kin, DEFAULT_FEET)
        expect(mask, `${c.animation} t=${s.t}`).toBe(s.mask)
        angles.forEach((v, j) => expect(v, `${c.animation} t=${s.t} joint ${j}`).toBeCloseTo(s.angles[j], 3))
        checked++
      }
    }
    expect(checked).toBe(59)
  })
  it('matches every player trace step', () => {
    let steps = 0
    for (const [ci, c] of expected.player.entries()) {
      const player = new Player(kin, DEFAULT_FEET)
      const live = {
        omega: c.live.body[0], phi: c.live.body[1], psi: c.live.body[2], xm: c.live.body[3], ym: c.live.body[4], zm: c.live.body[5],
        feet: DEFAULT_FEET.map((f, i) => [f[0] + c.live.feet[i][0], f[1] + c.live.feet[i][1], f[2] + c.live.feet[i][2], 1]),
        cumulative_x: 0, cumulative_y: 0, cumulative_z: 0, cumulative_roll: 0, cumulative_pitch: 0, cumulative_yaw: 0
      }
      player.play(clip(c.animation), params(c.params), capturePose(live, DEFAULT_FEET))
      for (const [step, row] of c.trace.entries()) {
        for (const e of c.events) {
          if (e.step !== step) continue
          if (e.action === 'play') player.play(clip(e.animation), params(e.params), undefined)
          else player.stop()
        }
        const pose = player.update(c.dt)
        const { angles, mask } = poseToAngles(pose, kin, DEFAULT_FEET)
        const where = `case ${ci} step ${step}`
        expect(State[player.state], where).toBe(row.state)
        expect(mask, where).toBe(row.mask)
        angles.forEach((v, j) => expect(v, `${where} joint ${j}`).toBeCloseTo(row.angles[j], 3))
        steps++
      }
    }
    expect(steps).toBe(819)
  })
})
```

`toBeCloseTo(x, 3)` asserts `|a - b| < 0.0005`, which is the intended 1e-3 class; if a row sits between 5e-4 and 1e-3, replace the call with `expect(Math.abs(v - s.angles[j])).toBeLessThan(TOL)` rather than loosening beyond `TOL`.

- [ ] **Step 2: Write the player unit tests**

`app/tests/unit/animation-player.spec.ts`: port `test_entry_blends_from_the_live_pose_and_arcs_a_relocating_foot`, `test_hold_end_freezes_on_the_last_keyframe`, `test_loop_wraps_and_repeat_counts_plays`, `test_zero_dt_and_a_single_keyframe_reach_hold_or_exit`, `test_play_during_exit_enters_from_the_current_blend_without_a_jump`, `test_stop_before_the_first_update_blends_from_the_live_pose` and `test_a_fixed_ride_height_entry_meets_playing_on_the_seam` from `simulation/test_animation.py`, keeping their step counts (`DT = 0.02`, one step of slack past transitions) and tolerances, using `Animation.fromPartial` to build the clips.

- [ ] **Step 3: Run to verify failure**

`pnpm test:unit -- tests/unit/animation-parity.spec.ts tests/unit/animation-player.spec.ts`: module not found.

- [ ] **Step 4: Add reach detection to the kinematics**

In `app/src/lib/kinematic.ts` add to `Kinematics`, mirroring `footReachable` in `firmware/include/kinematics.h` (the transform, the mount offset and rotation, `dx = lx - rootJ1`, `radial = hypot(dx, ly) - j1J2`, `lr = hypot(radial, lz)`, then `|j2J3 - j3Tip| <= lr && lr <= j2J3 + j3Tip`):

```ts
  // Inside the leg's reach annulus, which is exactly where neither acos argument in
  // inverseKinematics saturates. Mirrors Kinematics::footReachable in the firmware.
  footReachable(bodyState: body_state_t, leg: number): boolean {
    const T = get_transformation_matrix(bodyState)
    const [wx, wy, wz] = multiplyVector(T, bodyState.feet[leg]).slice(0, 3).map((v, idx) => v - this.mountPosition[leg][idx])
    const lx = wx * this.ca[leg] + wy * this.sa[leg]
    const ly = wx * this.sa[leg] - wy * this.ca[leg]
    const radial = Math.hypot(lx - this.rootJ1, ly) - this.j1J2
    const lr = Math.hypot(radial, wz)
    return Math.abs(this.j2J3 - this.j3Tip) <= lr && lr <= this.j2J3 + this.j3Tip
  }
```

- [ ] **Step 5: Write the evaluator**

`app/src/lib/animation/evaluator.ts` is a line-for-line port of `evaluate`, `_lifted`, `_resolve_leg`, `_segment`, `leg_joints_deg`, `_body_state`, `resolve_params`, `ease_value`, `pose_to_angles` and `capture_pose` from the reference, with these bindings: the app's `Kinematics.inverseKinematics(bodyState)` returns radians per leg, so `legJointsDeg` and `poseToAngles` multiply by `180 / Math.PI`; `body_state_t` is built by a local `bodyState(body6, stance, baseZ)` that fills `omega, phi, psi, xm, ym, zm = body6[Z] + baseZ`, `feet = stance.map(f => [...f])` and zero `cumulative_*` fields; `M_PI` is `Math.PI`; the mask uses `>>> 0` arithmetic on a number.
Write it with the reference open beside it; every function keeps the reference's name in camelCase and its docstring's rule as a comment only where the rule is not obvious from the code.
`DEFAULT_FEET` is exported from this file as the readonly 6x4 array above.

- [ ] **Step 6: Write the player**

`app/src/lib/animation/player.ts` is a line-for-line port of `Player`, `_blend_targets` and `_blend`, including `blend_fraction()` and `entry_base`, `REPEAT` as `Math.max(1, Math.floor(p[REPEAT] + 0.5))`, `%` replaced by a non-negative `fmod` (`t - Math.floor(t / d) * d`), and the Entry destination base rule (`entryBase = a.rideHeight ?? baseZ`).

- [ ] **Step 7: Run, lint, commit**

`pnpm test:unit -- tests/unit/animation-parity.spec.ts tests/unit/animation-player.spec.ts`: all pass, 59 samples and 819 steps.
A mismatch names the case, step and joint; find the diverging rule against the reference (segment end handling, overlay window inclusivity, the mixed-leg body, reach bits, the `fround` of an input) and fix the port, never the fixture or the tolerance.
`pnpm check`, `pnpm lint`.

```bash
git add app/src/lib/kinematic.ts app/src/lib/animation/evaluator.ts app/src/lib/animation/player.ts app/tests/unit/animation-parity.spec.ts app/tests/unit/animation-player.spec.ts
git commit -m "✨ Ports the animation evaluator and player to the app with fixture parity"
```

---

### Task 4: Transfer, control messages, mode and status

**Files:**
- Create: `app/src/lib/animation/transfer.ts`
- Modify: `app/src/lib/control.ts`, `app/src/lib/motion.ts`, `app/src/routes/+layout.svelte`
- Create: `app/src/lib/stores/animation.ts`
- Test: `app/tests/unit/animation-transfer.spec.ts`

**Interfaces:**
- Produces in `transfer.ts`: `CHUNK = 512`, `animationPath(name) = '/animations/' + name + '.pb'`, `uploadAnimation(a: Animation, onProgress?: (sent, total) => void): Promise<{ ok: true; report: AnimationReport } | { ok: false; error: string }>` (encodes with `Animation.encode(a).finish()`, writes chunks in order, on any non-200 or thrown request restarts once from offset 0, then validates and returns the report; an invalid file is `ok: false` with the report's error), `downloadAnimation(name): Promise<Animation>` (reads chunks until `offset >= totalSize`, decodes, `froundAnimation`, validates, throws on error), `listAnimations(): Promise<AnimationEntry[]>`, `validateAnimation(name): Promise<AnimationReport>`, `deleteAnimation(name): Promise<void>`.
- Produces in `control.ts`: `playAnimation(name: string, values: Map<ParamId, number>)`, `stopAnimation()`, `class PoseSender` with `send(pose: Pose)` throttled to 20 Hz through the app's `throttler` and `cancel()`, sending `PoseData` only when `get(isLinked)`.
- `MotionModes.ANIMATE = 'animate'` appended last (wire index 6).
- Produces in `stores/animation.ts`: `animationStatus: Readable<AnimationStatus | null>` fed by `dataBroker.on(AnimationStatus, ...)` from the layout (like `ModeData`), reset to null on disconnect; `animationPreview: Writable<{ angles: number[]; body: body_state_t } | null>` (angles in radians, IK order) that the 3D view consumes in Task 5.

- [ ] **Step 1: Write the failing tests**

`app/tests/unit/animation-transfer.spec.ts`, using the fake transport pattern from `tests/unit/databroker.spec.ts` (`createFakeTransport()` returning `{ transport, sent, receive }`, `Message.decode` on sent bytes, `receive(Message.create({...}))` to answer):

- an upload of a 1100-byte encoded animation sends chunks at offsets 0, 512 and 1024 with `totalSize` 1100 and the right lengths, each answered with `{ correlationResponse: { correlationId, statusCode: 200, empty: {} } }`, then an `animationValidate` for the name, and resolves `ok: true` with the report;
- a 400 on the second chunk makes the uploader restart at offset 0 (assert the sent sequence 0, 512, 0, 512, 1024) and succeed if the retry passes; a second failure resolves `ok: false` without a validate;
- a validate report with `ok: false` and `error` resolves `ok: false` with that error even though every chunk was 200;
- a download of a 700-byte file requests offsets 0 and 512 with `length` 512, reassembles, decodes and validates;
- `playAnimation('wave', new Map([[ParamId.SPEED, 1.5]]))` emits `animationPlay` with `name: 'wave'` and `params: [{ id: ParamId.SPEED, value: 1.5 }]`; `stopAnimation()` emits `animationStop`;
- `PoseSender.send` twice within 50 ms emits at most one `pose` (fake timers), with `legs` of six `LegTarget`s carrying `foot` or `joints`, and emits nothing when `isLinked` is false;
- `MotionModes` lists `ANIMATE` last and `Object.values(MotionModes).indexOf(MotionModes.ANIMATE) === 6`.

- [ ] **Step 2: Run to verify failure**

`pnpm test:unit -- tests/unit/animation-transfer.spec.ts`: module not found.

- [ ] **Step 3: Implement**

`transfer.ts` builds requests with `dataBroker.request({ fileWriteChunk: { path, offset, totalSize, content } })` and reads `res.statusCode`; the retry rule is one restart from offset 0; every function throws or returns an error string with the failing offset and status.
`control.ts` follows the existing `requestMode` style.
`motion.ts`: append `ANIMATE = 'animate'` to `MotionModes`; in `Motion.step()` the `ANIMATE` case returns `false` (the preview store drives the view instead, Task 5) with a comment saying so.
`+layout.svelte`: subscribe `dataBroker.on(AnimationStatus, s => animationStatus.set(s))` beside the `ModeData` mirror and clear it when `isLinked` turns false.

- [ ] **Step 4: Run, lint, commit**

`pnpm test:unit` (all unit tests), `pnpm check`, `pnpm lint`.

```bash
git add app/src/lib/animation/transfer.ts app/src/lib/control.ts app/src/lib/motion.ts app/src/routes/+layout.svelte app/src/lib/stores/animation.ts app/tests/unit/animation-transfer.spec.ts
git commit -m "✨ Talks animations to the robot: chunked transfer, play, stop, pose and status"
```

---

### Task 5: Preview override and foot handles in the 3D view

**Files:**
- Modify: `app/src/lib/components/Visualization.svelte`, `app/src/lib/sceneBuilder.ts`
- Create: `app/src/lib/animation/handles.ts`
- Test: `app/tests/unit/animation-handles.spec.ts`

**Interfaces:**
- `Visualization.svelte` gains the prop `handles?: { onDrag: (leg: number, offsetMm: Vec3) => void; onDragEnd: () => void } | undefined`; when `$animationPreview` is non-null the render loop applies its `angles` (IK order, radians, through `motion.order`) and its `body` instead of stepping `motion`; when `handles` is set, six small spheres sit at the stance feet plus the current foot offsets and a `TransformControls` (from `three/examples/jsm/controls/TransformControls.js`) in translate mode attaches to the clicked sphere, disabling the orbit while dragging.
- `sceneBuilder.ts` gains `addFootHandles(positions: Vec3[]): Mesh[]`, `attachTransform(target: Object3D | null)`, `setHandlePosition(leg, position)`, and raycast picking on pointerdown over the handle meshes.
- `handles.ts` (pure, tested): `stanceToScene(footMm: Vec3): Vec3` and `sceneToOffset(scenePos: Vec3, stanceMm: Vec3): Vec3`, the mm-to-scene conversion the view uses (`POSITION_SCALE = 1 / 12`, the model's axis convention in `orient_robot`: read it and encode the same mapping), plus `handleColor(clampedMaskForLeg: number): number` returning red for a non-zero mask.

- [ ] **Step 1: Write the failing test**

`app/tests/unit/animation-handles.spec.ts`: `sceneToOffset(stanceToScene([122, 152, -66]), [122, 152, -66])` is `[0, 0, 0]`; moving a scene point by the scene-space equivalent of +10 mm in body x yields offset `[10, 0, 0]` within 1e-9; `+z` up in mm maps to the scene's up axis; `handleColor(0)` is the neutral colour and `handleColor(0x6)` is red.

- [ ] **Step 2: Implement**

Read `orient_robot` and the model load (`populateModelCache`: rotation `x = -PI/2`, `z = PI/2`, scale 10) first and derive the mapping from mm in the body frame to the scene: write it once in `handles.ts` and use it for both the sphere placement and the drag readback, so the two cannot disagree.
In `Visualization.svelte`, keep the existing behaviour byte-for-byte when neither `handles` nor `$animationPreview` is in use; the URDF joints are set from the preview angles through the same `setTargetAngles(motion.order(angles))` path.
`TransformControls` emits `dragging-changed`; set `orbit.enabled = !dragging` and call `onDrag` on `objectChange` with the offset from `sceneToOffset`; `onDragEnd` on `mouseUp`.

- [ ] **Step 3: Run, lint, commit**

`pnpm test:unit`, `pnpm check`, `pnpm lint`; open `pnpm dev` (no robot) and confirm the controller page's view still behaves as before.

```bash
git add app/src/lib/components/Visualization.svelte app/src/lib/sceneBuilder.ts app/src/lib/animation/handles.ts app/tests/unit/animation-handles.spec.ts
git commit -m "✨ Lets the 3D view show a preview pose and drag the feet"
```

---

### Task 6: Library and editor route

**Files:**
- Create: `app/src/lib/animation/library.ts`, `app/src/lib/stores/animation-editor.ts`
- Create: `app/src/routes/animations/+page.ts`, `+page.svelte`, `Library.svelte`, `Editor.svelte`, `PosePanel.svelte`, `Timeline.svelte`, `AnimationPanel.svelte`
- Modify: `app/src/lib/components/menu/Menu.svelte`, `app/vite.config.ts`
- Test: `app/tests/unit/animation-editor.spec.ts`, `app/tests/integration/test.ts`

**Interfaces:**
- `library.ts`: `builtIn: Animation[]` from `import.meta.glob('../../../../animations/*.json', { eager: true, import: 'default' })` passed through `loadAnimationJson` (a failing built-in throws at import so the build catches it); `drafts` as a `persistentStore('animation_drafts', Record<string, unknown>)` of JSON documents keyed by name, wrapped in try/catch, with `saveDraft(a)`, `deleteDraft(name)`, `loadDraft(name)`.
- `animation-editor.ts`: a store `editor` with `document: Animation`, `selected: number` (keyframe index), `scrub: number` (seconds), `playing: boolean`, `values: Map<ParamId, number>`, `showOnRobot: boolean`, `dirty: boolean`; actions `newDocument()`, `open(a)`, `selectKeyframe(i)`, `addKeyframe(afterIndex)`, `duplicateKeyframe(i)`, `deleteKeyframe(i)`, `setKeyframeTime(i, t)` (keeps order and the first at 0), `setEase(i, ease)`, `setBody(i, axis, value)`, `setLeg(i, leg, target: Leg)` (converting through `forwardKinematics` when a leg flips from joints to foot, as the editor convenience), `addOverlay()`, `updateOverlay(i, patch)`, `removeOverlay(i)`, `setMeta(patch)`, `setParam(i, patch)`, `addParam(id)`, `removeParam(i)`, `setScrub(t)`, `setValue(id, v)`; derived `pose` (the evaluator at `scrub` with `values`, or the player's pose while previewing), `angles` and `mask` (through `poseToAngles`), `validation: string | null`.
- The route: `+page.ts` returns `{ title: 'Animations' }`; `+page.svelte` switches between `Library` and `Editor`.

- [ ] **Step 1: Write the failing editor-store test**

`app/tests/unit/animation-editor.spec.ts`: `newDocument()` yields a valid document with one keyframe at 0 and `validation` null; `addKeyframe(0)` inserts at `time + 0.5` and selects it; `setKeyframeTime` rejects a time that breaks order (leaves the document unchanged and sets an `error` message); `setLeg` from foot `[0, 0, 30]` to joints then back to foot round-trips within 1e-3 mm through FK; `setScrub(0.25)` changes `angles` on a document with a crouch keyframe; a document with a three-leg keyframe reports `validation` non-null; `open()` of an invalid document is refused with the error.

- [ ] **Step 2: Build the UI**

`Library.svelte`: three sections (Built-in, On robot, Drafts) as daisyUI cards in a responsive grid; each row has the name, description, the declared parameter sliders (range inputs labelled with the `ParamId` name, min, default, max), Play (`playAnimation`) and Stop, and for robot rows Download-to-editor and Delete; the On-robot list refreshes from `listAnimations()` when linked and shows "not connected" otherwise; the status line at the top shows `$animationStatus` (name, state, t, clamped mask as 18 dots).
`Editor.svelte`: left `Visualization` with `handles` bound to `setLeg` for foot legs; right column `PosePanel`, below it `Timeline`, then `AnimationPanel`; a toolbar with New, Open (file input, `loadAnimationJson`), Open from library or draft or robot (dropdowns), Save JSON (download `JSON.stringify(Animation.toJSON(doc), null, 2)`), Save draft, Upload to robot (`uploadAnimation` then a toast with the report), Play on robot (upload, then `playAnimation` with the current values), and the "Show on robot" toggle.
`PosePanel.svelte`: six body sliders (roll, pitch, yaw in rad with degree labels; x, y, z in mm) for the selected keyframe; per leg a foot/joints toggle with three numeric fields, red text and a named clamped joint when the mask bit is set.
`Timeline.svelte`: an axis from 0 to the duration with keyframe markers (drag horizontally to retime through `setKeyframeTime`, click to select), a scrub head bound to `scrub`, play/pause/loop (a local `Player` stepped by `requestAnimationFrame` with the real elapsed time, its pose written to the editor's preview), the speed slider, the ease select for the selected keyframe, and the overlay list.
`AnimationPanel.svelte`: name, description, loop, hold at end, entry and exit times, ride height (checkbox "fixed" plus a number), the exposed parameters with min, default, max, and the validation message.
"Show on robot": when toggled on and linked, `requestMode(MotionModes.ANIMATE)` and a `PoseSender`; every change to the derived `pose` while not previewing calls `send(pose)`; previewing sends the player's pose too; toggling off or losing the link cancels the sender; the toggle is disabled while unlinked.
Menu: add `{ title: 'Animations', icon: <an existing icon or a new one added to icons/index.ts>, href: withBase('animations'), feature: true }` at top level after Controller.
`vite.config.ts`: `server: { fs: { allow: ['..'] }, proxy: {...} }` so the dev server can serve the root `animations/` JSON that `library.ts` imports.

- [ ] **Step 3: Integration test**

Add to `app/tests/integration/test.ts` a test that `/animations` renders the "Built-in" heading and the seven names without a `pageerror`.

- [ ] **Step 4: Verify, lint, commit**

`pnpm test:unit`, `pnpm check`, `pnpm lint`, `pnpm build`, `pnpm test:integration`.
Manual, with `pnpm dev` and no robot: open `/animations`; the library lists seven built-ins with sliders; open `wave` in the editor; scrub shows the lean and the flick in the 3D view; drag a foot handle; flip a leg to joints and back; save JSON and re-open it; the "Show on robot" toggle is disabled.

```bash
git add app/src/lib/animation/library.ts app/src/lib/stores/animation-editor.ts app/src/routes/animations app/src/lib/components/menu/Menu.svelte app/vite.config.ts app/tests/unit/animation-editor.spec.ts app/tests/integration/test.ts
git commit -m "✨ Adds the animation library and editor pages"
```

---

### Task 7: Docs and the hardware acceptance list

**Files:**
- Modify: `docs/animation.md`, `CLAUDE.md`

- [ ] **Step 1: Docs**

`docs/animation.md`: an "App" section describing the port, the transfer rules the app follows (512-byte chunks, restart from 0, validate after the last chunk), the library and editor, "Show on robot" and "Play on robot", the draft storage as a per-browser convenience, and the float32 rule; update "Known gaps" (the app is no longer a gap); add the app steps to the acceptance list: upload each built-in from the library page and play it; toggle "Show on robot" and drag a foot; play on robot from the editor and watch the status run ENTRY, PLAYING, EXIT, IDLE with the mode returning to STAND; chain two; stop one mid-way.
`CLAUDE.md`: in "Web app architecture" add one sentence naming `app/src/lib/animation/` (the TS mirror of the animation reference, tested against the fixtures) and the `/animations` route.

- [ ] **Step 2: Commit**

```bash
git add docs/animation.md CLAUDE.md
git commit -m "📝 Documents the animation app and its acceptance steps"
```

---

## Self-review notes

- Spec coverage: section 4 library and editor (Task 6), "Show on robot" and "Play on robot" (Tasks 4 and 6), files (Tasks 2, 4, 6), code shape (`evaluator.ts`, `player.ts`, `transfer.ts`, the editor store), section 6 app tests (Tasks 2 to 6) and hardware acceptance (Task 7), plus the owner's library corrections (Task 1).
- The Review Focus items are pinned by the model tests (1), the transfer tests (2 and 4), the handle tests and the editor store test (3), and the parity trace (5).
- Out of scope, as the spec says: multi-animation sequences, sound, controller button mapping.
