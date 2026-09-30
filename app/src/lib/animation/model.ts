// Port of the constants, validator and pose types in simulation/src/robot/animation.py. The
// generated Animation type is the document; this file adds what the codec does not carry.
import {
  Animation,
  Ease,
  ParamId,
  type Keyframe,
  type LegTarget
} from '$lib/platform_shared/animation'

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

export enum BodyAxis {
  ROLL = 0,
  PITCH = 1,
  YAW = 2,
  X = 3,
  Y = 4,
  Z = 5
}
export const BODY_PARAM_FOR_AXIS = [
  ParamId.BODY_ROLL,
  ParamId.BODY_PITCH,
  ParamId.BODY_YAW,
  ParamId.BODY_X,
  ParamId.BODY_Y,
  ParamId.BODY_Z
] as const

export type Vec3 = [number, number, number]
export type Leg = { joints: boolean; v: Vec3 }
export interface Pose {
  body: number[]
  legs: Leg[]
}

export const stanceLeg = (): Leg => ({ joints: false, v: [0, 0, 0] })
export const stancePose = (): Pose => ({
  body: [0, 0, 0, 0, 0, 0],
  legs: Array.from({ length: 6 }, stanceLeg)
})
export const clonePose = (p: Pose): Pose => ({
  body: [...p.body],
  legs: p.legs.map(l => ({ joints: l.joints, v: [...l.v] as Vec3 }))
})

export const legOf = (t: LegTarget): Leg =>
  t.joints ?
    { joints: true, v: [t.joints.coxa, t.joints.femur, t.joints.tibia] }
  : { joints: false, v: [t.foot?.x ?? 0, t.foot?.y ?? 0, t.foot?.z ?? 0] }
export const legTarget = (k: Keyframe, leg: number): Leg =>
  k.legs.length ? legOf(k.legs[leg]) : stanceLeg()
export const bodyOf = (k: Keyframe): number[] => [
  k.body?.roll ?? 0,
  k.body?.pitch ?? 0,
  k.body?.yaw ?? 0,
  k.body?.x ?? 0,
  k.body?.y ?? 0,
  k.body?.z ?? 0
]

export const duration = (a: Animation) =>
  a.keyframes.length ? a.keyframes[a.keyframes.length - 1].time : 0
export const entrySeconds = (a: Animation) => (a.entryTime > 0 ? a.entryTime : DEFAULT_ENTRY_S)
export const exitSeconds = (a: Animation) => (a.exitTime > 0 ? a.exitTime : DEFAULT_EXIT_S)

// File values are float32; a document parsed from JSON must be rounded before it is evaluated,
// or the app diverges from the firmware and the fixtures.
export const froundAnimation = (a: Animation): Animation => {
  const f = Math.fround
  return {
    ...a,
    entryTime: f(a.entryTime),
    exitTime: f(a.exitTime),
    rideHeight: a.rideHeight === undefined ? undefined : f(a.rideHeight),
    keyframes: a.keyframes.map(k => ({
      ...k,
      time: f(k.time),
      body: k.body && {
        roll: f(k.body.roll),
        pitch: f(k.body.pitch),
        yaw: f(k.body.yaw),
        x: f(k.body.x),
        y: f(k.body.y),
        z: f(k.body.z)
      },
      legs: k.legs.map(l =>
        l.joints ?
          { joints: { coxa: f(l.joints.coxa), femur: f(l.joints.femur), tibia: f(l.joints.tibia) } }
        : { foot: { x: f(l.foot?.x ?? 0), y: f(l.foot?.y ?? 0), z: f(l.foot?.z ?? 0) } }
      )
    })),
    overlays: a.overlays.map(o => ({
      ...o,
      amplitude: f(o.amplitude),
      frequency: f(o.frequency),
      phase: f(o.phase),
      start: f(o.start),
      end: f(o.end)
    })),
    params: a.params.map(p => ({
      ...p,
      min: f(p.min),
      defaultValue: f(p.defaultValue),
      max: f(p.max)
    }))
  }
}

const NAME_RE = /^[a-z0-9_-]{1,32}$/
const finite = Number.isFinite

const nonFiniteField = (a: Animation): string | null => {
  if (!finite(a.entryTime) || !finite(a.exitTime)) return 'entry_time and exit_time must be finite'
  if (a.rideHeight !== undefined && !finite(a.rideHeight)) return 'ride_height must be finite'
  for (const [i, k] of a.keyframes.entries()) {
    const values = [k.time, ...bodyOf(k), ...k.legs.flatMap(l => legOf(l).v)]
    if (!values.every(finite)) return `keyframe ${i} has a non-finite value`
  }
  for (const [i, o] of a.overlays.entries())
    if (![o.amplitude, o.frequency, o.phase, o.start, o.end].every(finite))
      return `overlay ${i} has a non-finite value`
  for (const [i, p] of a.params.entries())
    if (![p.min, p.defaultValue, p.max].every(finite)) return `param ${i} has a non-finite value`
  return null
}

// Same rules and order as the reference validate(); the first failure is returned.
export const validate = (a: Animation): string | null => {
  if (a.schema !== SCHEMA_VERSION) return `schema ${a.schema} is not ${SCHEMA_VERSION}`
  if (!NAME_RE.test(a.name)) return `name must be 1-${NAME_LEN_MAX} characters of [a-z0-9_-]`
  if (new TextEncoder().encode(a.description).length > DESCRIPTION_LEN_MAX)
    return `description longer than ${DESCRIPTION_LEN_MAX} bytes`
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
    if ((o.bodyAxis === undefined) === (o.footChannel === undefined))
      return `overlay ${i} needs exactly one channel`
    if (o.bodyAxis !== undefined && (o.bodyAxis < 0 || o.bodyAxis > 5))
      return `overlay ${i} body_axis out of range`
    if (o.footChannel !== undefined && (o.footChannel < 0 || o.footChannel > 17))
      return `overlay ${i} foot_channel out of range`
    if (o.start < 0 || o.start >= o.end) return `overlay ${i} window must have 0 <= start < end`
    if (o.end > duration(a)) return `overlay ${i} end is after the last keyframe`
  }
  const seen = new Set<number>()
  for (const p of a.params) {
    if (p.id < 0 || p.id >= PARAM_COUNT) return `param id ${p.id} out of range`
    if (seen.has(p.id)) return `param ${ParamId[p.id]} is not unique`
    seen.add(p.id)
    if (!(p.min <= p.defaultValue && p.defaultValue <= p.max))
      return `param ${ParamId[p.id]} needs min <= default_value <= max`
    if (p.id === ParamId.SPEED && p.min <= 0) return 'param SPEED needs a positive min'
    if (p.id === ParamId.REPEAT && p.min < 1) return 'param REPEAT needs min >= 1'
  }
  return null
}

export const loadAnimationJson = (text: string): { animation: Animation } | { error: string } => {
  let parsed: unknown
  try {
    parsed = JSON.parse(text)
  } catch (e) {
    return { error: `not JSON: ${(e as Error).message}` }
  }
  let animation: Animation
  try {
    animation = froundAnimation(Animation.fromJSON(parsed))
  } catch (e) {
    return { error: `not an animation: ${(e as Error).message}` }
  }
  const error = validate(animation)
  return error ? { error } : { animation }
}

// The shortest decimal that reads back as the same float32, so a saved file shows 0.08 rather
// than the double 0.07999999821186066.
const shortestFloat32 = (v: number): number => {
  for (let p = 1; p <= 9; p++) {
    const candidate = Number(v.toPrecision(p))
    if (Math.fround(candidate) === v) return candidate
  }
  return v
}

// Canonical proto3 JSON with the two-space indentation and trailing newline of animations/*.json.
export const serializeAnimation = (a: Animation): string =>
  JSON.stringify(
    Animation.toJSON(froundAnimation(a)),
    (_, value) =>
      typeof value === 'number' && Number.isFinite(value) ? shortestFloat32(value) : value,
    2
  ) + '\n'
