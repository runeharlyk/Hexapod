// Port of the evaluator in simulation/src/robot/animation.py, the authority for its behaviour.
import { Ease, ParamId, type Animation, type Keyframe } from '$lib/platform_shared/animation'
import type Kinematics from '$lib/kinematic'
import type { body_state_t } from '$lib/kinematic'
import {
  BODY_PARAM_FOR_AXIS,
  BodyAxis,
  JOINT_LIMIT_DEG,
  PARAM_COUNT,
  bodyOf,
  duration,
  legTarget,
  type Leg,
  type Pose,
  type Vec3
} from './model'

export type Stance = readonly (readonly number[])[]

export const DEFAULT_FEET: Stance = [
  [122, 152, -66, 1],
  [171, 0, -66, 1],
  [122, -152, -66, 1],
  [-122, 152, -66, 1],
  [-171, 0, -66, 1],
  [-122, -152, -66, 1]
]

const DEG = 180 / Math.PI

export const easeValue = (kind: Ease, t: number): number => {
  if (kind === Ease.EASE_IN) return t * t
  if (kind === Ease.EASE_OUT) return t * (2 - t)
  if (kind === Ease.EASE_IN_OUT) return t < 0.5 ? 2 * t * t : -1 + (4 - 2 * t) * t
  return t
}

// Declared ids take the caller's value clamped to the spec, else the default; undeclared ids are
// 1 (the neutral multiplier and a single play).
export const resolveParams = (a: Animation, values: Map<ParamId, number> | undefined): number[] => {
  const out = new Array<number>(PARAM_COUNT).fill(1)
  for (const spec of a.params) {
    const v = values?.get(spec.id) ?? spec.defaultValue
    out[spec.id] = Math.min(Math.max(v, spec.min), spec.max)
  }
  return out
}

const bodyState = (body6: readonly number[], stance: Stance, baseZ = 0): body_state_t => ({
  omega: body6[BodyAxis.ROLL],
  phi: body6[BodyAxis.PITCH],
  psi: body6[BodyAxis.YAW],
  xm: body6[BodyAxis.X],
  ym: body6[BodyAxis.Y],
  zm: body6[BodyAxis.Z] + baseZ,
  feet: stance.map(f => [...f]),
  cumulative_x: 0,
  cumulative_y: 0,
  cumulative_z: 0,
  cumulative_roll: 0,
  cumulative_pitch: 0,
  cumulative_yaw: 0
})

const addFoot = (b: body_state_t, leg: number, foot: Vec3) => {
  for (let axis = 0; axis < 3; axis++) b.feet[leg][axis] += foot[axis]
}

// Solved on the body the runner outputs, the offsets plus the ride-height base on z, so a joint
// leg meets its foot endpoint at any base.
export const legJointsDeg = (
  kin: Kinematics,
  body6: readonly number[],
  foot: Vec3,
  leg: number,
  stance: Stance,
  baseZ = 0
): Vec3 => {
  const b = bodyState(body6, stance, baseZ)
  addFoot(b, leg, foot)
  return kin.inverseKinematics(b)[leg].map(v => v * DEG) as Vec3
}

// The keyframe pair bracketing t and the eased fraction between them, with t clamped.
const segment = (a: Animation, t: number): [Keyframe, Keyframe, number] => {
  const kfs = a.keyframes
  if (t <= 0 || kfs.length === 1) return [kfs[0], kfs[0], 0]
  const end = kfs[kfs.length - 1]
  if (t >= end.time) return [end, end, 0]
  let i = 1
  while (kfs[i].time < t) i++
  const k0 = kfs[i - 1]
  const k1 = kfs[i]
  return [k0, k1, easeValue(k1.ease, (t - k0.time) / (k1.time - k0.time))]
}

const lerp3 = (a: Vec3, b: Vec3, u: number): Vec3 => [
  a[0] + (b[0] - a[0]) * u,
  a[1] + (b[1] - a[1]) * u,
  a[2] + (b[2] - a[2]) * u
]

const lifted = (foot: Vec3, overlay: Vec3, lift: number): Vec3 => [
  foot[0] + overlay[0],
  foot[1] + overlay[1],
  (foot[2] + overlay[2]) * lift
]

const resolveLeg = (
  kin: Kinematics,
  a: Leg,
  b: Leg,
  overlay: Vec3,
  lift: number,
  body: number[],
  leg: number,
  u: number,
  stance: Stance,
  baseZ: number
): Leg => {
  if (!a.joints && !b.joints) return { joints: false, v: lifted(lerp3(a.v, b.v, u), overlay, lift) }
  const ja =
    a.joints ? a.v : legJointsDeg(kin, body, lifted(a.v, overlay, lift), leg, stance, baseZ)
  const jb =
    b.joints ? b.v : legJointsDeg(kin, body, lifted(b.v, overlay, lift), leg, stance, baseZ)
  return { joints: true, v: lerp3(ja, jb, u) }
}

// The pose at t clamped to [0, duration]: the interpolated body plus body overlays, times the
// BODY_* params, is the output body; foot overlays and FOOT_LIFT apply to foot endpoints only.
// A mixed leg lerps in joint space with its foot endpoint solved on the output body and the lifted
// foot, so it is continuous across a keyframe under any multiplier or overlay. baseZ enters only
// that IK; the returned body stays an offset.
export const evaluate = (
  a: Animation,
  params: readonly number[],
  t: number,
  kin: Kinematics,
  stance: Stance,
  baseZ = 0
): Pose => {
  const [k0, k1, u] = segment(a, t)
  t = Math.min(Math.max(t, 0), duration(a))
  const b0 = bodyOf(k0)
  const b1 = bodyOf(k1)
  const body = b0.map((v, i) => v + (b1[i] - v) * u)
  const footOverlay: Vec3[] = Array.from({ length: 6 }, () => [0, 0, 0])
  for (const o of a.overlays) {
    if (!(o.start <= t && t <= o.end)) continue
    const v =
      o.amplitude *
      params[ParamId.OVERLAY_AMPLITUDE] *
      Math.sin(2 * Math.PI * o.frequency * t + o.phase)
    if (o.bodyAxis !== undefined) body[o.bodyAxis] += v
    else if (o.footChannel !== undefined)
      footOverlay[Math.floor(o.footChannel / 3)][o.footChannel % 3] += v
  }
  BODY_PARAM_FOR_AXIS.forEach((pid, axis) => (body[axis] *= params[pid]))
  const lift = params[ParamId.FOOT_LIFT]
  const legs = Array.from({ length: 6 }, (_, i) =>
    resolveLeg(
      kin,
      legTarget(k0, i),
      legTarget(k1, i),
      footOverlay[i],
      lift,
      body,
      i,
      u,
      stance,
      baseZ
    )
  )
  return { body, legs }
}

// 18 servo angles (deg, IK order) and an 18-bit mask, bit leg * 3 + joint, of the joints that hit
// a limit. A foot leg that fails footReachable also sets its femur and tibia bits, the joints the
// IK saturates.
export const poseToAngles = (
  pose: Pose,
  kin: Kinematics,
  stance: Stance
): { angles: number[]; mask: number } => {
  const b = bodyState(pose.body, stance)
  pose.legs.forEach((leg, i) => {
    if (!leg.joints) addFoot(b, i, leg.v)
  })
  const raw = kin
    .inverseKinematics(b)
    .flatMap((joints, i) => (pose.legs[i].joints ? pose.legs[i].v : joints.map(v => v * DEG)))
  let mask = 0
  const angles = raw.map((v, j) => {
    const limit = JOINT_LIMIT_DEG[j % 3]
    const clamped = Math.min(Math.max(v, -limit), limit)
    if (clamped !== v) mask = (mask | (1 << j)) >>> 0
    return clamped
  })
  pose.legs.forEach((leg, i) => {
    if (!leg.joints && !kin.footReachable(b, i)) mask = (mask | (0b110 << (i * 3))) >>> 0
  })
  return { angles, mask }
}

export const capturePose = (body: body_state_t, stance: Stance): Pose => ({
  body: [body.omega, body.phi, body.psi, body.xm, body.ym, body.zm],
  legs: stance.map((f, i) => ({
    joints: false,
    v: [body.feet[i][0] - f[0], body.feet[i][1] - f[1], body.feet[i][2] - f[2]] as Vec3
  }))
})
