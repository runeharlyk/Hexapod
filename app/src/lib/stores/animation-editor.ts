import { derived, writable, type Readable } from 'svelte/store'
import {
  Animation,
  Ease,
  ParamId,
  type Keyframe,
  type LegTarget,
  type Overlay,
  type ParamSpec
} from '$lib/platform_shared/animation'
import Kinematics, { type body_state_t } from '$lib/kinematic'
import { config } from '$lib/components/config'
import { outControllerData } from '$lib/stores'
import {
  BodyAxis,
  SCHEMA_VERSION,
  bodyOf,
  duration,
  froundAnimation,
  legTarget,
  stancePose,
  validate,
  type Leg,
  type Pose,
  type Vec3
} from '$lib/animation/model'
import {
  DEFAULT_FEET,
  evaluate,
  legJointsDeg,
  poseToAngles,
  resolveParams,
  type Stance
} from '$lib/animation/evaluator'
import type { AnimationPreview } from './animation'

export const kinematics = new Kinematics(config)

export const LEG_NAMES = ['RF', 'RM', 'RR', 'LF', 'LM', 'LR'] as const
export const JOINT_NAMES = ['coxa', 'femur', 'tibia'] as const

// Documents hold float32 values and FK leaves 1e-15 residues; four decimals print them as typed.
export const shown = (v: number) => Number(v.toFixed(4)) || 0

export const clampedJoints = (mask: number): string[] =>
  Array.from({ length: 18 }, (_, j) => j)
    .filter(j => (mask >>> j) & 1)
    .map(j => `${LEG_NAMES[Math.floor(j / 3)]} ${JOINT_NAMES[j % 3]}`)

// Mirrors MotionService::updateFeetDistanceTarget: the controller's feet distance scales the
// standing feet on x and y.
export const stanceFor = (feetDistance: number): Stance => {
  const clamped = Math.min(Math.max(feetDistance, -1), 1)
  const scale = 0.75 + ((clamped + 1) / 2) * 0.5
  return DEFAULT_FEET.map(([x, y, z, w]) => [x * scale, y * scale, z, w])
}

// The firmware's ANIMATE base is the height slider (MotionService::handleCommand, zm = h * 50)
// unless the file fixes a ride height.
export const rideBase = (a: Animation, height: number) => a.rideHeight ?? height * 50

export interface EditorState {
  document: Animation
  selected: number
  scrub: number
  playing: boolean
  values: Map<ParamId, number>
  showOnRobot: boolean
  dirty: boolean
  error: string | null
}

export interface Frame {
  pose: Pose
  angles: number[]
  mask: number
  preview: AnimationPreview
}

const PARAM_RANGE: Partial<Record<ParamId, [number, number, number]>> = {
  [ParamId.SPEED]: [0.5, 1, 2],
  [ParamId.REPEAT]: [1, 1, 3]
}
const MULTIPLIER_RANGE: [number, number, number] = [0, 1, 1.5]
const KEYFRAME_STEP_S = 0.5
const RAD = Math.PI / 180

const blankKeyframe = (time: number): Keyframe => ({
  time,
  ease: Ease.LINEAR,
  body: undefined,
  legs: []
})

export const blankAnimation = (): Animation =>
  Animation.fromPartial({ name: 'untitled', schema: SCHEMA_VERSION, keyframes: [blankKeyframe(0)] })

const copyKeyframe = (k: Keyframe, time: number): Keyframe => ({
  ...structuredClone(k),
  time
})

const targetOf = (leg: Leg): LegTarget =>
  leg.joints ?
    { joints: { coxa: leg.v[0], femur: leg.v[1], tibia: leg.v[2] } }
  : { foot: { x: leg.v[0], y: leg.v[1], z: leg.v[2] } }

const bodyState = (body6: readonly number[], stance: Stance, baseZ: number): body_state_t => ({
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

type LegRad = [number, number, number]

const feetFromJoints = (body: body_state_t, joints: LegRad[]) =>
  kinematics.forwardKinematics(body, joints)

// FK of one leg's joint angles (deg) on a body, as an offset from that leg's standing foot.
const footFromJoints = (
  body6: readonly number[],
  joints: Vec3,
  leg: number,
  stance: Stance,
  baseZ: number
): Vec3 => {
  const rad = joints.map(v => v * RAD) as LegRad
  const foot = feetFromJoints(
    bodyState(body6, stance, baseZ),
    Array.from({ length: 6 }, () => rad)
  )[leg]
  return [foot[0] - stance[leg][0], foot[1] - stance[leg][1], foot[2] - stance[leg][2]]
}

// The pose goes through IK at the ride-height base, as the firmware runner does. A joint leg's
// foot is drawn where FK of its clamped angles puts it.
const frameOf = (pose: Pose, stance: Stance, baseZ: number): Frame => {
  const body = [...pose.body]
  body[BodyAxis.Z] += baseZ
  const { angles, mask } = poseToAngles({ body, legs: pose.legs }, kinematics, stance)
  const state = bodyState(body, stance, 0)
  const rad = angles.map(v => v * RAD)
  const fk = feetFromJoints(
    state,
    Array.from({ length: 6 }, (_, i) => rad.slice(i * 3, i * 3 + 3) as LegRad)
  )
  pose.legs.forEach((leg, i) => {
    state.feet[i] = leg.joints ? [...fk[i]] : state.feet[i].map((v, axis) => v + (leg.v[axis] ?? 0))
  })
  return { pose, angles, mask, preview: { angles: rad, body: state, mask } }
}

// A keyframe without 0 or 6 legs would crash the evaluator; the other structural errors do not.
const evaluable = (a: Animation) =>
  a.keyframes.length > 0 && a.keyframes.every(k => k.legs.length === 0 || k.legs.length === 6)

export const createEditor = (initial: Animation = blankAnimation()) => {
  const state = writable<EditorState>({
    document: initial,
    selected: 0,
    scrub: 0,
    playing: false,
    values: new Map(),
    showOnRobot: false,
    dirty: false,
    error: null
  })
  const playback = writable<Pose | null>(null)
  let current!: EditorState
  state.subscribe(s => (current = s))
  let controller = [0, 0, 0, 0, 0, 0, 0, 0]
  outControllerData.subscribe(d => (controller = d))

  const fail = (error: string) => {
    state.update(s => ({ ...s, error }))
    return error
  }

  // Applies a change to a copy of the document; a returned string refuses it unchanged.
  const edit = (change: (doc: Animation) => string | void, select?: number): string | null => {
    const doc = structuredClone(current.document)
    const error = change(doc)
    if (error) return fail(error)
    state.update(s => ({
      ...s,
      document: doc,
      dirty: true,
      error: null,
      selected: Math.max(0, Math.min(select ?? s.selected, doc.keyframes.length - 1)),
      scrub: Math.min(s.scrub, duration(doc))
    }))
    return null
  }

  const insertAfter = (doc: Animation, i: number) => {
    const t = doc.keyframes[i].time
    const next = doc.keyframes[i + 1]
    const time =
      next && next.time <= t + KEYFRAME_STEP_S ? (t + next.time) / 2 : t + KEYFRAME_STEP_S
    doc.keyframes.splice(i + 1, 0, copyKeyframe(doc.keyframes[i], time))
  }

  const reset = (document: Animation) =>
    state.update(s => ({
      ...s,
      document,
      selected: 0,
      scrub: 0,
      values: new Map(),
      dirty: false,
      error: null
    }))

  // Edits hold doubles; the preview and the mask are computed on the float32 values the robot
  // stores.
  const rounded = derived(state, s => froundAnimation(s.document))
  const validation = derived(rounded, validate)

  const frame: Readable<Frame> = derived(
    [state, rounded, playback, outControllerData],
    ([s, doc, playing, ctrl]) => {
      const stance = stanceFor(ctrl[7])
      const baseZ = rideBase(doc, ctrl[4])
      if (playing) return frameOf(playing, stance, baseZ)
      if (!evaluable(doc)) return frameOf(stancePose(), stance, baseZ)
      const params = resolveParams(doc, s.values)
      return frameOf(evaluate(doc, params, s.scrub, kinematics, stance, baseZ), stance, baseZ)
    }
  )

  return {
    subscribe: state.subscribe,
    validation,
    frame,
    pose: derived(frame, f => f.pose),
    angles: derived(frame, f => f.angles),
    mask: derived(frame, f => f.mask),

    newDocument: () => reset(blankAnimation()),

    open: (a: Animation): string | null => {
      const error = validate(froundAnimation(a))
      if (error) return fail(error)
      reset(structuredClone(a))
      return null
    },

    markSaved: () => state.update(s => ({ ...s, dirty: false })),

    selectKeyframe: (i: number) =>
      state.update(s => ({ ...s, selected: i, scrub: s.document.keyframes[i]?.time ?? s.scrub })),

    addKeyframe: (afterIndex: number) => edit(doc => insertAfter(doc, afterIndex), afterIndex + 1),

    duplicateKeyframe: (i: number) => edit(doc => insertAfter(doc, i), i + 1),

    deleteKeyframe: (i: number) =>
      edit(doc => {
        if (i === 0) return 'the first keyframe cannot be deleted'
        doc.keyframes.splice(i, 1)
      }, i - 1),

    // Keeps the first keyframe at 0 and the times strictly increasing.
    setKeyframeTime: (i: number, t: number) =>
      edit(doc => {
        if (!Number.isFinite(t)) return 'keyframe time must be a number'
        if (i === 0) return t === 0 ? undefined : 'the first keyframe stays at 0'
        const prev = doc.keyframes[i - 1].time
        const next = doc.keyframes[i + 1]?.time
        if (!(prev < t && (next === undefined || t < next)))
          return `keyframe ${i} must lie between ${prev} and ${next ?? 'the end'}`
        doc.keyframes[i].time = t
      }),

    setEase: (i: number, ease: Ease) =>
      edit(doc => {
        doc.keyframes[i].ease = ease
      }),

    setBody: (i: number, axis: BodyAxis, value: number) =>
      edit(doc => {
        const body = bodyOf(doc.keyframes[i])
        body[axis] = value
        const [roll, pitch, yaw, x, y, z] = body
        doc.keyframes[i].body = { roll, pitch, yaw, x, y, z }
      }),

    // A target in the leg's current mode is stored as given. A mode flip ignores target.v and
    // converts the stored value instead, IK from foot to joints and FK from joints to foot, both
    // on the keyframe's body at the current stance and ride-height base.
    setLeg: (i: number, leg: number, target: Leg) =>
      edit(doc => {
        const k = doc.keyframes[i]
        const now = legTarget(k, leg)
        let next = target
        if (now.joints !== target.joints) {
          const body = bodyOf(k)
          const stance = stanceFor(controller[7])
          const baseZ = rideBase(doc, controller[4])
          next = {
            joints: target.joints,
            v:
              target.joints ?
                legJointsDeg(kinematics, body, now.v, leg, stance, baseZ)
              : footFromJoints(body, now.v, leg, stance, baseZ)
          }
        }
        if (k.legs.length === 0)
          k.legs = Array.from({ length: 6 }, () => ({ foot: { x: 0, y: 0, z: 0 } }))
        k.legs[leg] = targetOf(next)
      }),

    addOverlay: () =>
      edit(doc => {
        doc.overlays.push({
          bodyAxis: BodyAxis.ROLL,
          footChannel: undefined,
          amplitude: 0,
          frequency: 1,
          phase: 0,
          start: 0,
          end: duration(doc)
        })
      }),

    updateOverlay: (i: number, patch: Partial<Overlay>) =>
      edit(doc => {
        doc.overlays[i] = { ...doc.overlays[i], ...patch }
      }),

    removeOverlay: (i: number) =>
      edit(doc => {
        doc.overlays.splice(i, 1)
      }),

    setMeta: (
      patch: Partial<
        Pick<
          Animation,
          'name' | 'description' | 'loop' | 'holdEnd' | 'entryTime' | 'exitTime' | 'rideHeight'
        >
      >
    ) =>
      edit(doc => {
        Object.assign(doc, patch)
      }),

    setParam: (i: number, patch: Partial<Omit<ParamSpec, 'id'>>) =>
      edit(doc => {
        doc.params[i] = { ...doc.params[i], ...patch }
      }),

    addParam: (id: ParamId) =>
      edit(doc => {
        if (doc.params.some(p => p.id === id)) return `${ParamId[id]} is already exposed`
        const [min, defaultValue, max] = PARAM_RANGE[id] ?? MULTIPLIER_RANGE
        doc.params.push({ id, min, defaultValue, max })
      }),

    removeParam: (i: number) =>
      edit(doc => {
        doc.params.splice(i, 1)
      }),

    setScrub: (t: number) =>
      state.update(s => ({ ...s, scrub: Math.min(Math.max(t, 0), duration(s.document)) })),

    setValue: (id: ParamId, v: number) =>
      state.update(s => ({ ...s, values: new Map(s.values).set(id, v) })),

    setShowOnRobot: (showOnRobot: boolean) => state.update(s => ({ ...s, showOnRobot })),

    // The preview player's pose while it runs; null returns the view to the scrub head.
    setPlayback: (pose: Pose | null) => {
      playback.set(pose)
      if (current.playing !== (pose !== null)) state.update(s => ({ ...s, playing: pose !== null }))
    }
  }
}

export type Editor = ReturnType<typeof createEditor>

export const editor = createEditor()
