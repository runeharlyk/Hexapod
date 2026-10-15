// The firmware's animation evaluator and player (firmware/include/animation/animation.h) compiled to WebAssembly
// (firmware/host/animation_api.cpp, built by scripts/build_virtual_robot.js), so the editor previews a clip with the
// code the robot plays it with.
import { Animation, type Ease, type ParamId } from '$lib/platform_shared/animation'
import type { VirtualRobotModule } from '$lib/simulation/virtual-robot/virtual_robot.mjs'
import type { Pose, Stance, Vec3 } from './model'

export type ParamValues = ReadonlyMap<ParamId, number> | undefined

export enum State {
  IDLE,
  ENTRY,
  PLAYING,
  HOLD,
  EXIT
}

/** Entry -> Playing -> Hold | Exit -> Idle; baseZ is the ride height the runner adds to the body z at the call. */
export interface Player {
  readonly state: State
  /** The playing clock in seconds. */
  readonly t: number
  /** The last Entry or Exit blend's progress, 0 to 1. */
  readonly blendFraction: number
  /** The base the last Entry converted its destination at, which the runner holds once Entry ends. */
  readonly entryBase: number
  readonly lastPose: Pose
  /** Enters from live, or from the pose last output when live is undefined. Throws on a clip that does not decode. */
  play(a: Animation, values: ParamValues, live: Pose | undefined, baseZ?: number): void
  stop(baseZ?: number): void
  update(dt: number, baseZ?: number): Pose
  /** Frees the player; it must not be used afterwards. */
  dispose(): void
}

export interface AnimationEngine {
  /** The clip's pose at t clamped to its duration, or null when the clip does not decode. */
  evaluate(
    a: Animation,
    values: ParamValues,
    t: number,
    stance: Stance,
    baseZ?: number
  ): Pose | null
  /** 18 servo angles in degrees and the 18-bit mask of joints at a limit or of a foot leg out of reach. */
  poseToAngles(pose: Pose, stance: Stance): { angles: number[]; mask: number }
  /** The joints of a foot offset, in degrees, solved on body at the ride-height base. */
  legJointsDeg(
    body: readonly number[],
    foot: Vec3,
    leg: number,
    stance: Stance,
    baseZ?: number
  ): Vec3
  ease(kind: Ease, t: number): number
  player(stance: Stance): Player
}

const POSE = 0
const ANGLES = 30

const loadModule = async (): Promise<VirtualRobotModule> => {
  const { default: create } = await import('$lib/simulation/virtual-robot/virtual_robot.mjs')
  return create()
}

// A session is one clip, its parameter values, the standing feet and a player in the module's memory (see
// animation_api.cpp); its io buffer carries poses and angles.
const sessions = (m: VirtualRobotModule) => {
  // Memory growth replaces the heap views, so they are read at every use.
  const io = (h: number) => m._animIo(h) / 4

  const writePose = (h: number, pose: Pose) => {
    const at = io(h) + POSE
    m.HEAPF32.set(pose.body, at)
    pose.legs.forEach((leg, i) => m.HEAPF32.set([leg.joints ? 1 : 0, ...leg.v], at + 6 + i * 4))
  }

  const readPose = (h: number): Pose => {
    const at = io(h) + POSE
    const f = m.HEAPF32
    return {
      body: Array.from(f.subarray(at, at + 6)),
      legs: Array.from({ length: 6 }, (_, i) => ({
        joints: f[at + 6 + i * 4] !== 0,
        v: Array.from(f.subarray(at + 7 + i * 4, at + 10 + i * 4)) as Vec3
      }))
    }
  }

  const angles = (h: number, count: number) => {
    const at = io(h) + ANGLES
    return Array.from(m.HEAPF32.subarray(at, at + count))
  }

  const setStance = (h: number, stance: Stance) =>
    m.HEAPF32.set(
      stance.flatMap(foot => [...foot]),
      m._animStance(h) / 4
    )

  const load = (h: number, a: Animation, values: ParamValues): string | null => {
    const bytes = Animation.encode(a).finish()
    m.HEAPU8.set(bytes, m._animClipBuffer(h, bytes.length))
    const error = m._animLoad(h, bytes.length)
    if (error) return m.UTF8ToString(error)
    m._animClearParams(h)
    values?.forEach((value, id) => m._animSetParam(h, id, value))
    return null
  }

  return { writePose, readPose, angles, setStance, load }
}

export async function loadAnimationEngine(): Promise<AnimationEngine> {
  const m = await loadModule()
  const { writePose, readPose, angles, setStance, load } = sessions(m)
  const h = m._animCreate()

  return {
    evaluate: (a, values, t, stance, baseZ = 0) => {
      if (load(h, a, values)) return null
      setStance(h, stance)
      m._animEvaluate(h, t, baseZ)
      return readPose(h)
    },

    poseToAngles: (pose, stance) => {
      setStance(h, stance)
      writePose(h, pose)
      const mask = m._animPoseToAngles(h) >>> 0
      return { angles: angles(h, 18), mask }
    },

    legJointsDeg: (body, foot, leg, stance, baseZ = 0) => {
      setStance(h, stance)
      const legs = Array.from({ length: 6 }, () => ({ joints: false, v: [0, 0, 0] as Vec3 }))
      legs[leg].v = foot
      writePose(h, { body: [...body], legs })
      m._animLegJoints(h, leg, baseZ)
      return angles(h, 3) as Vec3
    },

    ease: (kind, t) => m._animEase(kind, t),

    player: stance => {
      const p = m._animCreate()
      setStance(p, stance)
      return {
        get state() {
          return m._animState(p) as State
        },
        get t() {
          return m._animTime(p)
        },
        get blendFraction() {
          return m._animBlendFraction(p)
        },
        get entryBase() {
          return m._animEntryBase(p)
        },
        get lastPose() {
          m._animLastPose(p)
          return readPose(p)
        },
        play: (a, values, live, baseZ = 0) => {
          const error = load(p, a, values)
          if (error) throw new Error(`the clip does not decode: ${error}`)
          if (live) writePose(p, live)
          m._animPlay(p, live !== undefined, baseZ)
        },
        stop: (baseZ = 0) => m._animStop(p, baseZ),
        update: (dt, baseZ = 0) => {
          m._animUpdate(p, dt, baseZ)
          return readPose(p)
        },
        dispose: () => m._animDestroy(p)
      }
    }
  }
}
