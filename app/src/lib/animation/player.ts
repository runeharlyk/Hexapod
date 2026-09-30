// Port of Player, _blend_targets and _blend in simulation/src/robot/animation.py.
import { Ease, ParamId, type Animation } from '$lib/platform_shared/animation'
import type Kinematics from '$lib/kinematic'
import {
  PARAM_COUNT,
  STEP_ARC_FULL_TRAVEL_MM,
  STEP_ARC_MIN_TRAVEL_MM,
  STEP_ARC_MM,
  clonePose,
  duration,
  entrySeconds,
  exitSeconds,
  stancePose,
  type Leg,
  type Pose,
  type Vec3
} from './model'
import { easeValue, evaluate, legJointsDeg, resolveParams, type Stance } from './evaluator'

export enum State {
  IDLE,
  ENTRY,
  PLAYING,
  HOLD,
  EXIT
}

// Legs where either side is a joint target are converted to joints on both sides once, so the
// blend itself is a plain lerp. The source is converted at the base it is output on now, the
// destination at the base held when the blend ends.
const blendTargets = (
  kin: Kinematics,
  src: Pose,
  dst: Pose,
  stance: Stance,
  srcBaseZ: number,
  dstBaseZ: number
): [Pose, Pose] => {
  const a = clonePose(src)
  const b = clonePose(dst)
  for (let i = 0; i < 6; i++) {
    if (!a.legs[i].joints && !b.legs[i].joints) continue
    if (!a.legs[i].joints)
      a.legs[i] = { joints: true, v: legJointsDeg(kin, a.body, a.legs[i].v, i, stance, srcBaseZ) }
    if (!b.legs[i].joints)
      b.legs[i] = { joints: true, v: legJointsDeg(kin, b.body, b.legs[i].v, i, stance, dstBaseZ) }
  }
  return [a, b]
}

const blend = (a: Pose, b: Pose, u: number): Pose => {
  const e = easeValue(Ease.EASE_IN_OUT, u)
  const legs = a.legs.map((la, i): Leg => {
    const lb = b.legs[i]
    const v = la.v.map((x, axis) => x + (lb.v[axis] - x) * e) as Vec3
    if (la.joints) return { joints: true, v }
    const travel = Math.hypot(lb.v[0] - la.v[0], lb.v[1] - la.v[1])
    if (travel > STEP_ARC_MIN_TRAVEL_MM)
      v[2] += STEP_ARC_MM * Math.min(1, travel / STEP_ARC_FULL_TRAVEL_MM) * Math.sin(Math.PI * u)
    return { joints: false, v }
  })
  return { body: a.body.map((x, i) => x + (b.body[i] - x) * e), legs }
}

// Entry -> Playing -> Hold | Exit -> Idle around evaluate(). A non-looping animation plays
// max(1, floor(REPEAT + 0.5)) times. baseZ is the ride height the runner adds to the body z at the
// call; it only moves the foot-to-joint conversions. Entry blends toward the base held once it
// ends (the animation's rideHeight when set, else baseZ) and Exit toward baseZ.
export class Player {
  state = State.IDLE
  clip: Animation | undefined = undefined
  params: number[] = new Array<number>(PARAM_COUNT).fill(1)
  t = 0
  lastPose: Pose = stancePose()
  entryBase = 0
  private playsDone = 0
  private blendT = 0
  private blendSeconds = 1
  private blendFrom: Pose = stancePose()
  private blendTo: Pose = stancePose()

  constructor(
    private readonly kin: Kinematics,
    private readonly stance: Stance
  ) {}

  play(a: Animation, values: Map<ParamId, number> | undefined, live: Pose | undefined, baseZ = 0) {
    this.clip = a
    this.params = resolveParams(a, values)
    this.t = 0
    this.playsDone = 0
    const start = live ?? this.lastPose
    this.lastPose = clonePose(start)
    this.entryBase = a.rideHeight ?? baseZ
    this.startBlend(
      start,
      this.evaluateAt(a, 0, this.entryBase),
      entrySeconds(a),
      State.ENTRY,
      baseZ,
      this.entryBase
    )
  }

  stop(baseZ = 0) {
    if (this.state === State.IDLE || !this.clip) return
    this.startBlend(this.lastPose, stancePose(), exitSeconds(this.clip), State.EXIT, baseZ, baseZ)
  }

  update(dt: number, baseZ = 0): Pose {
    if (this.state === State.IDLE || !this.clip) return this.lastPose
    let pose: Pose
    if (this.state === State.ENTRY || this.state === State.EXIT) pose = this.advanceBlend(dt)
    else if (this.state === State.HOLD)
      pose = this.evaluateAt(this.clip, duration(this.clip), baseZ)
    else pose = this.advancePlaying(this.clip, dt, baseZ)
    this.lastPose = pose
    return pose
  }

  // The last Entry or Exit blend's progress, 0 to 1. A runner moves its base along it during
  // Entry, from the base at play() to entryBase, so the base arrives exactly when Entry ends.
  blendFraction(): number {
    return Math.min(1, this.blendT / this.blendSeconds)
  }

  private evaluateAt(a: Animation, t: number, baseZ: number): Pose {
    return evaluate(a, this.params, t, this.kin, this.stance, baseZ)
  }

  private startBlend(
    src: Pose,
    dst: Pose,
    seconds: number,
    state: State,
    srcBaseZ: number,
    dstBaseZ: number
  ) {
    ;[this.blendFrom, this.blendTo] = blendTargets(
      this.kin,
      src,
      dst,
      this.stance,
      srcBaseZ,
      dstBaseZ
    )
    this.blendSeconds = seconds
    this.blendT = 0
    this.state = state
  }

  private advanceBlend(dt: number): Pose {
    this.blendT += dt
    const u = Math.min(1, this.blendT / this.blendSeconds)
    const pose = blend(this.blendFrom, this.blendTo, u)
    if (u >= 1) {
      if (this.state === State.ENTRY) {
        this.state = State.PLAYING
        this.t = 0
      } else {
        this.state = State.IDLE
      }
    }
    return pose
  }

  private advancePlaying(a: Animation, dt: number, baseZ: number): Pose {
    const d = duration(a)
    this.t += dt * this.params[ParamId.SPEED]
    if (a.loop) {
      // t is never negative, so % is the exact non-negative fmod of the reference.
      this.t = d > 0 ? this.t % d : 0
      return this.evaluateAt(a, this.t, baseZ)
    }
    if (this.t < d) return this.evaluateAt(a, this.t, baseZ)
    this.playsDone++
    if (this.playsDone < Math.max(1, Math.floor(this.params[ParamId.REPEAT] + 0.5))) {
      this.t = d > 0 ? this.t - d : 0
      return this.evaluateAt(a, this.t, baseZ)
    }
    const final = this.evaluateAt(a, d, baseZ)
    if (a.holdEnd) {
      this.state = State.HOLD
    } else {
      this.lastPose = final
      this.stop(baseZ)
    }
    return final
  }
}
