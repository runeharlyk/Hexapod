import { GaitType } from '$lib/gait'
import { MotionModes } from '$lib/motion'
import { mode as modeStore } from '$lib/robot/state'
import { dataBroker } from '$lib/transport/databroker'
import { describeFailure } from '$lib/transport/reply'
import { notifications } from '$lib/components/toasts/notifications'
import { get } from 'svelte/store'
import { throttler } from '$lib/utilities'
import type { ParamId, LegTarget } from '$lib/platform_shared/animation'
import type { Pose } from '$lib/animation/model'
import {
  AnimationPlay,
  AnimationStop,
  ModeData,
  GaitData,
  PoseData,
  type MessageFns
} from '$lib/platform_shared/message'

// An action that moves the robot fails at once while no link is up, and says so, rather than wait for a link and move a
// robot whose state may have changed meanwhile (transport/request-kinds.ts).
const act = <T>(fns: MessageFns<T>, data: T, failed: string) => {
  const sent = dataBroker.emit(fns, data)
  if (!sent) notifications.error(`${failed}: ${describeFailure({ kind: 'disconnected' }, '')}`)
  return sent
}

// The mode and the gait shown are the robot's, from its pushes, never the request's: a mode it refuses pushes nothing.
export const requestMode = (nextMode: MotionModes) => {
  // The robot reads the mode alone; whether the handheld drives is its own to report.
  const mode = { mode: Object.values(MotionModes).indexOf(nextMode), handheld: false }
  act(ModeData, mode, 'Mode not changed')
}

export const requestGait = (nextGait: GaitType) => {
  const gait = { gait: Object.values(GaitType).indexOf(nextGait) }
  act(GaitData, gait, 'Gait not changed')
}

export const playAnimation = (name: string, values: Map<ParamId, number>) => {
  const play = { name, params: [...values].map(([id, value]) => ({ id, value })) }
  act(AnimationPlay, play, `${name} not played`)
}

export const stopAnimation = () => dataBroker.emit(AnimationStop, {})

// The robot ignores a play from DEACTIVATED or IDLE. The mode shown may be a stale one kept from a link that dropped, so
// this is a warning and the play is still sent.
export const inactiveModeWarning = (): string | null => {
  const current = get(modeStore)
  return current === MotionModes.DEACTIVATED || current === MotionModes.IDLE ?
      `Robot is ${current.toUpperCase()}; pick STAND on the controller page`
    : null
}

const POSE_INTERVAL_MS = 50

export class PoseSender {
  private readonly limiter = new throttler()
  private last: PoseData | undefined

  send(pose: Pose) {
    const [roll, pitch, yaw, x, y, z] = pose.body
    const legs = pose.legs.map((leg): LegTarget =>
      leg.joints ?
        { joints: { coxa: leg.v[0], femur: leg.v[1], tibia: leg.v[2] } }
      : { foot: { x: leg.v[0], y: leg.v[1], z: leg.v[2] } }
    )
    this.last = { body: { roll, pitch, yaw, x, y, z }, legs }
    this.emitLast()
  }

  // The robot drops a pose that arrives before it is in ANIMATE or while a play runs, and nothing
  // follows until the pose changes; this sends the last one again.
  resend() {
    this.emitLast()
  }

  private emitLast() {
    const data = this.last
    if (!data) return
    this.limiter.throttle(() => dataBroker.emitVolatile(PoseData, data), POSE_INTERVAL_MS)
  }

  cancel() {
    this.limiter.cancel()
  }
}
