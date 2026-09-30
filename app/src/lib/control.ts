import { GaitType } from '$lib/gait'
import { MotionModes } from '$lib/motion'
import { gait as gaitStore, mode as modeStore } from '$lib/stores'
import { dataBroker } from '$lib/transport/databroker'
import { get } from 'svelte/store'
import { throttler } from '$lib/utilities'
import { isLinked } from '$lib/stores/link'
import type { ParamId, LegTarget } from '$lib/platform_shared/animation'
import type { Pose } from '$lib/animation/model'
import {
  AnimationPlay,
  AnimationStop,
  ModeData,
  GaitData,
  PoseData
} from '$lib/platform_shared/message'

export const requestMode = (nextMode: MotionModes) => {
  modeStore.set(nextMode)
  dataBroker.emit(ModeData, { mode: Object.values(MotionModes).indexOf(nextMode) })
}

export const requestGait = (nextGait: GaitType) => {
  gaitStore.set(nextGait)
  dataBroker.emit(GaitData, { gait: Object.values(GaitType).indexOf(nextGait) })
}

export const playAnimation = (name: string, values: Map<ParamId, number>) => {
  dataBroker.emit(AnimationPlay, {
    name,
    params: [...values].map(([id, value]) => ({ id, value }))
  })
}

export const stopAnimation = () => dataBroker.emit(AnimationStop, {})

// The robot ignores a play from DEACTIVATED or IDLE. The mode store only learns the robot's mode
// from changes after connecting, so this is a warning and the play is still sent.
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
    const legs = pose.legs.map(
      (leg): LegTarget =>
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
    this.limiter.throttle(() => {
      if (get(isLinked)) dataBroker.emit(PoseData, data)
    }, POSE_INTERVAL_MS)
  }

  cancel() {
    this.limiter.cancel()
  }
}
