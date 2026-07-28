import { GaitType } from '$lib/gait'
import { MotionModes } from '$lib/motion'
import { gait as gaitStore, mode as modeStore } from '$lib/stores'
import { dataBroker } from '$lib/transport/databroker'
import { ModeData, GaitData } from '$lib/platform_shared/message'

export const requestMode = (nextMode: MotionModes) => {
  modeStore.set(nextMode)
  dataBroker.emit(ModeData, { mode: Object.values(MotionModes).indexOf(nextMode) })
}

export const requestGait = (nextGait: GaitType) => {
  gaitStore.set(nextGait)
  dataBroker.emit(GaitData, { gait: Object.values(GaitType).indexOf(nextGait) })
}
