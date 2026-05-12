import { GaitType } from '$lib/gait'
import { MessageTopic } from '$lib/interfaces/transport.interface'
import { MotionModes } from '$lib/motion'
import { gait as gaitStore, mode as modeStore } from '$lib/stores'
import { dataBroker } from '$lib/transport/databroker'

export const requestMode = (nextMode: MotionModes) => {
  modeStore.set(nextMode)
  dataBroker.send(MessageTopic.MODE, Object.values(MotionModes).indexOf(nextMode), true)
}

export const requestGait = (nextGait: GaitType) => {
  gaitStore.set(nextGait)
  dataBroker.send(MessageTopic.GAIT, Object.values(GaitType).indexOf(nextGait), true)
}
