import { GaitType } from '$lib/gait'
import { MessageTopic } from '$lib/interfaces/transport.interface'
import { MotionModes } from '$lib/motion'
import { dataBroker } from '$lib/transport/databroker'

export const requestMode = (mode: MotionModes) => {
  dataBroker.send(MessageTopic.MODE, Object.values(MotionModes).indexOf(mode))
}

export const requestGait = (gait: GaitType) => {
  dataBroker.send(MessageTopic.GAIT, Object.values(GaitType).indexOf(gait))
}
