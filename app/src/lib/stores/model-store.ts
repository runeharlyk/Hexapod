import { GaitType } from '$lib/gait'
import { MotionModes } from '$lib/motion'
import type { ControllerInput } from '$lib/types/models'
import { persistentStore } from '$lib/utilities/svelte-utilities'
import { writable, type Writable } from 'svelte/store'

export const emulateModel = writable(true)

export const jointNames = persistentStore('joint_names', <string[]>[])

export const model = writable()

export const mode: Writable<MotionModes> = writable(MotionModes.DEACTIVATED)

export const gait: Writable<GaitType> = writable(GaitType.TRI_GATE)

export const outControllerData = writable([0, 0, 0, 0, 0, 0, 0, 0])

export const input: Writable<ControllerInput> = writable({
  left: { x: 0, y: 0 },
  right: { x: 0, y: 0 },
  height: 0,
  speed: 0,
  s1: 0,
  feetDistance: 0
})
