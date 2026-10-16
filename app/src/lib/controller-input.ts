import type { ControllerInputData } from '$lib/platform_shared/message'
import type { Command } from '$lib/simulation/virtual-robot'
import type { ControllerInput } from '$lib/types/models'

/** The controller's sticks and sliders as the firmware's command. */
export const commandOf = (i: ControllerInput): Command => [
  i.left.x,
  i.left.y,
  i.right.x,
  i.right.y,
  i.height,
  i.speed,
  i.s1,
  i.feetDistance,
  i.stride
]

/** The message a command is sent to the robot as. */
export const toControllerInput = (c: Command): ControllerInputData => ({
  left: { x: c[0], y: c[1] },
  right: { x: c[2], y: c[3] },
  height: c[4],
  speed: c[5],
  s1: c[6],
  feetDistance: c[7],
  stride: c[8]
})
