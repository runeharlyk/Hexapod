import { derived, get, writable } from 'svelte/store'
import { requestGait, requestMode } from '$lib/control'
import { GaitType } from '$lib/gait'
import { MotionModes } from '$lib/motion'
import type { Command } from '$lib/simulation/virtual-robot'
import { gait, input, mode, outControllerData } from './model-store'

/**
 * The simulation view's own robot: the sticks, mode and gait the controls give it. None of them reaches the real
 * robot, and none follows it.
 */
export const simulation = {
  command: writable<Command>([0, 0, 0, 0, 0, 0, 0, 0, 0]),
  mode: writable(MotionModes.DEACTIVATED),
  gait: writable(GaitType.TRI_GATE)
}

/** Whether the controls drive the simulation view's robot instead of the real one: true while the view is shown. */
export const drivesSimulation = writable(false)

/** The mode and gait of the robot the controls drive. */
export const drivenMode = derived([drivesSimulation, simulation.mode, mode], ([sim, own, robot]) =>
  sim ? own : robot
)
export const drivenGait = derived([drivesSimulation, simulation.gait, gait], ([sim, own, robot]) =>
  sim ? own : robot
)

/** The controller's sticks and sliders, to the robot the controls drive. */
export const drive = (command: Command) =>
  (get(drivesSimulation) ? simulation.command : outControllerData).set(command)

export const driveMode = (next: MotionModes) =>
  get(drivesSimulation) ? simulation.mode.set(next) : requestMode(next)

export const driveGait = (next: GaitType) =>
  get(drivesSimulation) ? simulation.gait.set(next) : requestGait(next)

const centred = (c: Command): Command => [0, 0, 0, 0, ...c.slice(4)] as Command

/**
 * Hands the controls to the simulation view's robot, or back to the real one. Both robots and the controls' own sticks
 * are centred on the way, so neither robot is left with a stick meant for the other, and the real robot is stopped
 * when the simulation takes over. Each robot keeps its own sliders, which the controls then show, so the next stick
 * sends neither robot the other's.
 */
export const driveSimulation = (on: boolean) => {
  outControllerData.update(centred)
  simulation.command.update(centred)
  const [, , , , height, speed, s1, feetDistance, stride] = get(
    on ? simulation.command : outControllerData
  )
  input.set({
    left: { x: 0, y: 0 },
    right: { x: 0, y: 0 },
    height,
    speed,
    s1,
    feetDistance,
    stride
  })
  drivesSimulation.set(on)
}
