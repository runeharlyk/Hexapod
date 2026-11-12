import { derived, get, writable } from 'svelte/store'
import { playAnimation, stopAnimation, requestGait, requestMode, requestStance } from '$lib/control'
import { GaitType } from '$lib/gait'
import { Stance } from '$lib/stance'
import { MotionModes } from '$lib/motion'
import { haptics } from '$lib/utilities/haptics'
import type { Command } from '$lib/simulation/virtual-robot'
import { gait, goalStatus, mode, stance } from '$lib/robot/state'
import { input, outControllerData } from './model-store'
import type { ParamId } from '$lib/platform_shared/animation'
import type { AnimationStatus } from '$lib/platform_shared/message'
import type { VirtualRobot } from '$lib/simulation/virtual-robot'

/**
 * The simulation view's own robot: the sticks, mode, gait and stance the controls give it. None of them reaches the real
 * robot, and none follows it. The view sets the gait and the stance to those its robot reports (SimulationView.svelte).
 */
export const simulation = {
  command: writable<Command>([0, 0, 0, 0, 0, 0, 0, 0, 0, false, false]),
  mode: writable(MotionModes.DEACTIVATED),
  gait: writable(GaitType.TRI_GATE),
  stance: writable(Stance.DEFAULT),
  player: writable<Pick<VirtualRobot, 'playAnimation' | 'stopAnimation'> | null>(null),
  animationStatus: writable<AnimationStatus | null>(null)
}

/** Whether the controls drive the simulation view's robot instead of the real one: true while the view is shown. */
export const drivesSimulation = writable(false)

/** The mode, gait and stance of the robot the controls drive. */
export const drivenMode = derived([drivesSimulation, simulation.mode, mode], ([sim, own, robot]) =>
  sim ? own : robot
)
export const drivenGait = derived([drivesSimulation, simulation.gait, gait], ([sim, own, robot]) =>
  sim ? own : robot
)
export const drivenStance = derived(
  [drivesSimulation, simulation.stance, stance],
  ([sim, own, robot]) => (sim ? own : robot)
)

/** The robot's goal while the controls drive it; the simulation view's robot runs none. */
export const drivenGoal = derived([drivesSimulation, goalStatus], ([sim, goal]) =>
  sim ? null : goal
)

/** The controller's sticks and sliders, to the robot the controls drive. */
export const drive = (command: Command) =>
  (get(drivesSimulation) ? simulation.command : outControllerData).set(command)

/**
 * Asks for a mode, felt on the phone only when it differs from the current one: a gamepad button held while a stick
 * moves asks again on every change.
 */
export const driveMode = (next: MotionModes) => {
  if (next !== get(drivenMode))
    (next === MotionModes.DEACTIVATED ? haptics.stop : haptics.modeChange)()
  return get(drivesSimulation) ? simulation.mode.set(next) : requestMode(next)
}

export const driveGait = (next: GaitType) =>
  get(drivesSimulation) ? simulation.gait.set(next) : requestGait(next)

export const driveStance = (next: Stance) =>
  get(drivesSimulation) ? simulation.stance.set(next) : requestStance(next)

export const playDrivenAnimation = (name: string, values: Map<ParamId, number>) =>
  get(drivesSimulation) ?
    get(simulation.player)?.playAnimation(name, values)
  : playAnimation(name, values)

export const stopDrivenAnimation = () =>
  get(drivesSimulation) ? get(simulation.player)?.stopAnimation() : stopAnimation()

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
  const [, , , , height, speed, s1, feetDistance, stride, holdHeading, climb] = get(
    on ? simulation.command : outControllerData
  )
  input.set({
    left: { x: 0, y: 0 },
    right: { x: 0, y: 0 },
    height,
    speed,
    s1,
    feetDistance,
    stride,
    holdHeading,
    climb
  })
  drivesSimulation.set(on)
}
