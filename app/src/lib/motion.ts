import { ModesEnum } from '$lib/platform_shared/message'

export enum MotionModes {
  DEACTIVATED = 'deactivated',
  IDLE = 'idle',
  STAND = 'stand',
  WALK = 'walk',
  WALK_NN = 'walk nn',
  ANIMATE = 'animate'
}

// Each mode crosses the wire as its ModesEnum value, as the firmware's MOTION_STATE does. The proto reserves POSE's 2,
// so a mode's value is not its position in MotionModes and is looked up rather than counted.
const WIRE: Record<MotionModes, ModesEnum> = {
  [MotionModes.DEACTIVATED]: ModesEnum.DEACTIVATED,
  [MotionModes.IDLE]: ModesEnum.IDLE,
  [MotionModes.STAND]: ModesEnum.STAND,
  [MotionModes.WALK]: ModesEnum.WALK,
  [MotionModes.WALK_NN]: ModesEnum.WALK_NN,
  [MotionModes.ANIMATE]: ModesEnum.ANIMATE
}

/** The value a mode is sent and reported as. */
export const modeToWire = (mode: MotionModes): ModesEnum => WIRE[mode]

/** The mode a reported value is, or undefined for one this app knows no mode for, such as POSE's reserved 2. */
export const modeFromWire = (value: number): MotionModes | undefined =>
  Object.values(MotionModes).find(mode => WIRE[mode] === value)

/** The firmware's 18 joint angles, three per leg in leg order, in the order of the URDF model's joint names. */
export const urdfJointOrder = <T>(a: T[]) =>
  [3, 0, 4, 1, 5, 2].flatMap(i => a.slice(i * 3, i * 3 + 3))
