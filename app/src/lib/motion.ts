import { ModesEnum } from '$lib/platform_shared/message'
import { describeFailure } from '$lib/transport/reply'

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

/**
 * What a page says of a mode the robot reports and modeFromWire knows nothing of, such as an older firmware's POSE: the
 * robot is in no mode this app can show or drive.
 */
export const UNKNOWN_MODE_HINT = `The robot reports a mode this app does not know: ${describeFailure(
  { kind: 'too-old' },
  "this app's modes"
)}`

/** Every servo, a bit per servo in the firmware's order, as AnglesData's limp marks them. */
export const ALL_SERVOS = (1 << 18) - 1

/** The angles the robot reports (AnglesData): 18 in the firmware's order, with the servos it marks limp, or none. */
export type AnglesReport = { angles: readonly number[]; limp: number }

/**
 * The servos the robot reports limp, a bit per servo: driven by no pulse since its boards woke, so no angle is known for
 * them. An empty report is every servo limp, as the robot reports while it drives none.
 */
export const limpOf = ({ angles, limp }: AnglesReport) =>
  angles.length === 18 ? limp & ALL_SERVOS : ALL_SERVOS

/** How many servos a mask of them holds, a bit per servo. */
export const servoCount = (mask: number) =>
  Array.from({ length: 18 }, (_, servo) => (mask >> servo) & 1).reduce((sum, bit) => sum + bit, 0)

/**
 * The pose the 3D view draws from the angles the robot reports, in degrees in the firmware's order: none for an empty
 * report, and for a servo reported limp the angle `drawnDeg` gives it, what the view draws now, since its reported 0 is
 * no angle. A servo being dragged keeps the drag's angle, since the robot's echo of it trails the pointer.
 */
export const reportedPose = (
  report: AnglesReport,
  drawnDeg: readonly number[],
  dragged?: { servo: number; degrees: number }
): number[] | null => {
  if (report.angles.length !== 18) return null
  const pose = report.angles.map((angle, servo) =>
    report.limp & (1 << servo) ? drawnDeg[servo] : angle
  )
  if (dragged) pose[dragged.servo] = dragged.degrees
  return pose
}

/** The firmware's 18 joint angles, three per leg in leg order, in the order of the URDF model's joint names. */
export const urdfJointOrder = <T>(a: T[]) =>
  [3, 0, 4, 1, 5, 2].flatMap(i => a.slice(i * 3, i * 3 + 3))
