import type { HexapodConfig } from '$lib/kinematic'
import type { Servo } from '$lib/platform_shared/api'

/** Joint angles in degrees, in the firmware's inverse-kinematics terms: coxa, femur, tibia. */
export type LegAngles = [number, number, number]
/** A point in the body frame, in mm: x lateral (right), y forward, z up. */
export type BodyPoint = [number, number, number]
/** A point in a leg's vertical plane, in mm: r outwards from the coxa axis, z up. */
export type PlanePoint = [number, number]

/** Leg order of Kinematics::inverseKinematics: right legs first (positive mountX), front to rear. */
export const LEG_NAMES = [
  'Right front',
  'Right middle',
  'Right rear',
  'Left front',
  'Left middle',
  'Left rear'
] as const
export const JOINT_NAMES = ['Coxa', 'Femur', 'Tibia'] as const

/** The servo order of ServoController::setAngles: three joints per leg. */
export const servoJoint = (servoId: number) => ({
  leg: Math.floor(servoId / 3),
  joint: servoId % 3
})

/** What ServoController::setAngles adds to each joint's angle before toPwm: the tibia's square knee. */
export const SERVO_ANGLE_OFFSET: LegAngles = [0, 0, 90]

/** The joint angle the firmware drives a servo to with this PWM: ServoController::toPwm, inverted. */
export const jointAngleFromPwm = (servo: Servo, joint: number, pwm: number) =>
  (pwm - servo.centerPwm) / (servo.direction * servo.conversion) -
  servo.centerAngle -
  SERVO_ANGLE_OFFSET[joint]

/** The joint angle at the servo's centre PWM: the pose to line the real joint up with. */
export const referenceAngle = (servo: Servo, joint: number) =>
  -servo.centerAngle - SERVO_ANGLE_OFFSET[joint]

/** Every joint of a leg at its centre PWM, as the firmware's POSE mode (setCenterPwm) holds them. */
export const referenceLeg = (servos: Servo[], leg: number): LegAngles => [
  referenceAngle(servos[leg * 3], 0),
  referenceAngle(servos[leg * 3 + 1], 1),
  referenceAngle(servos[leg * 3 + 2], 2)
]

/** A leg at its reference pose, except the calibrated joint, which sits where the PWM drives it. */
export function calibrationLeg(servos: Servo[], servoId: number, pwm: number): LegAngles {
  const { leg, joint } = servoJoint(servoId)
  const angles = referenceLeg(servos, leg)
  angles[joint] = jointAngleFromPwm(servos[servoId], joint, pwm)
  return angles
}

/** The angle between femur and tibia: 180 when the leg is straight, 90 when square. */
export const kneeAngle = (tibia: number) => 180 + tibia

const DEG = Math.PI / 180

/**
 * A leg's joints in its own vertical plane, the inverse of Kinematics::inverseKinematics: the coxa link runs
 * level from the coxa axis, the femur pitches up from level, the tibia turns from the femur.
 */
export function legPlanePoints(cfg: HexapodConfig, [, femur, tibia]: LegAngles) {
  const femurPitch = femur * DEG
  const tibiaPitch = (femur + tibia) * DEG
  const femurAxis: PlanePoint = [cfg.legJoint1ToJoint2, 0]
  const knee: PlanePoint = [
    femurAxis[0] + cfg.legJoint2ToJoint3 * Math.cos(femurPitch),
    cfg.legJoint2ToJoint3 * Math.sin(femurPitch)
  ]
  const foot: PlanePoint = [
    knee[0] + cfg.legJoint3ToTip * Math.cos(tibiaPitch),
    knee[1] + cfg.legJoint3ToTip * Math.sin(tibiaPitch)
  ]
  return { coxaAxis: [0, 0] as PlanePoint, femurAxis, knee, foot }
}

/** A leg's joints in the body frame; a positive coxa angle turns the leg counter-clockwise seen from above. */
export function legPoints(cfg: HexapodConfig, leg: number, angles: LegAngles) {
  const mountAngle = cfg.legMountAngle[leg] * DEG
  const heading = mountAngle + angles[0] * DEG
  const mount: BodyPoint = [cfg.legMountX[leg], cfg.legMountY[leg], 0]
  const coxaAxis: BodyPoint = [
    mount[0] + cfg.legRootToJoint1 * Math.cos(mountAngle),
    mount[1] + cfg.legRootToJoint1 * Math.sin(mountAngle),
    0
  ]
  const place = ([r, z]: PlanePoint): BodyPoint => [
    coxaAxis[0] + r * Math.cos(heading),
    coxaAxis[1] + r * Math.sin(heading),
    z
  ]
  const plane = legPlanePoints(cfg, angles)
  return {
    mount,
    coxaAxis,
    femurAxis: place(plane.femurAxis),
    knee: place(plane.knee),
    foot: place(plane.foot)
  }
}

/**
 * Which way a leg's outward direction points when its plane is seen from behind the robot: +1 to the right, -1 to
 * the left. The side view is mirrored by this so that it shows a left leg as it looks from behind, too.
 */
export const outwardSide = (cfg: HexapodConfig, leg: number) =>
  Math.cos(cfg.legMountAngle[leg] * DEG) >= 0 ? 1 : -1
