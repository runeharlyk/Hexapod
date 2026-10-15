// The firmware's inverse kinematics (Kinematics::inverseKinematics in firmware/include/kinematics.h), run through
// the WebAssembly build's animation engine, whose legJointsDeg solves one foot on a body exactly as the robot does.
import type { AnimationEngine } from '$lib/animation/engine'
import { DEFAULT_FEET, type Vec3 } from '$lib/animation/model'
import type { body_state_t } from '$lib/kinematic'

export type LegRad = [number, number, number]

/** The joint angles in radians, per leg in IK order, that the firmware solves the feet of body into. */
export const firmwareInverseKinematics = (
  engine: AnimationEngine,
  body: body_state_t
): LegRad[] => {
  const pose = [body.omega, body.phi, body.psi, body.xm, body.ym, body.zm]
  return body.feet.map((foot, leg) => {
    const offset = [0, 1, 2].map(axis => foot[axis] - DEFAULT_FEET[leg][axis]) as Vec3
    return engine
      .legJointsDeg(pose, offset, leg, DEFAULT_FEET)
      .map(deg => (deg * Math.PI) / 180) as LegRad
  })
}
