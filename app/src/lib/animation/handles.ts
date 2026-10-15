import { DEFAULT_FEET } from './evaluator'
import type { Vec3 } from './model'

// Derived from orient_robot and the URDF load in populateModelCache. T * p = R * p + t, so a
// positive xm moves the feet +x against the body, i.e. the body -x in the world, which orient_robot
// renders as robot.position.z = -xm / 12: body +x is scene +z. By the same reading body +y is scene
// +x and body +z is scene +y (the up axis). The URDF rotation (x = -PI/2, z = PI/2) takes URDF
// (x, y, z) to scene (-y, z, -x), and motion.order places IK leg 0 on the URDF leg at (-x, -y), so
// the model agrees. The scene origin is the body origin of a zero pose, lowered to the ground plane
// by the stance height, so a default stance foot rests on the ground as the rendered robot does.
const MM_PER_UNIT = 12
const GROUND_Z = DEFAULT_FEET[0][2]

export const HANDLE_NEUTRAL = 0xffcc00
export const HANDLE_RED = 0xff0000

export const stanceToScene = ([x, y, z]: readonly number[]): Vec3 => [
  y / MM_PER_UNIT,
  (z - GROUND_Z) / MM_PER_UNIT,
  x / MM_PER_UNIT
]

export const sceneToOffset = (scenePos: readonly number[], stanceMm: readonly number[]): Vec3 => {
  const [sx, sy, sz] = stanceToScene(stanceMm)
  return [
    (scenePos[2] - sz) * MM_PER_UNIT,
    (scenePos[0] - sx) * MM_PER_UNIT,
    (scenePos[1] - sy) * MM_PER_UNIT
  ]
}

export const legMask = (mask: number, leg: number): number => (mask >>> (leg * 3)) & 0b111

export const handleColor = (clampedMaskForLeg: number): number =>
  clampedMaskForLeg === 0 ? HANDLE_NEUTRAL : HANDLE_RED
