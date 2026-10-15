// The app draws the robot with its own forward kinematics, which the firmware has none of; the drawing is true when
// that forward kinematics inverts the firmware's inverse kinematics, compiled to WebAssembly
// (scripts/build_virtual_robot.js) and run here on the robot's own geometry.
import { beforeAll, describe, expect, it } from 'vitest'
import Kinematics, { levelBody, type body_state_t } from '../../src/lib/kinematic'
import { config } from '../../src/lib/components/config'
import { loadAnimationEngine, type AnimationEngine } from '../../src/lib/animation/engine'
import { firmwareInverseKinematics, type LegRad } from './firmware-ik'

// The firmware solves in float32, a ten-thousandth of a degree and a thousandth of a millimetre at this size.
const ANGLE_RAD = 4
const FOOT_MM = 2

const kinematics = new Kinematics(config)
let engine: AnimationEngine

// inverseKinematics resolves the elbow with a fixed sign, so its image is the knee-down branch only: tibia <= 0.
const kneeDown: LegRad[] = [
  [0.1, 0.2, -1.3],
  [-0.2, 0.3, -1.4],
  [0.15, -0.1, -1.2],
  [-0.1, 0.25, -1.6],
  [0.2, -0.15, -1.25],
  [0.0, 0.1, -1.5]
]

const bodies: [string, body_state_t][] = [
  ['a level body', levelBody()],
  [
    'a tilted, turned and shifted body',
    { ...levelBody(), omega: 0.05, phi: -0.1, psi: 0.15, xm: 5, ym: -10, zm: 15 }
  ]
]

const withFeet = (body: body_state_t, angles: LegRad[]): body_state_t => ({
  ...body,
  feet: kinematics.forwardKinematics(body, angles)
})

describe('forward kinematics', () => {
  beforeAll(async () => {
    engine = await loadAnimationEngine()
  })

  it('reaches every foot straight out along its mount at zero angles', () => {
    const reach =
      config.legRootToJoint1 +
      config.legJoint1ToJoint2 +
      config.legJoint2ToJoint3 +
      config.legJoint3ToTip
    kinematics
      .forwardKinematics(levelBody(), Array(6).fill([0, 0, 0]))
      .forEach(([x, y, z, w], leg) => {
        const mount = (config.legMountAngle[leg] * Math.PI) / 180
        expect(x).toBeCloseTo(config.legMountX[leg] + reach * Math.cos(mount), 9)
        expect(y).toBeCloseTo(config.legMountY[leg] + reach * Math.sin(mount), 9)
        expect(z).toBeCloseTo(0, 9)
        expect(w).toBe(1)
      })
  })

  it.each(bodies)("inverts the firmware's inverse kinematics on %s", (_, body) => {
    firmwareInverseKinematics(engine, withFeet(body, kneeDown)).forEach((solved, leg) =>
      solved.forEach((rad, joint) => expect(rad).toBeCloseTo(kneeDown[leg][joint], ANGLE_RAD))
    )
  })

  it.each(bodies)(
    'puts a knee-up pose on the knee-down branch at the same foot on %s',
    (_, body) => {
      const kneeUp = kneeDown.map(([coxa, femur, tibia]): LegRad => [coxa, femur, -tibia])
      const feet = withFeet(body, kneeUp)
      const solved = firmwareInverseKinematics(engine, feet)
      const reached = kinematics.forwardKinematics(body, solved)
      solved.forEach(([, , tibia], leg) => {
        expect(tibia).toBeLessThanOrEqual(0)
        for (let axis = 0; axis < 3; axis++)
          expect(reached[leg][axis]).toBeCloseTo(feet.feet[leg][axis], FOOT_MM)
      })
    }
  )
})
