import { beforeAll, describe, it, expect } from 'vitest'
import { readFileSync } from 'node:fs'
import { resolve } from 'node:path'
import type { HexapodConfig, body_state_t } from '../../src/lib/kinematic'
import { loadAnimationEngine, type AnimationEngine } from '../../src/lib/animation/engine'
import { firmwareInverseKinematics } from './firmware-ik'
import { config } from '../../src/lib/components/config'
import {
  JOINT_NAMES,
  LEG_NAMES,
  SERVO_ANGLE_OFFSET,
  calibrationLeg,
  jointAngleFromPwm,
  kneeAngle,
  legPlanePoints,
  legPoints,
  outwardSide,
  referenceAngle,
  referenceLeg,
  servoJoint,
  type LegAngles
} from '../../src/lib/calibration/leg-pose'
import type { Servo } from '../../src/lib/platform_shared/api'

const firmware = (file: string) =>
  readFileSync(resolve(__dirname, '../../../firmware/include', file), 'utf-8')

const numbers = (text: string) => text.split(',').map(Number)

// kinematics.h's hexapodConfig, read from the source so the drawing cannot drift from the robot.
const firmwareConfig = (): HexapodConfig => {
  const match = firmware('kinematics.h').match(
    /hexapodConfig\s*=\s*\{\{([^}]*)\},\s*\{([^}]*)\},\s*\{([^}]*)\},\s*([^,]+),\s*([^,]+),\s*([^,]+),\s*([^}]+)\}/
  )
  if (!match) throw new Error('hexapodConfig not found in kinematics.h')
  const [, mountX, mountY, mountAngle, root, coxa, femur, tibia] = match
  return {
    legMountX: numbers(mountX),
    legMountY: numbers(mountY),
    legMountAngle: numbers(mountAngle),
    legRootToJoint1: Number(root),
    legJoint1ToJoint2: Number(coxa),
    legJoint2ToJoint3: Number(femur),
    legJoint3ToTip: Number(tibia)
  }
}

// servo_factory.h's factory calibration: centre 306, two PWM counts per degree, no centre angle.
const factoryServos = (): Servo[] => {
  const settings = firmware('settings/servo_factory.h')
  const match = settings.match(/FACTORY_DIRECTION\[FACTORY_SERVO_COUNT\]\s*=\s*\{([^}]*)\}/)
  if (!match) throw new Error('factory directions not found in servo_factory.h')
  expect(settings).toContain('FACTORY_SERVO_COUNT = 18;')
  expect(settings).toContain('FACTORY_CENTER_PWM = 306;')
  expect(settings).toContain('FACTORY_CONVERSION = 2;')
  expect(settings).toContain('FACTORY_CENTER_ANGLE = 0;')
  return numbers(match[1]).map((direction, i) => ({
    centerPwm: 306,
    direction,
    centerAngle: 0,
    conversion: 2,
    name: `Servo ${i}`
  }))
}

// ServoController's angle to PWM, unclamped: setAngles' offset, then toPwm.
const firmwarePwm = (servo: Servo, joint: number, angle: number) =>
  (angle + SERVO_ANGLE_OFFSET[joint] + servo.centerAngle) * servo.direction * servo.conversion +
  servo.centerPwm

const bodyAt = (feet: number[][]): body_state_t => ({
  omega: 0,
  phi: 0,
  psi: 0,
  xm: 0,
  ym: 0,
  zm: 0,
  feet,
  cumulative_x: 0,
  cumulative_y: 0,
  cumulative_z: 0,
  cumulative_roll: 0,
  cumulative_pitch: 0,
  cumulative_yaw: 0
})

const fwConfig = firmwareConfig()

const expectClose = (actual: number[], expected: number[]) =>
  expected.forEach((v, i) => expect(actual[i]).toBeCloseTo(v, 6))

describe('the firmware geometry', () => {
  it('is the geometry the app draws with', () => {
    expect(config).toEqual(fwConfig)
  })

  it('maps angles to PWM the way the page inverts', () => {
    const controller = firmware('peripherals/servo_controller.h')
    expect(controller).toContain(
      '(angle + cfg.centerAngle) * cfg.direction * cfg.conversion + cfg.centerPwm'
    )
    const offset = controller.match(/angle \+ \(joint == 2 \? ([\d.]+)f : 0\.0f\)/)
    expect(Number(offset?.[1])).toBe(SERVO_ANGLE_OFFSET[2])
    expect(SERVO_ANGLE_OFFSET.slice(0, 2)).toEqual([0, 0])
  })
})

describe('a leg drawn from joint angles', () => {
  let engine: AnimationEngine
  beforeAll(async () => {
    engine = await loadAnimationEngine()
  })

  const poses: LegAngles[] = [
    [0, 0, -90],
    [20, 15, -110],
    [-25, -30, -60],
    [10, 40, -130]
  ]

  it("puts the foot where the firmware's inverse kinematics aims it, on every leg", () => {
    for (let leg = 0; leg < 6; leg++) {
      for (const angles of poses) {
        const { foot } = legPoints(config, leg, angles)
        const feet = Array.from({ length: 6 }, () => [0, 0, 0, 1])
        feet[leg] = [...foot, 1]
        const solved = firmwareInverseKinematics(engine, bodyAt(feet))[leg]
        solved.forEach((rad, joint) => expect((rad * 180) / Math.PI).toBeCloseTo(angles[joint], 3))
      }
    }
  })

  it('keeps every link its firmware length', () => {
    const { mount, coxaAxis, femurAxis, knee, foot } = legPoints(config, 4, [20, 15, -110])
    const length = (a: number[], b: number[]) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2])
    expect(length(mount, coxaAxis)).toBeCloseTo(fwConfig.legRootToJoint1, 6)
    expect(length(coxaAxis, femurAxis)).toBeCloseTo(fwConfig.legJoint1ToJoint2, 6)
    expect(length(femurAxis, knee)).toBeCloseTo(fwConfig.legJoint2ToJoint3, 6)
    expect(length(knee, foot)).toBeCloseTo(fwConfig.legJoint3ToTip, 6)
  })

  it('reaches out level with the tibia hanging straight down at zero coxa and femur', () => {
    // Worked by hand from kinematics.h: the right front leg mounts at (44.82, 74.82) facing 45 degrees.
    const out = (38 + 54.06) * Math.SQRT1_2
    const { knee, foot } = legPoints(config, 0, [0, 0, -90])
    expectClose(knee, [44.82 + out, 74.82 + out, 0])
    expectClose(foot, [44.82 + out, 74.82 + out, -97])
    expectClose(legPlanePoints(config, [0, 0, -90]).foot, [38 + 54.06, -97])
  })

  it('reaches the left legs out to the left and the right legs out to the right', () => {
    const foot = (leg: number) => legPoints(config, leg, [0, 0, -90]).foot
    expectClose(foot(1), [61.03 + 92.06, 0, -97])
    expectClose(foot(4), [-61.03 - 92.06, 0, -97])
    expect(foot(3)[0]).toBeLessThan(0)
    expect(foot(3)[1]).toBeGreaterThan(0)
    expect(foot(5)[1]).toBeLessThan(0)
  })

  it('turns a positive coxa angle counter-clockwise seen from above on both sides', () => {
    // Counter-clockwise swings the right middle leg forwards and the left middle leg backwards.
    expect(legPoints(config, 1, [10, 0, -90]).foot[1]).toBeGreaterThan(0)
    expect(legPoints(config, 4, [10, 0, -90]).foot[1]).toBeLessThan(0)
  })

  it('raises the knee for a positive femur angle and folds the foot in for a smaller knee', () => {
    const level = legPlanePoints(config, [0, 0, -90])
    expect(legPlanePoints(config, [0, 20, -90]).knee[1]).toBeGreaterThan(0)
    expect(legPlanePoints(config, [0, 0, -120]).foot[0]).toBeLessThan(level.foot[0])
  })

  it('draws the side view as seen from behind: right legs outwards to the right', () => {
    expect([0, 1, 2, 3, 4, 5].map(leg => outwardSide(config, leg))).toEqual([1, 1, 1, -1, -1, -1])
    for (let leg = 0; leg < 6; leg++) {
      expect(Math.sign(legPoints(config, leg, [0, 0, -90]).foot[0])).toBe(outwardSide(config, leg))
    }
  })
})

describe('a calibration PWM read as a joint angle', () => {
  const servos = factoryServos()
  servos[7] = { ...servos[7], centerPwm: 290, centerAngle: -12.5, conversion: 2.2 }

  it("inverts the firmware's angle to PWM for every servo", () => {
    servos.forEach((servo, i) => {
      for (const angle of [-80, -15, 0, 30, 60]) {
        const pwm = firmwarePwm(servo, i % 3, angle)
        expect(jointAngleFromPwm(servo, i % 3, pwm)).toBeCloseTo(angle, 6)
      }
    })
  })

  it('puts each joint at its reference angle at its centre PWM', () => {
    servos.forEach((servo, i) => {
      expect(firmwarePwm(servo, i % 3, referenceAngle(servo, i % 3))).toBeCloseTo(
        servo.centerPwm,
        6
      )
    })
    expect(referenceAngle(servos[7], 1)).toBeCloseTo(12.5, 6)
  })

  it('holds the factory reference pose: coxa straight out, femur level, knee square', () => {
    const factory = factoryServos()
    for (let leg = 0; leg < 6; leg++) {
      const [coxa, femur, tibia] = referenceLeg(factory, leg)
      expect(coxa).toBeCloseTo(0, 6)
      expect(femur).toBeCloseTo(0, 6)
      expect(kneeAngle(tibia)).toBeCloseTo(90, 6)
    }
  })

  it('moves mirrored femurs opposite ways as the PWM rises', () => {
    // Factory directions: right femurs +1, left femurs -1, so the same PWM rise lifts one and drops the other.
    const rise = (servoId: number) =>
      jointAngleFromPwm(servos[servoId], 1, 326) - jointAngleFromPwm(servos[servoId], 1, 306)
    expect(rise(1)).toBeCloseTo(10, 6)
    expect(rise(10)).toBeCloseTo(-10, 6)
  })
})

describe('the leg shown while calibrating', () => {
  const servos = factoryServos()

  it('names servos in the firmware order', () => {
    const named = (id: number) => {
      const { leg, joint } = servoJoint(id)
      return `${LEG_NAMES[leg]} ${JOINT_NAMES[joint]}`
    }
    expect(named(0)).toBe('Right front Coxa')
    expect(named(4)).toBe('Right middle Femur')
    expect(named(17)).toBe('Left rear Tibia')
  })

  it('moves only the calibrated joint away from the reference pose', () => {
    expectClose(calibrationLeg(servos, 13, 346), [0, -20, -90])
    expectClose(calibrationLeg(servos, 14, 306), referenceLeg(servos, 4))
    expectClose(calibrationLeg(servos, 2, 346), [0, 0, -110])
  })
})
