// A joint of the 3D model dragged to drive the robot's servo (src/lib/joint-drive.ts): only the controller page's view,
// only in IDLE, only over a link and not while the controls drive the simulation view; leaving any of those stops it.
import { readFileSync } from 'node:fs'
import { resolve } from 'node:path'
import { describe, expect, it } from 'vitest'
import { writable } from 'svelte/store'
import { MotionModes, urdfJointOrder } from '$lib/motion'
import type { JointAngleData } from '$lib/platform_shared/message'
import {
  JOINT_LIMIT_DEG,
  JointDrive,
  drivesJoints,
  servoOfUrdfJoint,
  withinTravel
} from '$lib/joint-drive'

const SERVOS = Array.from({ length: 18 }, (_, servo) => servo)

const rad = (deg: number) => (deg * Math.PI) / 180

// kinematics.h's JOINT_LIMIT_DEG, read from the source: the travel ServoController holds every joint within.
const firmwareLimits = () => {
  const header = readFileSync(resolve(__dirname, '../../../firmware/include/kinematics.h'), 'utf-8')
  const match = header.match(/JOINT_LIMIT_DEG\[3\]\s*=\s*\{([^}]*)\}/)
  if (!match) throw new Error('JOINT_LIMIT_DEG not found in kinematics.h')
  return match[1].split(',').map(value => parseFloat(value))
}

const driver = (driving: boolean) => {
  const state = writable(driving)
  const sent: JointAngleData[] = []
  const drive = new JointDrive(state, joint => {
    sent.push(joint)
    return true
  })
  return { state, sent, drive }
}

describe('servoOfUrdfJoint', () => {
  it("is the servo whose angle the 3D view draws on that joint, urdfJointOrder's inverse", () => {
    const drawn = urdfJointOrder(SERVOS)
    for (let joint = 0; joint < 18; joint++) expect(servoOfUrdfJoint(joint)).toBe(drawn[joint])
    expect(new Set(SERVOS.map(servoOfUrdfJoint)).size).toBe(18)
  })
})

describe('withinTravel', () => {
  it("is the firmware's travel, kinematics.h JOINT_LIMIT_DEG", () =>
    expect(JOINT_LIMIT_DEG).toEqual(firmwareLimits()))

  it('holds each joint of the model within the travel of the servo it drives', () => {
    const limits = firmwareLimits()
    for (let joint = 0; joint < 18; joint++) {
      const limit = limits[servoOfUrdfJoint(joint) % 3]
      expect(withinTravel(joint, rad(limit + 20))).toBeCloseTo(rad(limit), 10)
      expect(withinTravel(joint, rad(-limit - 20))).toBeCloseTo(rad(-limit), 10)
      expect(withinTravel(joint, rad(limit - 1))).toBe(rad(limit - 1))
    }
  })

  it("stops a femur at the firmware's -90 degrees, not at the model's -152.8", () => {
    const femur = urdfJointOrder(SERVOS).indexOf(1)
    expect(withinTravel(femur, rad(-152.8))).toBeCloseTo(rad(-90), 10)
  })
})

describe('drivesJoints', () => {
  const live = { controllerView: true, linked: true, simulation: false }

  it("drives from the controller page's view in IDLE over a link", () =>
    expect(drivesJoints({ ...live, mode: MotionModes.IDLE })).toBe(true))

  it('drives in no other mode', () => {
    for (const mode of Object.values(MotionModes).filter(m => m !== MotionModes.IDLE))
      expect(drivesJoints({ ...live, mode })).toBe(false)
  })

  it('never drives from any other 3D view, whatever the mode', () => {
    for (const mode of Object.values(MotionModes))
      expect(drivesJoints({ ...live, controllerView: false, mode })).toBe(false)
  })

  it('drives nothing without a link or while the controls drive the simulation view', () => {
    expect(drivesJoints({ ...live, linked: false, mode: MotionModes.IDLE })).toBe(false)
    expect(drivesJoints({ ...live, simulation: true, mode: MotionModes.IDLE })).toBe(false)
  })
})

describe('JointDrive', () => {
  it("sends a dragged joint as its servo's angle in degrees while driving", () => {
    const { sent, drive } = driver(true)
    expect(drive.drag(0, Math.PI / 6)).toBe(true)
    expect(sent).toHaveLength(1)
    expect(sent[0].servo).toBe(servoOfUrdfJoint(0))
    expect(sent[0].degrees).toBeCloseTo(30, 10)
  })

  it('sends nothing while not driving, and stops the moment driving ends', () => {
    const { state, sent, drive } = driver(false)
    expect(drive.drag(4, 0.2)).toBe(false)
    expect(sent).toHaveLength(0)

    state.set(true)
    drive.drag(4, 0.2)
    state.set(false)
    expect(drive.drag(4, 0.3)).toBe(false)
    expect(sent.map(joint => joint.servo)).toEqual([servoOfUrdfJoint(4)])
  })

  it('follows the driving state no longer once disposed', () => {
    const { state, sent, drive } = driver(false)
    drive.dispose()
    state.set(true)
    expect(drive.drag(1, 0.1)).toBe(false)
    expect(sent).toHaveLength(0)
  })
})
