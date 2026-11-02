// A joint of the 3D model dragged to drive the robot's servo (src/lib/joint-drive.ts): only the controller page's view,
// only in IDLE, only over a link and not while the controls drive the simulation view; leaving any of those stops it.
import { readFileSync } from 'node:fs'
import { resolve } from 'node:path'
import { beforeAll, describe, expect, it } from 'vitest'
import { writable } from 'svelte/store'
import { MotionModes, urdfJointOrder } from '$lib/motion'
import type { JointAngleData } from '$lib/platform_shared/message'
import {
  JOINT_LIMIT_DEG,
  JointDrive,
  draggedPose,
  drivesJoints,
  servoOfUrdfJoint,
  withinTravel
} from '$lib/joint-drive'
import { loadVirtualRobot, type VirtualRobot } from '$lib/simulation/virtual-robot'

const SERVOS = Array.from({ length: 18 }, (_, servo) => servo)

const rad = (deg: number) => (deg * Math.PI) / 180

// kinematics.h's JOINT_LIMIT_DEG, read from the source: each leg's travel, coxa, femur and tibia, that ServoController
// holds every joint within at the factory calibration.
const firmwareLimits = () => {
  const header = readFileSync(resolve(__dirname, '../../../firmware/include/kinematics.h'), 'utf-8')
  const match = header.match(/JOINT_LIMIT_DEG\[6\]\[3\]\s*=\s*\{([\s\S]*?)\};/)
  if (!match) throw new Error('JOINT_LIMIT_DEG not found in kinematics.h')
  return [...match[1].matchAll(/\{([^{}]*)\}/g)].map(row =>
    row[1].split(',').map(value => parseFloat(value))
  )
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
      const servo = servoOfUrdfJoint(joint)
      const limit = limits[Math.floor(servo / 3)][servo % 3]
      expect(withinTravel(joint, rad(limit + 20))).toBeCloseTo(rad(limit), 10)
      expect(withinTravel(joint, rad(-limit - 20))).toBeCloseTo(rad(-limit), 10)
      expect(withinTravel(joint, rad(limit - 1))).toBe(rad(limit - 1))
    }
  })

  it("stops a femur at the firmware's -90 degrees, not at the model's -152.8", () => {
    const femur = urdfJointOrder(SERVOS).indexOf(1)
    expect(withinTravel(femur, rad(-152.8))).toBeCloseTo(rad(-90), 10)
  })

  // urdf-loader clamps a joint to the model's limits, so a narrower one would draw a joint short of where the robot holds
  // it, as the crab's corner coxas at 45 degrees, and stop a dragged joint short of the robot's travel.
  it("lets every joint of the model, static/model.xacro's shoulder, leg and foot, reach the firmware's travel", () => {
    const xacro = readFileSync(resolve(__dirname, '../../static/model.xacro'), 'utf-8')
    const property = (name: string) => {
      const value = xacro.match(new RegExp(`<xacro:property name="${name}" value="([^"]+)"`))
      if (!value) throw new Error(`model.xacro has no property ${name}`)
      return Number(value[1])
    }
    // Each leg's shoulder takes its coxa_limit, at least its own firmware leg's: the legs are declared in the URDF
    // model's joint order, three joints each, so the n-th one's shoulder is URDF joint 3n.
    const legs = [
      ...xacro.matchAll(/<xacro:gen_full_leg_joint pos="(\w+)"[^>]*coxa_limit="\$\{(\w+)\}"/g)
    ]
    expect(legs).toHaveLength(6)
    legs.forEach(([, pos, name], n) => {
      const firmwareLeg = Math.floor(servoOfUrdfJoint(3 * n) / 3)
      expect(property(name), `${pos} shoulder`).toBeGreaterThanOrEqual(
        rad(JOINT_LIMIT_DEG[firmwareLeg][0])
      )
    })
    expect(xacro).toMatch(/<limit[^>]*lower="\$\{-coxa_limit\}" upper="\$\{coxa_limit\}"/)
    ;['leg', 'foot'].forEach((joint, j) => {
      const limit = xacro.match(
        new RegExp(
          String.raw`name="\$\{pos\}_${joint}"[\s\S]*?<limit[^>]*lower="([^"]+)"[^>]*upper="([^"]+)"`
        )
      )
      if (!limit) throw new Error(`the ${joint} joint has no limit in model.xacro`)
      expect(-Number(limit[1]), `${joint} lower`).toBeGreaterThanOrEqual(
        rad(JOINT_LIMIT_DEG[0][j + 1])
      )
      expect(Number(limit[2]), `${joint} upper`).toBeGreaterThanOrEqual(
        rad(JOINT_LIMIT_DEG[0][j + 1])
      )
    })
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

// platform_shared/leg_clearance_fixtures.txt, which the firmware's native test and the simulation pass too.
const clearanceFixtures = () =>
  readFileSync(resolve(__dirname, '../../../platform_shared/leg_clearance_fixtures.txt'), 'utf-8')
    .split('\n')
    .map(line => line.trim().split(/\s+/))
    .filter(words => words[0] === 'case')
    .map(([, name, , , clear, , ...angles]) => ({
      name,
      clear: clear === '1',
      angles: angles.map(Number)
    }))

const urdfJointOf = (servo: number) => urdfJointOrder(SERVOS).indexOf(servo)

describe('the clearance a drag is judged by', () => {
  let robot: VirtualRobot
  let keepsLegsApart: VirtualRobot['keepsLegsApart']

  beforeAll(async () => {
    robot = await loadVirtualRobot()
    keepsLegsApart = robot.keepsLegsApart
  })

  const named = (name: string) => clearanceFixtures().find(c => c.name === name)!.angles

  it("is the firmware's own check, leg_clearance.h, on every shared fixture", () => {
    // From the clear stance, a move keeps the legs apart exactly when the pose moved to keeps the clearance.
    const cases = clearanceFixtures()
    expect(cases.length).toBeGreaterThan(20)
    for (const { name, clear, angles } of cases)
      expect(keepsLegsApart(named('stance'), angles), name).toBe(clear)
  })

  it('refuses a drag that would bring two legs together and passes one that keeps them apart', () => {
    // Legs 0 and 1 meet with both coxas turned about 38.7 deg toward each other; one turned 40 deg alone stays apart.
    const stand = Array.from(robot.standPose())
    const leg1Turned = draggedPose(stand, urdfJointOf(3), rad(40), keepsLegsApart)
    expect(leg1Turned?.[3]).toBeCloseTo(40, 4)
    expect(draggedPose(leg1Turned!, urdfJointOf(0), rad(-40), keepsLegsApart)).toBeNull()
    const both = draggedPose(leg1Turned!, urdfJointOf(0), rad(-20), keepsLegsApart)
    expect(both?.[0]).toBeCloseTo(-20, 4)
    expect(both?.filter((_, servo) => servo !== 0)).toEqual(
      leg1Turned!.filter((_, servo) => servo !== 0)
    )
  })

  it('from legs already through each other, passes a drag that parts them and refuses one that closes them', () => {
    const touching = named('touching_0_1')
    const coxa0 = touching[0]
    expect(draggedPose(touching, urdfJointOf(0), rad(coxa0 + 2), keepsLegsApart)?.[0]).toBeCloseTo(
      coxa0 + 2,
      4
    )
    expect(draggedPose(touching, urdfJointOf(0), rad(coxa0 - 2), keepsLegsApart)).toBeNull()
  })

  it('judges every leg pair on its own, so legs through each other let no other pair come together', () => {
    // Legs 0 and 1 through each other, leg 4's coxa turned 40 deg toward leg 3: leg 3's turned 40 deg toward leg 4 would
    // bring that pair through each other too (they meet at about 38.7 deg each).
    const from = [...named('through_0_1')]
    const toward = named('touching_3_4')
    from[12] = Math.sign(toward[12]) * 40
    expect(
      draggedPose(from, urdfJointOf(9), rad(Math.sign(toward[9]) * 40), keepsLegsApart)
    ).toBeNull()
    expect(
      draggedPose(from, urdfJointOf(9), rad(Math.sign(toward[9]) * 10), keepsLegsApart)?.[9]
    ).toBeCloseTo(Math.sign(toward[9]) * 10, 4)
  })

  it('judges the dragged joint within its travel, as the robot holds it', () => {
    const stand = Array.from(robot.standPose())
    expect(draggedPose(stand, urdfJointOf(3), rad(80), keepsLegsApart)?.[3]).toBeCloseTo(
      JOINT_LIMIT_DEG[1][0],
      4
    )
  })
})
