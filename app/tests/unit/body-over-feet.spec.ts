import { beforeAll, describe, expect, it } from 'vitest'
import Kinematics, { type body_state_t } from '../../src/lib/kinematic'
import { config } from '../../src/lib/components/config'
import { DEFAULT_FEET } from '../../src/lib/animation/model'
import { loadAnimationEngine, type AnimationEngine } from '../../src/lib/animation/engine'
import { firmwareInverseKinematics } from './firmware-ik'

const kinematics = new Kinematics(config)
let engine: AnimationEngine
const STAND_Z = DEFAULT_FEET[0][2]
// The firmware solves the angles in float32, which leaves the recovered pose within 2e-5 rad and 1e-5 mm.
const DECIMALS = 4

const body = (pose: Partial<body_state_t>, feet = DEFAULT_FEET.map(f => [...f])): body_state_t => ({
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
  cumulative_yaw: 0,
  ...pose
})

// The joint angles the robot drives for these feet: the firmware's inverse kinematics.
const angles = (b: body_state_t) => firmwareInverseKinematics(engine, b)

// The inverse kinematics saturates on a foot out of reach, and then the forward kinematics does not return the foot.
const reachable = (b: body_state_t) =>
  kinematics
    .forwardKinematics(b, angles(b))
    .every((foot, leg) => [0, 1, 2].every(axis => Math.abs(foot[axis] - b.feet[leg][axis]) < 1e-3))

// The 3D view shows only joint angles (the robot's telemetry or the firmware's code), so it recovers the body's pose
// over the ground from them: the inverse kinematics of a known pose must come back as that pose.
describe('the body over its feet', () => {
  beforeAll(async () => {
    engine = await loadAnimationEngine()
  })

  it.each([
    { omega: 0, phi: 0, zm: 0 },
    { omega: 0.2, phi: 0, zm: 0 },
    { omega: 0, phi: -0.2, zm: 0 },
    { omega: -0.15, phi: 0.1, zm: 12 },
    { omega: 0.2, phi: 0.2, zm: -10 }
  ])('recovers omega $omega, phi $phi and zm $zm', pose => {
    expect(reachable(body(pose))).toBe(true)
    const found = kinematics.bodyOverFeet(angles(body(pose)), STAND_Z)
    expect(found.omega).toBeCloseTo(pose.omega, DECIMALS)
    expect(found.phi).toBeCloseTo(pose.phi, DECIMALS)
    expect(found.zm).toBeCloseTo(pose.zm, DECIMALS)
  })

  it('stands on the planted feet, not on a foot lifted in its swing', () => {
    const feet = DEFAULT_FEET.map(f => [...f])
    feet[0][2] += 30
    feet[2][2] += 30
    feet[4][2] += 30
    const pose = { omega: 0.1, phi: -0.15, zm: 5 }
    const found = kinematics.bodyOverFeet(angles(body(pose, feet)), STAND_Z)
    expect(found.omega).toBeCloseTo(pose.omega, DECIMALS)
    expect(found.phi).toBeCloseTo(pose.phi, DECIMALS)
    expect(found.zm).toBeCloseTo(pose.zm, DECIMALS)
  })

  it('keeps the tilt of a body shifted over its feet', () => {
    const pose = { omega: 0.1, phi: 0.1, xm: 25, ym: -20 }
    const found = kinematics.bodyOverFeet(angles(body(pose)), STAND_Z)
    expect(found.omega).toBeCloseTo(pose.omega, DECIMALS)
    expect(found.phi).toBeCloseTo(pose.phi, DECIMALS)
  })
})
