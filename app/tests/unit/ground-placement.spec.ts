// The 3D view draws only joint angles (the robot's telemetry or the firmware's own code) and sets the model on the
// ground under the feet they place. Driven by the firmware's inverse kinematics for known poses, the planted feet must
// land on the floor, a lifted one above it, and none below it.
import { beforeAll, describe, expect, it } from 'vitest'
import { Vector3 } from 'three'
import Kinematics, { levelBody, type body_state_t } from '../../src/lib/kinematic'
import { config } from '../../src/lib/components/config'
import { DEFAULT_FEET } from '../../src/lib/animation/model'
import { stanceToScene } from '../../src/lib/animation/handles'
import { groundPlacement, type Placement } from '../../src/lib/ground-placement'
import { loadAnimationEngine, type AnimationEngine } from '../../src/lib/animation/engine'
import { firmwareInverseKinematics } from './firmware-ik'

const MM_PER_UNIT = 12
// A tenth of a millimetre, in scene units.
const ON_THE_FLOOR = 0.1 / MM_PER_UNIT

const kinematics = new Kinematics(config)
const LEVEL = groundPlacement(null, 0).orientation
let engine: AnimationEngine

// A body-frame point (mm) of a robot placed so, in the scene: stanceToScene is where a level robot at the origin draws
// it, which the placement turns and moves.
const placed = ({ orientation, position }: Placement, mm: readonly number[]) =>
  new Vector3(...stanceToScene(mm))
    .applyQuaternion(LEVEL.clone().invert())
    .applyQuaternion(orientation)
    .add(position)

const body = (pose: Partial<body_state_t>, feet = DEFAULT_FEET.map(f => [...f])): body_state_t => ({
  ...levelBody(),
  feet,
  ...pose
})

// The feet the shown joint angles put in the body frame, and where the robot placed on their ground draws them.
const drawn = (pose: body_state_t, yaw = 0) => {
  const legs = firmwareInverseKinematics(engine, pose)
  const ground = kinematics.groundUnderFeet(legs)
  const placement = groundPlacement(ground, yaw)
  const feet = kinematics.forwardKinematics(levelBody(), legs).map(foot => placed(placement, foot))
  return { ground, feet }
}

describe('the robot on its ground', () => {
  beforeAll(async () => {
    engine = await loadAnimationEngine()
  })

  it.each([
    { omega: 0.2, phi: 0, zm: 0 },
    { omega: 0, phi: -0.2, zm: 0 },
    { omega: -0.15, phi: 0.1, zm: 12 },
    { omega: 0.2, phi: 0.2, zm: -10 },
    { omega: 0.1, phi: -0.1, xm: 25, ym: -20 }
  ])('stands every foot on the floor under a body at %o', pose => {
    const { ground, feet } = drawn(body(pose))
    expect(ground.planted).toHaveLength(6)
    for (const foot of feet) expect(Math.abs(foot.y)).toBeLessThan(ON_THE_FLOOR)
  })

  it('stands on the planted feet and holds a swinging one above the floor', () => {
    const feet = DEFAULT_FEET.map(f => [...f])
    for (const leg of [0, 2, 4]) feet[leg][2] += 30
    const { ground, feet: shown } = drawn(body({ omega: 0.1, phi: -0.15, zm: 5 }, feet))
    expect(ground.planted).toHaveLength(3)
    for (const leg of [1, 3, 5]) expect(Math.abs(shown[leg].y)).toBeLessThan(ON_THE_FLOOR)
    for (const leg of [0, 2, 4]) expect(shown[leg].y).toBeCloseTo(30 / MM_PER_UNIT, 4)
  })

  it('turns the whole robot by a yaw without lifting a foot', () => {
    const { feet } = drawn(body({ omega: 0.15, phi: 0.1 }), 0.4)
    for (const foot of feet) expect(Math.abs(foot.y)).toBeLessThan(ON_THE_FLOOR)
  })

  it('draws a level stance where the foot handles stand', () => {
    const { feet } = drawn(body({}))
    feet.forEach((foot, leg) => {
      const handle = stanceToScene(DEFAULT_FEET[leg])
      expect(foot.distanceTo(new Vector3(...handle))).toBeLessThan(ON_THE_FLOOR)
    })
  })
})
