// The 3D view draws only joint angles (the robot's telemetry or the firmware's own code) and sets the model on the
// ground under the feet they place. Driven by the firmware's inverse kinematics for known poses, the planted feet must
// land on the floor, a lifted one above it, and none below it.
import { beforeAll, describe, expect, it } from 'vitest'
import { Vector3 } from 'three'
import Kinematics, { levelBody, type body_state_t } from '../../src/lib/kinematic'
import { config } from '../../src/lib/components/config'
import { DEFAULT_FEET } from '../../src/lib/animation/model'
import { stanceToScene } from '../../src/lib/animation/handles'
import {
  groundPlacement,
  imuPlacement,
  ShownRobot,
  type Placement
} from '../../src/lib/ground-placement'
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

// The connected robot is drawn tilted as its IMU measures, from IMUData's DMP roll and pitch, and lowered until its
// lowest foot touches the floor. The readings are the robot's own, from docs/conventions.md: nose down 12 deg read
// roll -12.0 deg, right side down 17.7 deg read pitch -17.7 deg.
describe('the connected robot on its IMU', () => {
  const NOSE_DOWN = { x: (-12 * Math.PI) / 180, y: 0, z: 0 }
  const RIGHT_SIDE_DOWN = { x: 0, y: (-17.7 * Math.PI) / 180, z: 0 }
  const DEG = Math.PI / 180

  beforeAll(async () => {
    engine = await loadAnimationEngine()
  })

  const feetOf = (pose: body_state_t) =>
    kinematics.forwardKinematics(levelBody(), firmwareInverseKinematics(engine, pose))
  const levelFeet = () => feetOf(body({}))
  // How far a body-frame segment from `low` to `high` (mm) rises in the drawn scene, in degrees.
  const rise = (placement: Placement, low: number[], high: number[]) => {
    const a = placed(placement, low)
    const b = placed(placement, high)
    return Math.asin((b.y - a.y) / a.distanceTo(b)) / DEG
  }

  it('draws the nose 12 deg down for the nose-down reading', () => {
    const placement = imuPlacement(NOSE_DOWN, levelFeet())
    expect(rise(placement, [0, 100, 0], [0, -100, 0])).toBeCloseTo(12, 3)
    expect(Math.abs(rise(placement, [-100, 0, 0], [100, 0, 0]))).toBeLessThan(1e-6)
  })

  it('draws the right side 17.7 deg down for the right-side-down reading', () => {
    const placement = imuPlacement(RIGHT_SIDE_DOWN, levelFeet())
    expect(rise(placement, [100, 0, 0], [-100, 0, 0])).toBeCloseTo(17.7, 3)
    expect(Math.abs(rise(placement, [0, -100, 0], [0, 100, 0]))).toBeLessThan(1e-6)
  })

  it('takes no yaw from the IMU', () => {
    const still = imuPlacement(NOSE_DOWN, levelFeet())
    const turned = imuPlacement({ ...NOSE_DOWN, z: 1.2 }, levelFeet())
    expect(turned.orientation.angleTo(still.orientation)).toBeLessThan(1e-6)
  })

  it.each([
    { pose: {}, imu: { x: 0, y: 0, z: 0 } },
    { pose: { omega: 0.15, phi: -0.1, zm: 10 }, imu: RIGHT_SIDE_DOWN },
    { pose: { omega: -0.1, xm: 20 }, imu: NOSE_DOWN }
  ])('stands its lowest foot on the floor for a body at $pose', ({ pose, imu }) => {
    const feet = feetOf(body(pose))
    const placement = imuPlacement(imu, feet)
    const heights = feet.map(foot => placed(placement, foot).y)
    expect(Math.abs(Math.min(...heights))).toBeLessThan(ON_THE_FLOOR)
  })

  it('keeps the body over the same spot of the floor whatever the tilt', () => {
    for (const imu of [NOSE_DOWN, RIGHT_SIDE_DOWN]) {
      const origin = placed(imuPlacement(imu, levelFeet()), [0, 0, 0])
      expect(Math.hypot(origin.x, origin.z)).toBeLessThan(ON_THE_FLOOR)
    }
  })

  it('takes the tilt from the IMU, not from the ground the legs imply', () => {
    const onTiltedGround = feetOf(body({ omega: 0.2, phi: -0.15 }))
    const fromLegs = groundPlacement(
      kinematics.groundUnderFeet(
        firmwareInverseKinematics(engine, body({ omega: 0.2, phi: -0.15 }))
      ),
      0
    )
    const level = imuPlacement(NOSE_DOWN, levelFeet())
    const tilted = imuPlacement(NOSE_DOWN, onTiltedGround)
    expect(tilted.orientation.angleTo(level.orientation)).toBeLessThan(1e-6)
    expect(fromLegs.orientation.angleTo(level.orientation)).toBeGreaterThan(0.05)
  })
})

// The view draws whatever angles it has, the robot's or the virtual robot's, and places them by the link: a linked
// robot by its IMU, whether or not it has sent angles yet (it sends none in DEACTIVATED, IDLE and POSE), and only an
// unlinked robot and the animation preview on the ground under their feet.
describe('the view places the robot by its link', () => {
  const NOSE_DOWN = { x: (-12 * Math.PI) / 180, y: 0, z: 0 }

  beforeAll(async () => {
    engine = await loadAnimationEngine()
  })

  const stand = () => firmwareInverseKinematics(engine, body({}))
  const onTiltedGround = () => firmwareInverseKinematics(engine, body({ omega: 0.2, phi: -0.15 }))

  it('tilts a linked robot by its IMU before it has sent any angles', () => {
    const view = new ShownRobot(kinematics)
    view.show(stand())
    view.link(true)
    view.measure(NOSE_DOWN)
    const expected = imuPlacement(NOSE_DOWN, kinematics.forwardKinematics(levelBody(), stand()))
    const placement = view.placement(null)
    expect(placement.orientation.angleTo(expected.orientation)).toBeLessThan(1e-6)
    expect(placement.position.distanceTo(expected.position)).toBeLessThan(ON_THE_FLOOR)
    expect(placement.orientation.angleTo(LEVEL)).toBeCloseTo((12 * Math.PI) / 180, 6)
  })

  it('stands an unlinked robot on the ground under its feet, whatever the last reading', () => {
    const view = new ShownRobot(kinematics)
    view.link(true)
    view.measure(NOSE_DOWN)
    view.link(false)
    view.show(onTiltedGround())
    const expected = groundPlacement(kinematics.groundUnderFeet(onTiltedGround()), 0)
    expect(view.placement(null).orientation.angleTo(expected.orientation)).toBeLessThan(1e-6)
  })

  it('stands the animation preview on the ground under its feet while linked', () => {
    const view = new ShownRobot(kinematics)
    view.link(true)
    view.measure(NOSE_DOWN)
    view.show(stand())
    const preview = { legs: onTiltedGround(), yaw: 0.3 }
    const expected = groundPlacement(kinematics.groundUnderFeet(preview.legs), preview.yaw)
    expect(view.placement(preview).orientation.angleTo(expected.orientation)).toBeLessThan(1e-6)
  })
})
