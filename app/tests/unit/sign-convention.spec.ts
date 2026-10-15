// @vitest-environment node
// The app moves the robot with the firmware's own motion code (the virtual robot), so its side of the shared sign
// convention is checked there, from the stick on: each fixture stick is walked through the firmware's command
// handling and gait, and the feet the forward kinematics places from its joint angles give the signs.
// Needs the virtual robot built (scripts/build_virtual_robot.js).
import { beforeAll, describe, expect, it } from 'vitest'
import { readFileSync } from 'node:fs'
import { resolve } from 'node:path'
import Kinematics, { type body_state_t } from '../../src/lib/kinematic'
import { config } from '../../src/lib/components/config'
import { MotionModes } from '../../src/lib/motion'
import { loadVirtualRobot, type VirtualRobot } from '../../src/lib/simulation/virtual-robot'

type Command = {
  name: string
  stick: { lx: number; ly: number; rx: number }
  body_velocity_sign: number[]
  yaw_sign: number
  stance_foot_displacement_sign: number[][]
}

// The same file the firmware's host test (firmware/test/test_sign_convention) and the simulation's
// test_sign_convention.py read; docs/conventions.md derives the signs.
const fixture: {
  legs: string[]
  stance_mm: number[][]
  sampling: { dt_s: number; ticks: number; zero_mm_per_tick: number; zero_rad_per_tick: number }
  commands: Command[]
} = JSON.parse(
  readFileSync(resolve(__dirname, '../../../platform_shared/sign_convention.json'), 'utf8')
)
const { sampling } = fixture
// A middle leg's resting y is mountY + reach * sin(mount angle), which floating point leaves near 0
// rather than at it; a leg's side is decided well clear of that.
const SIDE_ZERO_MM = 1
// A planted foot rests on the ground the lowest feet span; a swinging one is lifted well clear of it.
const STANCE_BAND_MM = 1
// The command ramp and the feet's ease from a stand settle well within this before sampling starts.
const SETTLE_TICKS = 300
const STAND = Object.values(MotionModes).indexOf(MotionModes.STAND)
const WALK = Object.values(MotionModes).indexOf(MotionModes.WALK)

const kinematics = new Kinematics(config)
const level: body_state_t = {
  omega: 0,
  phi: 0,
  psi: 0,
  xm: 0,
  ym: 0,
  zm: 0,
  feet: [],
  cumulative_x: 0,
  cumulative_y: 0,
  cumulative_z: 0,
  cumulative_roll: 0,
  cumulative_pitch: 0,
  cumulative_yaw: 0
}

const sign = (value: number, zero: number) =>
  value > zero ? 1
  : value < -zero ? -1
  : 0

let robot: VirtualRobot

// The feet in the body frame (mm) for the angles the firmware drives the servos to.
const feet = () => {
  const deg = robot.servoAngles()
  const legs = [0, 1, 2, 3, 4, 5].map(
    leg => [0, 1, 2].map(j => (deg[leg * 3 + j] * Math.PI) / 180) as [number, number, number]
  )
  return kinematics.forwardKinematics(level, legs).map(([x, y, z]) => [x, y, z])
}

const restingStance = () => {
  robot.standPose()
  robot.setMode(STAND)
  robot.tick()
  return feet()
}

// Walks the stick from a stand. A planted foot moves against the body, so v_body = -mean(v_foot), and the yaw rate
// is the least-squares rotation of the feet about the body centre, negated.
const walk = (command: Command) => {
  const resting = restingStance()
  robot.setMode(WALK)
  const { lx, ly, rx } = command.stick
  const step = () => {
    robot.command([lx, ly, rx, 0, 0, 0, 0, 0])
    robot.tick()
    return feet()
  }
  for (let tick = 0; tick < SETTLE_TICKS; tick++) step()

  const planted = (f: number[][]) => {
    const ground = Math.min(...f.map(foot => foot[2]))
    return f.map(foot => foot[2] <= ground + STANCE_BAND_MM)
  }
  const total = resting.map(() => [0, 0])
  const ticks = resting.map(() => 0)
  let before = feet()
  for (let tick = 0; tick < sampling.ticks; tick++) {
    const after = step()
    const was = planted(before)
    const is = planted(after)
    for (let i = 0; i < 6; i++) {
      if (!was[i] || !is[i]) continue
      total[i][0] += after[i][0] - before[i][0]
      total[i][1] += after[i][1] - before[i][1]
      ticks[i]++
    }
    before = after
  }

  const foot = total.map(([x, y], i) => [x / Math.max(ticks[i], 1), y / Math.max(ticks[i], 1)])
  const bodyVelocity = [0, 1].map(axis => -foot.reduce((sum, f) => sum + f[axis], 0) / 6)
  const cross = foot.reduce((sum, [dx, dy], i) => sum + resting[i][0] * dy - resting[i][1] * dx, 0)
  const radiusSquared = resting.reduce((sum, [x, y]) => sum + x * x + y * y, 0)
  return { foot, ticks, bodyVelocity, yaw: -cross / radiusSquared }
}

describe('sign convention', () => {
  beforeAll(async () => {
    robot = await loadVirtualRobot()
  })

  it('reads three commands for six legs', () => {
    expect(fixture.commands).toHaveLength(3)
    expect(fixture.legs).toHaveLength(6)
  })

  it("puts each leg of the firmware's resting stance on its fixture side", () => {
    const resting = restingStance()
    fixture.legs.forEach((leg, i) => {
      expect(
        resting[i].slice(0, 2).map(v => sign(v, SIDE_ZERO_MM)),
        leg
      ).toEqual(fixture.stance_mm[i].slice(0, 2).map(v => sign(v, 0)))
    })
  })

  describe.each(fixture.commands)('$name', command => {
    it('moves each stance foot with the fixture signs', () => {
      const { foot, ticks } = walk(command)
      fixture.legs.forEach((leg, i) => {
        expect(ticks[i], leg).toBeGreaterThan(0)
        expect(
          foot[i].map(v => sign(v, sampling.zero_mm_per_tick)),
          leg
        ).toEqual(command.stance_foot_displacement_sign[i])
      })
    })

    it('implies the fixture body velocity and yaw signs', () => {
      const { bodyVelocity, yaw } = walk(command)
      expect(bodyVelocity.map(v => sign(v, sampling.zero_mm_per_tick))).toEqual(
        command.body_velocity_sign
      )
      expect(sign(yaw, sampling.zero_rad_per_tick)).toBe(command.yaw_sign)
    })
  })
})
