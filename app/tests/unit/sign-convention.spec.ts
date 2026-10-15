import { describe, expect, it } from 'vitest'
import { readFileSync } from 'node:fs'
import { resolve } from 'node:path'
import { default_offset, default_stand_frac, GaitType } from '../../src/lib/gait'
import type { gait_state_t } from '../../src/lib/gait'
import type { body_state_t } from '../../src/lib/kinematic'
import Motion, { MotionModes } from '../../src/lib/motion'

type Command = {
  name: string
  stick: { lx: number; ly: number; rx: number }
  gait: { step_x: number; step_y: number; step_angle: number }
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

const sign = (value: number, zero: number) =>
  value > zero ? 1
  : value < -zero ? -1
  : 0

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

// The app's own resting stance and gait controller, as the 3D view builds them.
const appStance = () => {
  const motion = new Motion()
  return { controller: motion.gait, resting: motion.defaultPosition.map(foot => [...foot]) }
}

// Walks the command on the app's resting stance. A planted foot moves against the body, so
// v_body = -mean(v_foot), and the yaw rate is the least-squares rotation of the feet about the body
// centre, negated.
const walk = (command: Command) => {
  const { controller, resting } = appStance()
  const gait: gait_state_t = {
    ...command.gait,
    step_height: 40,
    step_speed: 1,
    step_depth: 0.002,
    stand_frac: default_stand_frac[GaitType.TRI_GATE],
    offset: default_offset[GaitType.TRI_GATE],
    gait_type: GaitType.TRI_GATE
  }
  const body = bodyAt(resting.map(foot => [...foot]))
  controller.step(gait, body, sampling.dt_s)

  const inStance = (phase: number, leg: number) => (phase + gait.offset[leg]) % 1 < gait.stand_frac
  const total = resting.map(() => [0, 0])
  const ticks = resting.map(() => 0)
  for (let tick = 0; tick < sampling.ticks; tick++) {
    const phaseBefore = controller.phase
    const before = body.feet.map(([x, y]) => [x, y])
    controller.step(gait, body, sampling.dt_s)
    for (let i = 0; i < 6; i++) {
      if (!inStance(phaseBefore, i) || !inStance(controller.phase, i)) continue
      total[i][0] += body.feet[i][0] - before[i][0]
      total[i][1] += body.feet[i][1] - before[i][1]
      ticks[i]++
    }
  }

  const foot = total.map(([x, y], i) => [x / Math.max(ticks[i], 1), y / Math.max(ticks[i], 1)])
  const bodyVelocity = [0, 1].map(axis => -foot.reduce((sum, f) => sum + f[axis], 0) / 6)
  const cross = foot.reduce((sum, [dx, dy], i) => sum + resting[i][0] * dy - resting[i][1] * dx, 0)
  const radiusSquared = resting.reduce((sum, [x, y]) => sum + x * x + y * y, 0)
  return { foot, ticks, bodyVelocity, yaw: -cross / radiusSquared }
}

describe('sign convention', () => {
  it('reads three commands for six legs', () => {
    expect(fixture.commands).toHaveLength(3)
    expect(fixture.legs).toHaveLength(6)
  })

  it("puts each leg of the app's resting stance on its fixture side", () => {
    const { resting } = appStance()
    fixture.legs.forEach((leg, i) => {
      expect(
        resting[i].slice(0, 2).map(v => sign(v, SIDE_ZERO_MM)),
        leg
      ).toEqual(fixture.stance_mm[i].slice(0, 2).map(v => sign(v, 0)))
    })
  })

  describe.each(fixture.commands)('$name', command => {
    it('maps the stick to the fixture gait command', () => {
      const motion = new Motion()
      motion.setMode(MotionModes.WALK)
      const { lx, ly, rx } = command.stick
      motion.handleCommand([lx, ly, rx, 0, 0, 0, 0, 0])
      expect(motion.gait_state.step_x).toBeCloseTo(command.gait.step_x)
      expect(motion.gait_state.step_y).toBeCloseTo(command.gait.step_y)
      expect(motion.gait_state.step_angle).toBeCloseTo(command.gait.step_angle)
    })

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
