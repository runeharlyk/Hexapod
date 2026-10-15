import { describe, expect, it } from 'vitest'
import fixture from './fixtures/servo_model.json'
import { GEAR_FRICTION, SERVO, servoTorque } from '../../src/lib/simulation/servo'

describe('the simulated servo', () => {
  it('has the Python model constants', () => {
    const { kp, kv, j_eff, stall, k_hold, noload } = fixture.constants
    expect(SERVO).toEqual({ kp, kv, jEff: j_eff, stall, kHold: k_hold, noload })
  })

  it('commands the torque HexapodSim commands', () => {
    for (const { target, position, velocity, torque } of fixture.samples) {
      expect(servoTorque(target, position, velocity)).toBeCloseTo(torque, 12)
    }
  })

  it('adds the gear friction HexapodSim adds to a powered joint', () => {
    for (const { frictionloss, base_frictionloss } of fixture.samples) {
      expect(base_frictionloss + GEAR_FRICTION).toBeCloseTo(frictionloss, 12)
    }
  })
})
