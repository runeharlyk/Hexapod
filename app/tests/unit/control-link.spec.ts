import { describe, it, expect } from 'vitest'
import { readFileSync } from 'node:fs'
import { resolve } from 'node:path'
import { COMMAND_HEARTBEAT_MS, needsHeartbeat } from '../../src/lib/control-link'
import { MotionModes } from '../../src/lib/motion'

const firmwareConstant = (name: string) => {
  const source = readFileSync(resolve(__dirname, '../../../firmware/include/motion.h'), 'utf-8')
  const match = source.match(new RegExp(`${name}\\s*=\\s*(\\d+)`))
  if (!match) throw new Error(`${name} not found in motion.h`)
  return Number(match[1])
}

describe('control link', () => {
  it('re-sends often enough that two lost heartbeats do not stop a steering robot', () => {
    expect(3 * COMMAND_HEARTBEAT_MS).toBeLessThanOrEqual(firmwareConstant('STEERING_TIMEOUT_MS'))
  })

  it('keeps every stick-driven mode alive, the learned walk included', () => {
    for (const mode of [MotionModes.STAND, MotionModes.WALK, MotionModes.WALK_NN]) {
      expect(needsHeartbeat(mode)).toBe(true)
    }
  })

  it('leaves animations and the idle modes to the firmware', () => {
    for (const mode of [MotionModes.DEACTIVATED, MotionModes.IDLE, MotionModes.ANIMATE]) {
      expect(needsHeartbeat(mode)).toBe(false)
    }
  })
})
