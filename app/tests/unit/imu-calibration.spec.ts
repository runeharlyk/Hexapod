import { describe, it, expect } from 'vitest'
import { readFileSync } from 'node:fs'
import { resolve } from 'node:path'
import { describeCalibration } from '../../src/lib/imu-calibration'
import { cardinalDirection } from '../../src/lib/compass'

describe('describeCalibration', () => {
  it('maps every status the firmware answers with', () => {
    // The firmware's codes, read from its handler so a renumbering there fails here.
    const main = readFileSync(resolve(__dirname, '../../../firmware/src/main.cpp'), 'utf-8')
    const handler = main.slice(main.indexOf('CorrelationRequest_imu_calibrate_tag'))
    const codes = [...handler.slice(0, handler.indexOf('break;')).matchAll(/status_code = (\d+)/g)]
    const ternary = handler.match(/Check::OK \? (\d+) : (\d+)/)
    const firmwareCodes = new Set([
      ...codes.map(m => Number(m[1])),
      Number(ternary?.[1]),
      Number(ternary?.[2])
    ])
    expect(firmwareCodes).toEqual(new Set([200, 409, 422, 503]))
    for (const code of firmwareCodes) {
      const result = describeCalibration(code)
      expect(result.ok).toBe(code === 200)
      if (!result.ok) expect(result.error).not.toMatch(/status/)
    }
  })

  it('reports an unexpected status as such', () => {
    expect(describeCalibration(500)).toEqual({
      ok: false,
      error: 'Calibration failed with status 500.'
    })
  })
})

describe('cardinalDirection', () => {
  it('names the eight points, rounding to the nearest', () => {
    expect(cardinalDirection(0)).toBe('N')
    expect(cardinalDirection(22)).toBe('N')
    expect(cardinalDirection(23)).toBe('NE')
    expect(cardinalDirection(180)).toBe('S')
    expect(cardinalDirection(350)).toBe('N')
  })

  it('accepts negative and multi-turn headings', () => {
    expect(cardinalDirection(-90)).toBe('W')
    expect(cardinalDirection(450)).toBe('E')
  })
})
