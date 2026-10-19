import { describe, it, expect } from 'vitest'
import { readFileSync } from 'node:fs'
import { resolve } from 'node:path'
import { calibrateImu } from '../../src/lib/imu-calibration'
import { RequestFailed } from '../../src/lib/transport/reply'
import { cardinalDirection } from '../../src/lib/compass'

// The robot as calibrateImu asks it: one reply, or a failure as the data broker raises it.
const answering = (statusCode: number, errorMessage = '') => ({
  request: async () => ({ statusCode, errorMessage })
})

describe('calibrateImu', () => {
  it('explains every status the firmware answers with', async () => {
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
      const result = await calibrateImu(answering(code))
      expect(result.ok).toBe(code === 200)
      if (!result.ok) expect(result.error).not.toMatch(/status/)
    }
  })

  it("passes on the robot's reason for a failure it explains itself", async () => {
    expect(
      await calibrateImu(answering(504, 'The IMU calibration did not finish in time'))
    ).toEqual({ ok: false, error: 'The IMU calibration did not finish in time' })
  })

  // ImuCalibrateRequest has no fields, so a bare 400 is a firmware from before the calibration.
  it('names a firmware that predates the calibration', async () => {
    expect(await calibrateImu(answering(400))).toEqual({
      ok: false,
      error: "the robot's firmware does not support IMU calibration; update it"
    })
  })

  it('reports a link that drops before the robot answers', async () => {
    const dropped = {
      request: async (): Promise<{ statusCode: number; errorMessage: string }> => {
        throw new RequestFailed({ kind: 'disconnected' })
      }
    }
    expect(await calibrateImu(dropped)).toEqual({ ok: false, error: 'the robot is not connected' })
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
