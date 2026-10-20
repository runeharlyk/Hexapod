// The calibration page's "Centre all servos" (src/lib/servo-centre.ts), which replaced the POSE mode: a request the
// robot carries out in IDLE alone.
import { describe, expect, it } from 'vitest'
import { readFileSync } from 'node:fs'
import { resolve } from 'node:path'
import { centreServos } from '$lib/servo-centre'

const answering = (statusCode: number, errorMessage = '') => {
  const asked: object[] = []
  return {
    asked,
    request: async (payload: object) => {
      asked.push(payload)
      return { statusCode, errorMessage }
    }
  }
}

// The reason the firmware gives with its 409, read from its handler so a change there fails here.
const firmwareRefusal = () => {
  const main = readFileSync(resolve(__dirname, '../../../firmware/src/main.cpp'), 'utf-8')
  const handler = main.slice(main.indexOf('CorrelationRequest_servo_centre_tag'))
  return handler
    .slice(0, handler.indexOf('break;'))
    .match(/strncpy\(res\.error_message, "([^"]+)"/)?.[1]
}

describe('centreServos', () => {
  it('asks the robot for the centring, and reports it done on a 200', async () => {
    const robot = answering(200)
    expect(await centreServos(robot)).toEqual({ ok: true })
    expect(robot.asked).toEqual([{ servoCentre: {} }])
  })

  it("passes on the robot's reason when it is not idle", async () => {
    const reason = firmwareRefusal()
    expect(reason).toMatch(/idle/i)
    expect(await centreServos(answering(409, reason))).toEqual({ ok: false, error: reason })
  })

  // ServoCentreRequest has no fields, so a bare 400 is a firmware from before the request, as is a 501.
  it('names a firmware that predates the centring', async () => {
    const outdated = {
      ok: false,
      error: "the robot's firmware does not support centring the servos; update it"
    }
    expect(await centreServos(answering(400))).toEqual(outdated)
    expect(await centreServos(answering(501))).toEqual(outdated)
  })
})
