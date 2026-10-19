import { describe, it, expect } from 'vitest'
import { OTA_CHUNK_BYTES, sendFirmware } from '../../src/lib/firmware/ota'
import type { CorrelationRequest, CorrelationResponse } from '../../src/lib/platform_shared/message'

type Payload = Omit<CorrelationRequest, 'correlationId'>

const reply = (statusCode: number, errorMessage = ''): CorrelationResponse =>
  ({ correlationId: 0, statusCode, errorMessage }) as CorrelationResponse

// A robot that keeps what it is sent, as firmware/include/ota_session.h does, and can refuse a step.
const fakeRobot = (refuse?: (p: Payload) => CorrelationResponse | undefined) => {
  const sent: Payload[] = []
  const written: number[] = []
  const request = async (p: Payload) => {
    sent.push(p)
    const refusal = refuse?.(p)
    if (refusal) return refusal
    if (p.otaChunk) written.push(...p.otaChunk.data)
    return reply(200)
  }
  return { sent, written, request }
}

const image = Uint8Array.from({ length: 2 * OTA_CHUNK_BYTES + 100 }, (_, i) => i & 0xff)

describe('sendFirmware', () => {
  it('announces the size, sends the image in numbered chunks in order and finishes', async () => {
    const robot = fakeRobot()
    const progress: number[] = []
    await sendFirmware(image, robot.request, sent => progress.push(sent))

    expect(robot.sent[0]).toEqual({ otaStart: { size: image.length } })
    expect(robot.sent.slice(1, -1).map(p => p.otaChunk?.index)).toEqual([0, 1, 2])
    expect(robot.sent.at(-1)).toEqual({ otaFinish: {} })
    expect(robot.sent.slice(1, -1).every(p => p.otaChunk!.data.length <= OTA_CHUNK_BYTES)).toBe(
      true
    )
    expect(Uint8Array.from(robot.written)).toEqual(image)
    expect(progress.at(-1)).toBe(image.length)
  })

  it("stops at the first refusal and reports the robot's reason", async () => {
    const robot = fakeRobot(p =>
      p.otaChunk?.index === 1 ? reply(500, 'Writing the update slot failed') : undefined
    )
    await expect(sendFirmware(image, robot.request)).rejects.toThrow(
      'Writing the update slot failed'
    )
    expect(robot.sent.some(p => p.otaFinish)).toBe(false)
    expect(robot.sent.filter(p => p.otaChunk)).toHaveLength(2)
  })

  it('names the step when the robot gives no reason', async () => {
    const robot = fakeRobot(p => (p.otaStart ? reply(409) : undefined))
    await expect(sendFirmware(image, robot.request)).rejects.toThrow(
      'the robot refused starting the update (status 409)'
    )
  })
})
