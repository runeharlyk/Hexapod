import { describe, expect, it } from 'vitest'
import { readFileSync } from 'node:fs'
import { resolve } from 'node:path'
import {
  ask,
  describeFailure,
  interpret,
  isFieldless,
  RequestFailed,
  type Reply
} from '../../src/lib/transport/reply'

const reply = (statusCode: number, errorMessage = ''): Reply => ({ statusCode, errorMessage })

// A link that answers every request with one reply, and one that fails every request as the broker does.
const answering = (answer: Reply) => ({ request: async () => answer })
const failing = (error: unknown) => ({
  request: async (): Promise<Reply> => {
    throw error
  }
})

describe('interpret', () => {
  it('reads any 2xx as an answer, the 202 of a running scan included', () => {
    expect(interpret({ modeGet: {} }, reply(200)).kind).toBe('ok')
    expect(interpret({ wifiNetworksGet: {} }, reply(202)).kind).toBe('ok')
  })

  it('reads a bare 400 to a request without fields as firmware that does not know it', () => {
    expect(interpret({ taskStatsRequest: {} }, reply(400))).toEqual({ kind: 'too-old' })
    expect(interpret({ coreDumpGet: {} }, reply(400))).toEqual({ kind: 'too-old' })
  })

  // A request with fields can be malformed, and the firmware answers that with a bare 400 too (a settings update its
  // validator refuses, a file path outside the filesystem): that is a refusal, never a firmware too old.
  it('reads a bare 400 to a request with fields as a refusal', () => {
    expect(interpret({ servoSettingsUpdate: { servos: [] } }, reply(400))).toEqual({
      kind: 'refused',
      status: 400,
      message: ''
    })
    expect(interpret({ coreDumpRead: { offset: 0, length: 4096 } }, reply(400)).kind).toBe(
      'refused'
    )
  })

  // fileDelete with an empty path encodes to no bytes at all: the message type decides, not the values sent.
  it('decides by the request type, not by whether the values sent are defaults', () => {
    expect(interpret({ fileDelete: { path: '' } }, reply(400)).kind).toBe('refused')
  })

  it("keeps the robot's reason, and a reason makes a 400 a refusal", () => {
    expect(interpret({ taskStatsRequest: {} }, reply(400, 'No room'))).toEqual({
      kind: 'refused',
      status: 400,
      message: 'No room'
    })
    expect(interpret({ coreDumpErase: {} }, reply(409, 'Deactivate the robot first'))).toEqual({
      kind: 'refused',
      status: 409,
      message: 'Deactivate the robot first'
    })
  })

  it('reads only a 400 as firmware too old', () => {
    expect(interpret({ taskStatsRequest: {} }, reply(503)).kind).toBe('refused')
    expect(interpret({ taskStatsRequest: {} }, reply(404)).kind).toBe('refused')
  })

  it('reads a status the caller accepts as an answer', () => {
    expect(interpret({ animationValidate: { name: 'wave' } }, reply(422), [422]).kind).toBe('ok')
    expect(interpret({ animationValidate: { name: 'wave' } }, reply(422)).kind).toBe('refused')
  })
})

describe('isFieldless', () => {
  // Every request the firmware answers with a bare 400 for a reason of its own must have fields, or that 400 would read
  // as firmware too old. The handler's cases are read from main.cpp, so a new one that does fails here.
  it('holds no request whose handler answers a bare 400 of its own (firmware/src/main.cpp)', () => {
    const main = readFileSync(resolve(__dirname, '../../../firmware/src/main.cpp'), 'utf-8')
    const handler = main.slice(main.indexOf('c.on<socket_message_CorrelationRequest>'))
    const cases = handler
      .slice(0, handler.indexOf('default:'))
      .split(/case socket_message_CorrelationRequest_/)
    const bare400 = cases
      .slice(1)
      .filter(body => /status_code =[^;]*\b400\b/.test(body) || /\breturn 400\b/.test(body))
      .map(body => body.slice(0, body.indexOf('_tag')))
    expect(bare400).toContain('file_delete')
    const camel = (snake: string) =>
      snake.replace(/_([a-z0-9])/g, (_, c: string) => c.toUpperCase())
    for (const request of bare400)
      expect(isFieldless({ [camel(request)]: {} }), request).toBe(false)
  })
})

describe('describeFailure', () => {
  it('names what the firmware lacks and says to update it', () => {
    expect(describeFailure({ kind: 'too-old' }, 'the task table')).toBe(
      "the robot's firmware does not support the task table; update it"
    )
  })

  it("shows the robot's reason, and the status when it gave none", () => {
    expect(
      describeFailure({ kind: 'refused', status: 409, message: 'Busy' }, 'the core-dump erase')
    ).toBe('Busy')
    expect(describeFailure({ kind: 'refused', status: 503, message: '' }, 'the I2C scan')).toBe(
      'the robot refused the I2C scan (status 503)'
    )
  })
})

describe('ask', () => {
  it('resolves to the reply the robot answered', async () => {
    const answer = { ...reply(200), mode: { mode: 3, handheld: false } }
    await expect(ask(answering(answer), { modeGet: {} }, 'the mode')).resolves.toBe(answer)
  })

  it('rejects with the failure and a sentence naming the feature', async () => {
    const failure = ask(answering(reply(400)), { taskStatsRequest: {} }, 'the task table')
    await expect(failure).rejects.toBeInstanceOf(RequestFailed)
    await expect(failure).rejects.toMatchObject({
      failure: { kind: 'too-old' },
      message: "the robot's firmware does not support the task table; update it"
    })
  })

  it("names the feature in the broker's timeout and drop", async () => {
    await expect(
      ask(failing(new RequestFailed({ kind: 'timeout' })), { modeGet: {} }, 'the mode')
    ).rejects.toMatchObject({
      failure: { kind: 'timeout' },
      message: 'the robot did not answer in time for the mode'
    })
    await expect(
      ask(failing(new RequestFailed({ kind: 'disconnected' })), { modeGet: {} }, 'the mode')
    ).rejects.toMatchObject({ failure: { kind: 'disconnected' } })
  })

  it('lets an abort through as the AbortError it is', async () => {
    const abort = new DOMException('Aborted', 'AbortError')
    await expect(ask(failing(abort), { wifiStatusGet: {} }, 'the Wi-Fi status')).rejects.toBe(abort)
  })
})
