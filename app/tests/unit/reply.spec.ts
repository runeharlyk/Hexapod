import { describe, expect, it } from 'vitest'
import { readdirSync, readFileSync } from 'node:fs'
import { join, relative, resolve } from 'node:path'
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
    expect(interpret({ taskStatsRequest: {} }, reply(200)).kind).toBe('ok')
    expect(interpret({ wifiNetworksGet: {} }, reply(202)).kind).toBe('ok')
  })

  it('reads a 501 as firmware that does not know the request, whatever the request', () => {
    expect(interpret({ taskStatsRequest: {} }, reply(501))).toEqual({ kind: 'too-old' })
    expect(interpret({ mdnsQuery: { service: 'http', protocol: 'tcp' } }, reply(501))).toEqual({
      kind: 'too-old'
    })
    expect(interpret({ otaStart: { size: 1 } }, reply(501, 'Not here'))).toEqual({
      kind: 'too-old'
    })
  })

  // Firmware up to v0.1.1 answers a request it does not know with a bare 400 rather than the 501.
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

  // The listing came after v0.1.1, and the firmware that knows it gives its own 400 a reason: only a firmware that
  // does not know it answers a bare one.
  it('reads a bare 400 to a file listing as firmware that does not know it', () => {
    expect(interpret({ fileList: { path: '/', offset: 0 } }, reply(400))).toEqual({
      kind: 'too-old'
    })
    expect(interpret({ fileList: { path: '/', offset: 0 } }, reply(400, 'bad path'))).toEqual({
      kind: 'refused',
      status: 400,
      message: 'bad path'
    })
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

  it('reads only a 501, or a bare 400, as firmware too old', () => {
    expect(interpret({ taskStatsRequest: {} }, reply(503)).kind).toBe('refused')
    expect(interpret({ taskStatsRequest: {} }, reply(404)).kind).toBe('refused')
  })

  it('reads a status the caller accepts as an answer', () => {
    expect(interpret({ animationValidate: { name: 'wave' } }, reply(422), [422]).kind).toBe('ok')
    expect(interpret({ animationValidate: { name: 'wave' } }, reply(422)).kind).toBe('refused')
  })
})

describe('isFieldless', () => {
  it('holds the requests whose message declares no field, and only those', () => {
    expect(isFieldless({ taskStatsRequest: {} })).toBe(true)
    expect(isFieldless({ otaFinish: {} })).toBe(true)
    expect(isFieldless({ fileDelete: { path: '' } })).toBe(false)
    expect(isFieldless({ servoSettingsUpdate: { servos: [] } })).toBe(false)
  })
})

// The 501 is what makes "firmware too old" exact, so it must mean nothing else: the `default:` of the correlation
// handler in firmware/src/main.cpp answers it, without a message, and no other firmware code answers a 501. The handler
// is part of setup() and does not build on the host, so its source is read; a structure the reading does not find
// fails the test rather than passing it.
const firmware = resolve(__dirname, '../../../firmware')

const withoutCommentsAndStrings = (source: string) =>
  source
    .replace(/\/\*[\s\S]*?\*\//g, ' ')
    .replace(/\/\/[^\n]*/g, ' ')
    .replace(/"(?:\\.|[^"\\\n])*"/g, '""')
    .replace(/'(?:\\.|[^'\\\n])*'/g, "''")

const mustFind = (index: number, what: string) => {
  if (index < 0) throw new Error(`firmware/src/main.cpp: ${what} not found`)
  return index
}

// The text inside the braces of the block whose `{` is at `open`.
const block = (source: string, open: number) => {
  let depth = 0
  for (let i = open; i < source.length; i++) {
    if (source[i] === '{') depth++
    else if (source[i] === '}' && --depth === 0) return source.slice(open + 1, i)
  }
  throw new Error(`firmware/src/main.cpp: the block at ${open} is not closed`)
}

// The `default:` arm of the switch over the request, at the switch's own depth: a nested switch's is not it.
const unknownRequestArm = () => {
  const main = withoutCommentsAndStrings(readFileSync(join(firmware, 'src/main.cpp'), 'utf-8'))
  const handler = mustFind(
    main.indexOf('c.on<socket_message_CorrelationRequest>'),
    'the correlation handler'
  )
  const switchAt = mustFind(
    main.indexOf('switch (req.which_request)', handler),
    "the handler's switch over the request"
  )
  const body = block(main, mustFind(main.indexOf('{', switchAt), "the switch's body"))
  let depth = 0
  for (let i = 0; i < body.length; i++) {
    if (body[i] === '{') depth++
    else if (body[i] === '}') depth--
    else if (depth === 0 && /^default\s*:/.test(body.slice(i))) return body.slice(i)
  }
  throw new Error("firmware/src/main.cpp: the switch's default: arm not found")
}

const firmwareSources = (dir: string): string[] =>
  readdirSync(dir, { withFileTypes: true }).flatMap(entry => {
    const path = join(dir, entry.name)
    // The generated protobuf sources answer nothing.
    if (entry.isDirectory()) return entry.name === 'platform_shared' ? [] : firmwareSources(path)
    return /\.(h|hpp|cpp)$/.test(entry.name) ? [path] : []
  })

describe("the firmware's answer to a request it does not know", () => {
  it('is a 501 without a message', () => {
    expect(unknownRequestArm()).toMatch(
      /^default\s*:\s*res\.status_code\s*=\s*501\s*;\s*break\s*;\s*$/
    )
  })

  it('is the only 501 in the firmware', () => {
    const found = [join(firmware, 'src'), join(firmware, 'include')]
      .flatMap(firmwareSources)
      .flatMap(path =>
        [
          ...withoutCommentsAndStrings(readFileSync(path, 'utf-8')).matchAll(
            /(?<![\w.])501(?![\w.])/g
          )
        ].map(() => relative(firmware, path).replaceAll('\\', '/'))
      )
    expect(found).toEqual(['src/main.cpp'])
  })
})

describe('describeFailure', () => {
  it('names what the firmware lacks and says to update it', () => {
    expect(describeFailure({ kind: 'too-old' }, 'the task table')).toBe(
      "the robot's firmware does not support the task table; update it"
    )
  })

  it("shows the robot's reason, and without one names what was refused but not the status", () => {
    expect(
      describeFailure({ kind: 'refused', status: 409, message: 'Busy' }, 'the core-dump erase')
    ).toBe('Busy')
    expect(describeFailure({ kind: 'refused', status: 503, message: '' }, 'the I2C scan')).toBe(
      'the robot refused the I2C scan'
    )
  })
})

describe('ask', () => {
  it('resolves to the reply the robot answered', async () => {
    const answer = {
      ...reply(200),
      taskStats: { windowUs: 1, snapshotUs: 0, taskCount: 0, tasks: [] }
    }
    await expect(ask(answering(answer), { taskStatsRequest: {} }, 'the task table')).resolves.toBe(
      answer
    )
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
      ask(
        failing(new RequestFailed({ kind: 'timeout' })),
        { taskStatsRequest: {} },
        'the task table'
      )
    ).rejects.toMatchObject({
      failure: { kind: 'timeout' },
      message: 'the robot did not answer in time for the task table'
    })
    await expect(
      ask(
        failing(new RequestFailed({ kind: 'disconnected' })),
        { taskStatsRequest: {} },
        'the task table'
      )
    ).rejects.toMatchObject({ failure: { kind: 'disconnected' } })
  })

  it('lets an abort through as the AbortError it is', async () => {
    const abort = new DOMException('Aborted', 'AbortError')
    await expect(ask(failing(abort), { coreDumpGet: {} }, 'the core dump')).rejects.toBe(abort)
  })
})
