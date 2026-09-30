import { afterEach, beforeAll, beforeEach, describe, expect, it, vi } from 'vitest'
import { derived, writable } from 'svelte/store'

const linked = vi.hoisted(() => ({ value: null as null | { set: (v: boolean) => void } }))
vi.mock('$lib/stores/link', async () => {
  const { writable } = await import('svelte/store')
  const isLinked = writable(true)
  linked.value = isLinked
  return { isLinked }
})

vi.mock('$lib/stores', async () => {
  const { writable } = await import('svelte/store')
  return { mode: writable('idle'), gait: writable('tri_gate') }
})

import { dataBroker } from '$lib/transport/databroker'
import type { ITransport, LinkStatus } from '$lib/interfaces/transport.interface'
import { CorrelationResponse, Message } from '$lib/platform_shared/message'
import { Animation, ParamId } from '$lib/platform_shared/animation'
import { CHUNK, animationPath, downloadAnimation, uploadAnimation } from '$lib/animation/transfer'
import { PoseSender, playAnimation, stopAnimation } from '$lib/control'
import { MotionModes } from '$lib/motion'
import { stancePose } from '$lib/animation/model'

type Responder = (request: Message) => Partial<CorrelationResponse> | null

const status = writable<LinkStatus>('disconnected')
const sent: Message[] = []
const dataCallbacks: ((bytes: Uint8Array) => void)[] = []
const connectCallbacks: (() => void)[] = []
let responder: Responder = () => null

const transport: ITransport = {
  status,
  connected: derived(status, s => s === 'connected'),
  connect: async () => {
    status.set('connected')
    connectCallbacks.forEach(cb => cb())
  },
  disconnect: async () => status.set('disconnected'),
  send: bytes => {
    const message = Message.decode(bytes)
    sent.push(message)
    const req = message.correlationRequest
    const reply = req ? responder(message) : null
    if (req && reply)
      queueMicrotask(() =>
        dataCallbacks.forEach(cb =>
          cb(
            Message.encode(
              Message.create({
                correlationResponse: { correlationId: req.correlationId, statusCode: 200, ...reply }
              })
            ).finish()
          )
        )
      )
  },
  onData: cb => dataCallbacks.push(cb),
  onConnect: cb => connectCallbacks.push(cb),
  onDisconnect: () => {}
}

const writes = () =>
  sent.flatMap(m =>
    m.correlationRequest?.fileWriteChunk ? [m.correlationRequest.fileWriteChunk] : []
  )
const validates = () => sent.filter(m => m.correlationRequest?.animationValidate)

const animationLargerThan = (bytes: number): Animation => {
  for (let count = 1; count <= 32; count++) {
    const a = Animation.fromPartial({
      name: 'pad',
      schema: 1,
      keyframes: Array.from({ length: count }, (_, i) => ({
        time: i,
        body: { roll: 0.5, x: 3.25, y: 4.5 },
        legs: Array.from({ length: 6 }, () => ({ foot: { x: 1.5, y: 2.25, z: 3.5 } }))
      }))
    })
    if (Animation.encode(a).finish().length > bytes) return a
  }
  throw new Error('cannot reach the requested size')
}

const okReport = { animationReport: { ok: true, error: '', clampedMask: 0 } }

beforeAll(async () => {
  dataBroker.addTransport(transport)
  await transport.connect()
})

beforeEach(() => {
  sent.length = 0
  responder = () => null
  linked.value?.set(true)
})

afterEach(() => vi.useRealTimers())

describe('uploadAnimation', () => {
  it('writes 512-byte chunks in order and then validates', async () => {
    const a = animationLargerThan(1030)
    const total = Animation.encode(a).finish().length
    expect(total).toBeGreaterThan(2 * CHUNK)
    expect(total).toBeLessThanOrEqual(3 * CHUNK)
    responder = m => (m.correlationRequest?.animationValidate ? okReport : { empty: {} })
    const progress: number[] = []

    const result = await uploadAnimation(a, done => progress.push(done))

    expect(result).toEqual({ ok: true, report: { ok: true, error: '', clampedMask: 0 } })
    expect(writes().map(w => w.offset)).toEqual([0, 512, 1024])
    expect(writes().map(w => w.content.length)).toEqual([512, 512, total - 1024])
    expect(writes().every(w => w.totalSize === total && w.path === animationPath('pad'))).toBe(true)
    expect(validates()).toHaveLength(1)
    expect(validates()[0].correlationRequest?.animationValidate?.name).toBe('pad')
    expect(progress).toEqual([512, 1024, total])
    expect(sent[sent.length - 1]).toBe(validates()[0])
  })

  it('restarts once from offset 0 after a failed chunk', async () => {
    let writeCount = 0
    responder = m => {
      if (m.correlationRequest?.animationValidate) return okReport
      return ++writeCount === 2 ? { statusCode: 400 } : { empty: {} }
    }

    const result = await uploadAnimation(animationLargerThan(1030))

    expect(result.ok).toBe(true)
    expect(writes().map(w => w.offset)).toEqual([0, 512, 0, 512, 1024])
  })

  it('gives up after a second failure without validating', async () => {
    responder = m =>
      m.correlationRequest?.fileWriteChunk?.offset === 512 ? { statusCode: 400 } : { empty: {} }

    const result = await uploadAnimation(animationLargerThan(1030))

    expect(result.ok).toBe(false)
    expect(result.ok === false && result.error).toContain('offset 512')
    expect(result.ok === false && result.error).toContain('400')
    expect(writes().map(w => w.offset)).toEqual([0, 512, 0, 512])
    expect(validates()).toHaveLength(0)
  })

  it('reports the validator error that the firmware sends with status 422', async () => {
    responder = m =>
      m.correlationRequest?.animationValidate ?
        { statusCode: 422, animationReport: { ok: false, error: 'bad keyframe', clampedMask: 0 } }
      : { empty: {} }

    const result = await uploadAnimation(animationLargerThan(1030))

    expect(result).toEqual({ ok: false, error: 'bad keyframe' })
    expect(writes()).toHaveLength(3)
  })

  it('throws with the status when validate answers without a report', async () => {
    responder = m =>
      m.correlationRequest?.animationValidate ? { statusCode: 500, empty: {} } : { empty: {} }

    await expect(uploadAnimation(animationLargerThan(10))).rejects.toThrow(/validate pad.*500/)
  })
})

describe('downloadAnimation', () => {
  it('reads chunks until the total size and decodes a valid animation', async () => {
    const source = animationLargerThan(600)
    const bytes = Animation.encode(source).finish()
    expect(bytes.length).toBeGreaterThan(CHUNK)
    expect(bytes.length).toBeLessThanOrEqual(2 * CHUNK)
    responder = m => {
      const r = m.correlationRequest!.fileReadChunk!
      return {
        fileChunk: { content: bytes.slice(r.offset, r.offset + r.length), totalSize: bytes.length }
      }
    }

    const a = await downloadAnimation('pad')

    const reads = sent.map(m => m.correlationRequest!.fileReadChunk!)
    expect(reads.map(r => r.offset)).toEqual([0, 512])
    expect(reads.every(r => r.length === 512 && r.path === '/animations/pad.pb')).toBe(true)
    expect(a.name).toBe('pad')
    expect(a).toEqual(source)
  })

  it('throws with the offset and status on a failed read', async () => {
    responder = () => ({ statusCode: 404 })
    await expect(downloadAnimation('gone')).rejects.toThrow(/offset 0.*404/)
  })
})

describe('control messages', () => {
  it('emits play with the parameter values and stop', () => {
    playAnimation('wave', new Map([[ParamId.SPEED, 1.5]]))
    stopAnimation()

    expect(sent[0].animationPlay).toEqual({
      name: 'wave',
      params: [{ id: ParamId.SPEED, value: 1.5 }]
    })
    expect(sent[1].animationStop).toBeDefined()
  })
})

describe('PoseSender', () => {
  const pose = () => {
    const p = stancePose()
    p.body = [0.5, 0.25, 0.125, 4, 5, 6]
    p.legs[2] = { joints: true, v: [0.5, 0.25, 0.125] }
    p.legs[3] = { joints: false, v: [1, 2, 3] }
    return p
  }
  const poses = () => sent.filter(m => m.pose)

  it('throttles to one pose per 50 ms and carries six leg targets', () => {
    vi.useFakeTimers()
    const sender = new PoseSender()
    const p = pose()
    const before = structuredClone(p)

    sender.send(p)
    sender.send(p)
    expect(poses()).toHaveLength(0)
    vi.advanceTimersByTime(49)
    expect(poses()).toHaveLength(0)
    vi.advanceTimersByTime(1)

    expect(poses()).toHaveLength(1)
    const data = poses()[0].pose!
    expect(data.body).toEqual({ roll: 0.5, pitch: 0.25, yaw: 0.125, x: 4, y: 5, z: 6 })
    expect(data.legs).toHaveLength(6)
    expect(data.legs[2].joints).toEqual({ coxa: 0.5, femur: 0.25, tibia: 0.125 })
    expect(data.legs[3].foot).toEqual({ x: 1, y: 2, z: 3 })
    expect(data.legs[0].foot).toEqual({ x: 0, y: 0, z: 0 })
    expect(p).toEqual(before)
  })

  it('sends nothing while the link is down', () => {
    vi.useFakeTimers()
    linked.value?.set(false)
    new PoseSender().send(pose())
    vi.advanceTimersByTime(200)
    expect(poses()).toHaveLength(0)
  })

  it('cancel drops the pending pose', () => {
    vi.useFakeTimers()
    const sender = new PoseSender()
    sender.send(pose())
    sender.cancel()
    vi.advanceTimersByTime(200)
    expect(poses()).toHaveLength(0)
  })
})

describe('MotionModes', () => {
  it('lists ANIMATE last at wire index 6', () => {
    const modes = Object.values(MotionModes)
    expect(modes[modes.length - 1]).toBe(MotionModes.ANIMATE)
    expect(modes.indexOf(MotionModes.ANIMATE)).toBe(6)
  })
})
