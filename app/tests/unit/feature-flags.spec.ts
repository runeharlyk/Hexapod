import { beforeEach, describe, expect, it, vi } from 'vitest'
import { derived, get, writable, type Writable } from 'svelte/store'
import type { ITransport, LinkStatus } from '$lib/interfaces/transport.interface'
import { CorrelationResponse, FeaturesDataResponse, Message } from '$lib/platform_shared/message'

type FeatureFlagsModule = typeof import('$lib/stores/featureFlags')

// The robot's webserver, which the flags fall back to on an origin that can reach it: here it never answers.
const httpFeatures = vi.hoisted(() => vi.fn())
vi.mock('$lib/api', () => ({ api: { get: httpFeatures } }))

const featuresOf = (imu: boolean) =>
  FeaturesDataResponse.create({
    firmwareName: 'hexapod',
    firmwareVersion: '1.2.3',
    imu,
    servo: true
  })

// The robot as the feature request reaches it: it answers with `statusCode`, `errorMessage` and `features`, at once or,
// while `holding`, when the test releases the reply.
let robot: {
  statusCode: number
  errorMessage: string
  features: FeaturesDataResponse
  holding: boolean
  held: (() => void)[]
  asked: number
}

const fakeTransport = () => {
  const status = writable<LinkStatus>('disconnected')
  const dataCallbacks: ((bytes: Uint8Array) => void)[] = []
  const connectCallbacks: (() => void)[] = []
  const transport: ITransport = {
    status,
    connected: derived(status, s => s === 'connected'),
    connect: async () => {
      status.set('connected')
      connectCallbacks.forEach(cb => cb())
    },
    disconnect: async () => status.set('disconnected'),
    send: bytes => {
      const request = Message.decode(bytes).correlationRequest
      if (!request?.featuresDataRequest) return
      robot.asked++
      const reply = Message.encode(
        Message.create({
          correlationResponse: CorrelationResponse.create({
            correlationId: request.correlationId,
            statusCode: robot.statusCode,
            errorMessage: robot.errorMessage,
            featuresDataResponse: robot.features
          })
        })
      ).finish()
      const deliver = () => dataCallbacks.forEach(cb => cb(reply))
      if (robot.holding) robot.held.push(deliver)
      else queueMicrotask(deliver)
    },
    onData: cb => dataCallbacks.push(cb),
    onConnect: cb => connectCallbacks.push(cb),
    onDisconnect: () => {}
  }
  return transport
}

// The flags and the broker are module state: every test starts from a fresh load of both.
let flags: FeatureFlagsModule
// The robot the link reaches, as the layout derives it: null while no link is up.
let reached: Writable<string | null>

beforeEach(async () => {
  robot = {
    statusCode: 200,
    errorMessage: '',
    features: featuresOf(true),
    holding: false,
    held: [],
    asked: 0
  }
  httpFeatures.mockReset()
  httpFeatures.mockResolvedValue({ isErr: () => true, inner: new Error('no webserver') })
  vi.resetModules()
  const { dataBroker } = await import('$lib/transport/databroker')
  flags = await import('$lib/stores/featureFlags')
  const transport = fakeTransport()
  dataBroker.addTransport(transport)
  await transport.connect()
  reached = writable(null)
})

const state = () => get(flags.featureFlagsState)
const shown = () => get(flags.useFeatureFlags())

describe('refreshFeatureFlags', () => {
  it('holds the flags pending until the robot has sent its features', async () => {
    expect(state()).toEqual({ kind: 'pending' })
    await flags.refreshFeatureFlags()
    expect(state()).toEqual({ kind: 'known' })
  })

  it("reads the robot's features", async () => {
    await expect(flags.refreshFeatureFlags()).resolves.toMatchObject({
      firmware_name: 'hexapod',
      firmware_version: '1.2.3',
      imu: true,
      servo: true
    })
    expect(httpFeatures).not.toHaveBeenCalled()
  })

  it("takes no features from a reply the robot refused, and says why in the robot's words", async () => {
    robot.statusCode = 500
    robot.errorMessage = 'Busy'
    await expect(flags.refreshFeatureFlags()).rejects.toThrow('Busy')
    expect(httpFeatures).toHaveBeenCalledWith('/api/features')
    expect(state()).toEqual({ kind: 'failed', message: 'Busy' })
    expect(shown().imu).toBe(false)
  })

  it('names a firmware too old for the request', async () => {
    robot.statusCode = 501
    await expect(flags.refreshFeatureFlags()).rejects.toThrow()
    expect(state()).toEqual({
      kind: 'failed',
      message: "the robot's firmware does not support the feature list; update it"
    })
  })

  // The retry a page offers after a failure.
  it('reads again after a failure, pending while it does', async () => {
    robot.statusCode = 500
    await flags.refreshFeatureFlags().catch(() => {})
    robot.statusCode = 200
    const retry = flags.refreshFeatureFlags()
    expect(state()).toEqual({ kind: 'pending' })
    await retry
    expect(state()).toEqual({ kind: 'known' })
    expect(shown().imu).toBe(true)
  })
})

describe('followFeatureFlags', () => {
  it('reads the flags when the link comes up', async () => {
    flags.followFeatureFlags(reached)
    expect(robot.asked).toBe(0)
    reached.set('websocket hexapod.local')
    await vi.waitFor(() => expect(state()).toEqual({ kind: 'known' }))
    expect(shown().imu).toBe(true)
  })

  // A page asks for the flags before the link is up; the broker sends that request as the link comes up, possibly
  // before the link store reports it, and it reaches the robot the link reaches.
  it('asks once when a read waited for the link', async () => {
    vi.resetModules()
    const { dataBroker } = await import('$lib/transport/databroker')
    flags = await import('$lib/stores/featureFlags')
    const transport = fakeTransport()
    dataBroker.addTransport(transport)
    flags.followFeatureFlags(reached)
    flags.useFeatureFlags()
    await transport.connect()
    reached.set('websocket hexapod.local')
    await vi.waitFor(() => expect(state()).toEqual({ kind: 'known' }))
    expect(robot.asked).toBe(1)
  })

  it('reads again when the link comes back after a failed read', async () => {
    robot.statusCode = 500
    flags.followFeatureFlags(reached)
    reached.set('websocket hexapod.local')
    await vi.waitFor(() => expect(state().kind).toBe('failed'))
    robot.statusCode = 200
    reached.set(null)
    reached.set('websocket hexapod.local')
    await vi.waitFor(() => expect(state()).toEqual({ kind: 'known' }))
  })

  // A drop makes the flags stale, not wrong: the robot that comes back is the same one, so the pages that depend on
  // them stay mounted, and the flags are read again once it is back.
  it('keeps the flags through a drop and reads them again when the same robot is back', async () => {
    flags.followFeatureFlags(reached)
    reached.set('websocket hexapod.local')
    await vi.waitFor(() => expect(state()).toEqual({ kind: 'known' }))
    reached.set(null)
    expect(state()).toEqual({ kind: 'known' })
    expect(shown().imu).toBe(true)
    reached.set('websocket hexapod.local')
    await vi.waitFor(() => expect(robot.asked).toBe(2))
    expect(state()).toEqual({ kind: 'known' })
    expect(shown().imu).toBe(true)
  })

  it('keeps the flags it has when reading them again after a drop fails', async () => {
    flags.followFeatureFlags(reached)
    reached.set('websocket hexapod.local')
    await vi.waitFor(() => expect(state()).toEqual({ kind: 'known' }))
    reached.set(null)
    robot.statusCode = 500
    reached.set('websocket hexapod.local')
    await vi.waitFor(() => expect(robot.asked).toBe(2))
    await new Promise(resolve => setTimeout(resolve))
    expect(state()).toEqual({ kind: 'known' })
    expect(shown().imu).toBe(true)
  })

  it("never shows one robot's flags for another", async () => {
    flags.followFeatureFlags(reached)
    reached.set('websocket hexapod-a.local')
    await vi.waitFor(() => expect(shown().imu).toBe(true))
    robot.features = featuresOf(false)
    robot.holding = true
    reached.set('websocket hexapod-b.local')
    expect(state()).toEqual({ kind: 'pending' })
    expect(shown().imu).toBe(false)
    await vi.waitFor(() => expect(robot.held).toHaveLength(1))
    robot.held.forEach(deliver => deliver())
    await vi.waitFor(() => expect(state()).toEqual({ kind: 'known' }))
    expect(shown().imu).toBe(false)
  })

  it('resets the flags when the link comes back to another robot', async () => {
    flags.followFeatureFlags(reached)
    reached.set('websocket hexapod-a.local')
    await vi.waitFor(() => expect(shown().imu).toBe(true))
    reached.set(null)
    robot.holding = true
    reached.set('bluetooth')
    expect(state()).toEqual({ kind: 'pending' })
    expect(shown().imu).toBe(false)
  })

  it('takes nothing from a read that another robot overtook', async () => {
    robot.holding = true
    flags.followFeatureFlags(reached)
    reached.set('websocket hexapod-a.local')
    await vi.waitFor(() => expect(robot.held).toHaveLength(1))
    reached.set(null)
    reached.set('websocket hexapod-b.local')
    await vi.waitFor(() => expect(robot.held).toHaveLength(2))
    robot.held[0]()
    await new Promise(resolve => setTimeout(resolve))
    expect(state()).toEqual({ kind: 'pending' })
    expect(shown().imu).toBe(false)
  })
})
