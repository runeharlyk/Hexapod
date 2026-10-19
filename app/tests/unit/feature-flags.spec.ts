import { beforeAll, beforeEach, describe, expect, it, vi } from 'vitest'
import { derived, writable } from 'svelte/store'
import { dataBroker } from '$lib/transport/databroker'
import type { ITransport, LinkStatus } from '$lib/interfaces/transport.interface'
import { CorrelationResponse, FeaturesDataResponse, Message } from '$lib/platform_shared/message'
import { refreshFeatureFlags } from '$lib/stores/featureFlags'

// The robot's webserver, which the flags fall back to on an origin that can reach it: here it never answers.
const httpFeatures = vi.hoisted(() => vi.fn())
vi.mock('$lib/api', () => ({ api: { get: httpFeatures } }))

const features = FeaturesDataResponse.create({
  firmwareName: 'hexapod',
  firmwareVersion: '1.2.3',
  imu: true,
  servo: true
})

// The robot as the feature request reaches it: it answers with `statusCode` and its features either way.
let statusCode = 200
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
    const reply = Message.encode(
      Message.create({
        correlationResponse: CorrelationResponse.create({
          correlationId: request.correlationId,
          statusCode,
          featuresDataResponse: features
        })
      })
    ).finish()
    queueMicrotask(() => dataCallbacks.forEach(cb => cb(reply)))
  },
  onData: cb => dataCallbacks.push(cb),
  onConnect: cb => connectCallbacks.push(cb),
  onDisconnect: () => {}
}

beforeAll(async () => {
  dataBroker.addTransport(transport)
  await transport.connect()
})

beforeEach(() => {
  statusCode = 200
  httpFeatures.mockReset()
  httpFeatures.mockResolvedValue({ isErr: () => true, inner: new Error('no webserver') })
})

describe('refreshFeatureFlags', () => {
  it("reads the robot's features", async () => {
    await expect(refreshFeatureFlags()).resolves.toMatchObject({
      firmware_name: 'hexapod',
      firmware_version: '1.2.3',
      imu: true,
      servo: true
    })
    expect(httpFeatures).not.toHaveBeenCalled()
  })

  it('takes no features from a reply the robot refused', async () => {
    statusCode = 500
    await expect(refreshFeatureFlags()).rejects.toThrow('no webserver')
    expect(httpFeatures).toHaveBeenCalledWith('/api/features')
  })
})
