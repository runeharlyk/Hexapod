import { afterEach, beforeEach, describe, expect, it, vitest } from 'vitest'
import { derived, writable } from 'svelte/store'
import { DataBroker } from '../../src/lib/transport/databroker'
import type { ITransport, LinkStatus } from '../../src/lib/interfaces/transport.interface'
import { CorrelationResponse, IMUData, Message } from '../../src/lib/platform_shared/message'

const createFakeTransport = () => {
  const status = writable<LinkStatus>('disconnected')
  const sent: Message[] = []
  const callbacks = {
    data: [] as ((bytes: Uint8Array) => void)[],
    connect: [] as (() => void)[],
    disconnect: [] as (() => void)[]
  }
  const transport: ITransport = {
    status,
    connected: derived(status, s => s === 'connected'),
    connect: async () => {
      status.set('connected')
      callbacks.connect.forEach(cb => cb())
    },
    disconnect: async () => {
      status.set('disconnected')
      callbacks.disconnect.forEach(cb => cb())
    },
    send: bytes => sent.push(Message.decode(bytes)),
    onData: cb => callbacks.data.push(cb),
    onConnect: cb => callbacks.connect.push(cb),
    onDisconnect: cb => callbacks.disconnect.push(cb)
  }
  const receive = (message: Message) =>
    callbacks.data.forEach(cb => cb(Message.encode(message).finish()))
  return { transport, sent, receive }
}

const imu = (x: number) => Message.create({ imu: IMUData.create({ x }) })

describe('DataBroker', () => {
  let broker: DataBroker

  beforeEach(() => {
    vitest.useFakeTimers()
    broker = new DataBroker()
  })

  afterEach(() => {
    vitest.useRealTimers()
  })

  it('keeps delivering to the other listeners when one throws', async () => {
    const link = createFakeTransport()
    broker.addTransport(link.transport)
    await link.transport.connect()
    const received: number[] = []
    const consoleError = vitest.spyOn(console, 'error').mockImplementation(() => {})

    broker.on(IMUData, () => {
      throw new Error('broken widget')
    })
    broker.on(IMUData, data => received.push(data.x))

    expect(() => link.receive(imu(1.5))).not.toThrow()
    link.receive(imu(2.5))
    expect(received).toEqual([1.5, 2.5])
    consoleError.mockRestore()
  })

  it('sends only on the highest-priority connected transport', async () => {
    const serial = createFakeTransport()
    const websocket = createFakeTransport()
    broker.addTransport(serial.transport)
    broker.addTransport(websocket.transport)

    await websocket.transport.connect()
    await serial.transport.connect()
    serial.sent.length = 0
    websocket.sent.length = 0

    broker.emit(IMUData, IMUData.create({ x: 1 }))
    expect(serial.sent).toHaveLength(1)
    expect(websocket.sent).toHaveLength(0)

    await serial.transport.disconnect()
    websocket.sent.length = 0
    broker.emit(IMUData, IMUData.create({ x: 2 }))
    expect(websocket.sent.map(m => m.imu?.x)).toEqual([2])
  })

  it('moves subscriptions to the remaining transport when the active one drops', async () => {
    const serial = createFakeTransport()
    const websocket = createFakeTransport()
    broker.addTransport(serial.transport)
    broker.addTransport(websocket.transport)
    await websocket.transport.connect()
    await serial.transport.connect()
    broker.on(IMUData, () => {})
    websocket.sent.length = 0

    await serial.transport.disconnect()
    expect(websocket.sent.some(m => m.subNotif !== undefined)).toBe(true)
  })

  it('rejects pending requests once the last transport disconnects', async () => {
    const link = createFakeTransport()
    broker.addTransport(link.transport)
    await link.transport.connect()

    const request = broker.request({ featuresDataRequest: {} })
    await link.transport.disconnect()

    await expect(request).rejects.toThrow('disconnected')
  })

  it('resolves a request by correlation id and ignores later timeouts', async () => {
    const link = createFakeTransport()
    broker.addTransport(link.transport)
    await link.transport.connect()

    const request = broker.request({ featuresDataRequest: {} })
    const sentRequest = link.sent.find(m => m.correlationRequest)?.correlationRequest
    link.receive(
      Message.create({
        correlationResponse: CorrelationResponse.create({
          correlationId: sentRequest!.correlationId,
          statusCode: 200
        })
      })
    )

    await expect(request).resolves.toMatchObject({ statusCode: 200 })
    vitest.advanceTimersByTime(20000)
  })

  it('removes the abort listener once a request settles', async () => {
    const link = createFakeTransport()
    broker.addTransport(link.transport)
    await link.transport.connect()
    const controller = new AbortController()
    const removeListener = vitest.spyOn(controller.signal, 'removeEventListener')

    const request = broker.request({ featuresDataRequest: {} }, { signal: controller.signal })
    vitest.advanceTimersByTime(15000)

    await expect(request).rejects.toThrow('timed out')
    expect(removeListener).toHaveBeenCalledWith('abort', expect.any(Function))
  })
})
