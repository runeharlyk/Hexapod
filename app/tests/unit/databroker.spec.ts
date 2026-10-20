import { afterEach, beforeEach, describe, expect, it, vitest } from 'vitest'
import { derived, get, writable } from 'svelte/store'
import { DataBroker } from '../../src/lib/transport/databroker'
import { tagOf } from '../../src/lib/transport/message-codec'
import type { ITransport, LinkStatus } from '../../src/lib/interfaces/transport.interface'
import {
  ControllerInputData,
  CorrelationResponse,
  IMUData,
  Message
} from '../../src/lib/platform_shared/message'

const createFakeTransport = () => {
  const status = writable<LinkStatus>('disconnected')
  const sent: Message[] = []
  // The motion frames handed over as the latest of their kind (transport/latest-frames.ts), with that kind.
  const latest: { kind: string; message: Message }[] = []
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
    sendLatest: (kind, bytes) => {
      if (get(status) !== 'connected') return false
      latest.push({ kind, message: Message.decode(bytes) })
      return true
    },
    onData: cb => callbacks.data.push(cb),
    onConnect: cb => callbacks.connect.push(cb),
    onDisconnect: cb => callbacks.disconnect.push(cb)
  }
  const receive = (message: Message) =>
    callbacks.data.forEach(cb => cb(Message.encode(message).finish()))
  return { transport, sent, latest, receive }
}

const imu = (x: number) => Message.create({ imu: IMUData.create({ x }) })
const sticks = (y: number) => ControllerInputData.create({ left: { x: 0, y } })

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

    await expect(request).rejects.toMatchObject({ failure: { kind: 'disconnected' } })
  })

  it('drops the oldest request held while no transport is connected once too many wait', async () => {
    const link = createFakeTransport()
    broker.addTransport(link.transport)

    const oldest = broker.request({ featuresDataRequest: {} })
    for (let i = 0; i < 16; i++) broker.request({ taskStatsRequest: {} }).catch(() => {})

    await expect(oldest).rejects.toMatchObject({ failure: { kind: 'disconnected' } })
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

    await expect(request).rejects.toMatchObject({ failure: { kind: 'timeout' } })
    expect(removeListener).toHaveBeenCalledWith('abort', expect.any(Function))
  })

  // Motion goes to the link up as the latest frame of its kind, which the link coalesces; nothing is held for a link
  // that is down, so nothing old is sent after a reconnect.
  it('hands a volatile frame to the link up as the latest of its kind, and takes none while no link is up', async () => {
    const serial = createFakeTransport()
    broker.addTransport(serial.transport)
    await serial.transport.connect()

    expect(broker.emitVolatile(ControllerInputData, sticks(0.5))).toBe(true)
    await serial.transport.disconnect()
    expect(broker.emitVolatile(ControllerInputData, sticks(0.75))).toBe(false)
    await serial.transport.connect()

    expect(serial.latest.map(({ kind, message }) => [kind, message.input?.left?.y])).toEqual([
      [`${tagOf(ControllerInputData)}`, 0.5]
    ])
    expect(serial.sent.filter(m => m.input)).toEqual([])
  })
})
