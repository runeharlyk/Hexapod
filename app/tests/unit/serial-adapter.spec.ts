import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest'
import { get } from 'svelte/store'
import { Message } from '../../src/lib/platform_shared/message'
import {
  SERIAL_PROBE_INTERVAL_MS,
  createSerialAdapter
} from '../../src/lib/transport/serial-adapter'
import { encodeSerialFrame } from '../../src/lib/transport/serial-framing'
import { FakeSerialPort, fakeSerial, pongs, serialRobot } from './fake-serial'

const frameOf = (message: Message) => encodeSerialFrame(Message.encode(message).finish())

describe('serial adapter', () => {
  let port: FakeSerialPort
  let fake: ReturnType<typeof fakeSerial>

  beforeEach(() => {
    port = new FakeSerialPort()
    fake = fakeSerial([], port)
    vi.stubGlobal('navigator', { serial: fake.serial })
  })

  afterEach(() => {
    vi.useRealTimers()
    vi.unstubAllGlobals()
  })

  it('opens the picked port with DTR and RTS released, so the board is not held in reset', async () => {
    const adapter = createSerialAdapter()
    await adapter.connect()
    expect(port.isOpen).toBe(true)
    expect(port.signals).toEqual([{ dataTerminalReady: false, requestToSend: false }])
    await adapter.disconnect()
  })

  it('reopens a port granted before without asking the user again', async () => {
    const granted = new FakeSerialPort()
    fake = fakeSerial([granted])
    vi.stubGlobal('navigator', { serial: fake.serial })
    const adapter = createSerialAdapter()
    await adapter.connect()
    expect(granted.isOpen).toBe(true)
    await adapter.disconnect()
  })

  it('counts as connected only once the robot answers, pinging it while it boots', async () => {
    vi.useFakeTimers()
    const robot = serialRobot(port, () => undefined)
    const adapter = createSerialAdapter()
    const connected = vi.fn()
    adapter.onConnect(connected)
    await adapter.connect()
    await vi.advanceTimersByTimeAsync(2 * SERIAL_PROBE_INTERVAL_MS + 10)

    expect(get(adapter.status)).toBe('connecting')
    expect(robot.received.filter(m => m.pingmsg).length).toBe(3)
    expect(connected).not.toHaveBeenCalled()

    adapter.send(Message.encode(Message.create({ pingmsg: {} })).finish())
    port.deliver(frameOf(Message.create({ pongmsg: {} })))
    await vi.advanceTimersByTimeAsync(0)
    expect(get(adapter.status)).toBe('connected')
    expect(connected).toHaveBeenCalledOnce()

    // The probing stops once answered.
    const pings = robot.received.length
    await vi.advanceTimersByTimeAsync(3 * SERIAL_PROBE_INTERVAL_MS)
    expect(robot.received.length).toBe(pings)
    await adapter.disconnect()
  })

  it('hands over every frame however the driver splits the bytes', async () => {
    serialRobot(port, pongs)
    const adapter = createSerialAdapter()
    const data = vi.fn()
    adapter.onData(data)
    await adapter.connect()
    await vi.waitFor(() => expect(get(adapter.status)).toBe('connected'))

    const message = Message.create({ pongmsg: {} })
    const bytes = new Uint8Array([...frameOf(message), ...frameOf(message)])
    for (let at = 0; at < bytes.length; at += 3) port.deliver(bytes.slice(at, at + 3))

    await vi.waitFor(() => expect(data).toHaveBeenCalledTimes(2))
    expect(Message.decode(data.mock.calls[0][0]).pongmsg).toBeDefined()
    await adapter.disconnect()
  })

  it('reopens the same board after it resets and comes back as another port', async () => {
    serialRobot(port, pongs)
    const adapter = createSerialAdapter()
    const disconnected = vi.fn()
    const connected = vi.fn()
    adapter.onDisconnect(disconnected)
    adapter.onConnect(connected)
    await adapter.connect()
    await vi.waitFor(() => expect(connected).toHaveBeenCalledOnce())

    fake.unplug(port)
    await vi.waitFor(() => expect(disconnected).toHaveBeenCalledOnce())
    expect(get(adapter.status)).toBe('connecting')

    const back = new FakeSerialPort()
    serialRobot(back, pongs)
    fake.plugIn(back)
    await vi.waitFor(() => expect(connected).toHaveBeenCalledTimes(2))
    expect(back.isOpen).toBe(true)
    await adapter.disconnect()
  })

  it('stays closed after the user disconnects, even when the board returns', async () => {
    serialRobot(port, pongs)
    const adapter = createSerialAdapter()
    await adapter.connect()
    await vi.waitFor(() => expect(get(adapter.status)).toBe('connected'))

    await adapter.disconnect()
    expect(port.isOpen).toBe(false)
    expect(get(adapter.status)).toBe('disconnected')

    fake.plugIn(new FakeSerialPort())
    await new Promise(resolve => setTimeout(resolve, 20))
    expect(get(adapter.status)).toBe('disconnected')
  })
})
