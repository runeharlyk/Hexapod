// The BLE adapter over a stand-in for Web Bluetooth. The robot sends each frame as [uint16 LE length][payload] in
// notifications of 180 bytes and a shorter last one, and a frame cut short must never be completed with the bytes of
// the frames after it: the adapter drops it and reconnects, so the stream starts clean.
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { get } from 'svelte/store'
import type { ITransport } from '$lib/interfaces/transport.interface'

const CHUNK = 180

const frame = (length: number, seed: number) =>
  Uint8Array.from({ length }, (_, i) => (seed + i * 7) & 0xff)

const notifications = (message: Uint8Array) => {
  const framed = new Uint8Array(2 + message.length)
  framed[0] = message.length & 0xff
  framed[1] = message.length >> 8
  framed.set(message, 2)
  const out: Uint8Array[] = []
  for (let at = 0; at < framed.length; at += CHUNK) out.push(framed.slice(at, at + CHUNK))
  return out
}

/** A robot reached over Web Bluetooth, which notifies only while the link is up. */
const fakeRobot = () => {
  let onValue: ((event: Event) => void) | undefined
  const onDrop = new Set<() => void>()
  const tx = {
    startNotifications: async () => {},
    addEventListener: (_: string, cb: (event: Event) => void) => (onValue = cb)
  }
  const rx = { writeValueWithoutResponse: async () => {} }
  const service = {
    getCharacteristic: async (uuid: string) => (uuid.startsWith('6e400002') ? rx : tx)
  }
  const gatt = {
    connected: false,
    connects: 0,
    /** The next connects that fail, as for a robot out of range. */
    refusals: 0,
    connect: async () => {
      gatt.connects++
      if (gatt.refusals > 0) {
        gatt.refusals--
        throw new DOMException('Connection failed', 'NetworkError')
      }
      gatt.connected = true
      return { getPrimaryService: async () => service }
    },
    // The browser fires gattserverdisconnected for an explicit disconnect too.
    disconnect: () => robot.drop()
  }
  const device = {
    name: 'Hexapod',
    gatt,
    addEventListener: (_: string, cb: () => void) => onDrop.add(cb)
  }
  const robot = {
    gatt,
    device,
    notify: (bytes: Uint8Array) => {
      if (!gatt.connected) return
      const value = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength)
      onValue?.({ target: { value } } as unknown as Event)
    },
    send: (message: Uint8Array) => notifications(message).forEach(robot.notify),
    drop: () => {
      if (!gatt.connected) return
      gatt.connected = false
      onDrop.forEach(cb => cb())
    }
  }
  return robot
}

describe('BLE adapter', () => {
  let robot: ReturnType<typeof fakeRobot>
  let ble: ITransport
  let received: Uint8Array[]
  let connects: number

  beforeEach(async () => {
    vi.resetModules()
    robot = fakeRobot()
    vi.stubGlobal('navigator', { bluetooth: { requestDevice: async () => robot.device } })
    ble = (await import('$lib/transport/ble-adapter')).ble
    received = []
    connects = 0
    ble.onData(data => received.push(data))
    ble.onConnect(() => connects++)
    await ble.connect()
  })

  afterEach(async () => {
    vi.useRealTimers()
    await ble.disconnect()
    vi.unstubAllGlobals()
  })

  it('reassembles a frame sent in several notifications', () => {
    const message = frame(400, 1)
    robot.send(message)
    expect(received).toEqual([message])
  })

  it('drops a frame cut short and resyncs through a reconnect, never completing it with later frames', async () => {
    const cut = notifications(frame(400, 3))[0]
    const later = Array.from({ length: 10 }, (_, i) => frame(30, 50 + i))
    robot.notify(cut)
    later.forEach(robot.send)

    await vi.waitFor(() => expect(connects).toBe(2))
    expect(robot.gatt.connects).toBe(2)
    const whole = frame(400, 9)
    robot.send(whole)

    expect(received.at(-1)).toEqual(whole)
    const sent = [...later, whole]
    for (const data of received) expect(sent).toContainEqual(data)
  })

  it('takes a length beyond the largest frame as a stream out of step and reconnects', async () => {
    robot.notify(Uint8Array.of(0xff, 0xff, 1, 2, 3))

    await vi.waitFor(() => expect(connects).toBe(2))
    expect(received).toEqual([])
  })

  it('reconnects to the same robot when the link drops', async () => {
    robot.drop()

    await vi.waitFor(() => expect(get(ble.status)).toBe('connected'))
    expect(connects).toBe(2)
    const message = frame(20, 4)
    robot.send(message)
    expect(received).toEqual([message])
  })

  it('gives up reconnecting after a bounded number of attempts', async () => {
    const { BLE_RECONNECT_ATTEMPTS } = await import('$lib/transport/ble-adapter')
    vi.useFakeTimers()
    robot.gatt.refusals = Infinity
    robot.drop()

    await vi.runAllTimersAsync()

    expect(robot.gatt.connects).toBe(1 + BLE_RECONNECT_ATTEMPTS)
    expect(get(ble.status)).toBe('disconnected')
  })

  it('does not reconnect after the user disconnects', async () => {
    vi.useFakeTimers()
    await ble.disconnect()

    await vi.runAllTimersAsync()

    expect(robot.gatt.connects).toBe(1)
    expect(get(ble.status)).toBe('disconnected')
  })
})
