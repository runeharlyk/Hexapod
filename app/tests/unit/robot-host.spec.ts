// The host the app reaches the robot at (src/lib/stores/robot-host.ts): a saved robot's name that does not open is
// followed by the IP the robot last reported, in the WebSocket adapter, and the HTTP paths use whichever host opened.
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { get } from 'svelte/store'

class FakeSocket {
  static readonly CONNECTING = 0
  static readonly OPEN = 1
  static readonly CLOSING = 2
  static readonly CLOSED = 3
  static all: FakeSocket[] = []
  readyState = FakeSocket.CONNECTING
  bufferedAmount = 0
  binaryType = 'blob'
  onopen: (() => void) | null = null
  onclose: (() => void) | null = null
  onerror: (() => void) | null = null
  onmessage: ((event: { data: unknown }) => void) | null = null
  constructor(readonly url: string) {
    FakeSocket.all.push(this)
  }
  send() {}
  close() {
    this.readyState = FakeSocket.CLOSED
  }
  open() {
    this.readyState = FakeSocket.OPEN
    this.onopen?.()
  }
  end() {
    this.readyState = FakeSocket.CLOSED
    this.onclose?.()
  }
}

const IP = '192.168.1.50'

const load = async (saved: { address: string; lastIp?: string }[]) => {
  vi.resetModules()
  const { robots } = await import('../../src/lib/stores/robots')
  robots.set(saved.map(r => ({ name: r.address, lastSeenAt: null, robotId: 'aaaaaaaaaaaa', ...r })))
  const { location } = await import('../../src/lib/stores/location-store')
  const host = await import('../../src/lib/stores/robot-host')
  const { websocket } = await import('../../src/lib/transport/websocket-adapter')
  const { resolveUrl } = await import('../../src/lib/proto-api')
  return { location, ...host, websocket, resolveUrl }
}

const urls = () => FakeSocket.all.map(s => s.url)

describe('the saved robot fallback', () => {
  beforeEach(() => {
    vi.useFakeTimers()
    FakeSocket.all = []
    vi.stubGlobal('WebSocket', FakeSocket)
  })

  afterEach(() => {
    vi.useRealTimers()
    vi.unstubAllGlobals()
  })

  it('after a failed open of the name the reconnects alternate name and lastIp', async () => {
    const { location, websocket } = await load([{ address: 'hexapod.local', lastIp: IP }])
    location.set('hexapod.local')

    await websocket.connect()
    for (let attempt = 0; attempt < 4; attempt++) {
      FakeSocket.all[attempt].end()
      await vi.advanceTimersByTimeAsync(15000)
    }

    expect(urls()).toEqual([
      'ws://hexapod.local/api/ws',
      `ws://${IP}/api/ws`,
      'ws://hexapod.local/api/ws',
      `ws://${IP}/api/ws`,
      'ws://hexapod.local/api/ws'
    ])
    await websocket.disconnect()
  })

  it('keeps retrying the name alone for a robot with no last IP', async () => {
    const { location, websocket } = await load([{ address: 'hexapod.local' }])
    location.set('hexapod.local')

    await websocket.connect()
    FakeSocket.all[0].end()
    await vi.advanceTimersByTimeAsync(15000)
    FakeSocket.all[1].end()
    await vi.advanceTimersByTimeAsync(15000)

    expect(urls()).toEqual(Array(3).fill('ws://hexapod.local/api/ws'))
    await websocket.disconnect()
  })

  it('does not swap hosts after a socket that opened', async () => {
    const { location, fallbackHost, websocket } = await load([
      { address: 'hexapod.local', lastIp: IP }
    ])
    location.set('hexapod.local')
    await websocket.connect()
    FakeSocket.all[0].end()
    await vi.advanceTimersByTimeAsync(15000)
    FakeSocket.all[1].open()

    FakeSocket.all[1].end()
    await vi.advanceTimersByTimeAsync(15000)

    expect(urls()).toEqual(['ws://hexapod.local/api/ws', `ws://${IP}/api/ws`, `ws://${IP}/api/ws`])
    expect(get(fallbackHost)).toBe(IP)
    await websocket.disconnect()
  })

  it('does not count a socket the user closed as a failure of the name', async () => {
    const { location, fallbackHost, websocket } = await load([
      { address: 'hexapod.local', lastIp: IP }
    ])
    location.set('hexapod.local')
    await websocket.connect()

    await websocket.disconnect()
    FakeSocket.all[0].end()
    await vi.advanceTimersByTimeAsync(15000)

    expect(get(fallbackHost)).toBeNull()
    expect(FakeSocket.all).toHaveLength(1)
  })

  it('a new location clears the fallback', async () => {
    const { location, fallbackHost, robotHost } = await load([
      { address: 'hexapod.local', lastIp: IP }
    ])
    location.set('hexapod.local')
    fallbackHost.set(IP)
    expect(get(robotHost)).toBe(IP)

    location.set('esp32.local')

    expect(get(fallbackHost)).toBeNull()
    expect(get(robotHost)).toBe('esp32.local')
  })

  it('serves the page host while no robot is saved at all', async () => {
    const { robotHost } = await load([])

    expect(get(robotHost)).toBe(window.location.host)
  })

  it('resolveUrl uses the host the socket reached', async () => {
    const { location, websocket, resolveUrl } = await load([
      { address: 'hexapod.local', lastIp: IP }
    ])
    location.set('hexapod.local')
    await websocket.connect()
    expect(resolveUrl('/api/features')).toBe('http://hexapod.local/api/features')

    FakeSocket.all[0].end()
    await vi.advanceTimersByTimeAsync(15000)
    FakeSocket.all[1].open()

    expect(resolveUrl('/api/features')).toBe(`http://${IP}/api/features`)
    expect(resolveUrl('http://elsewhere/x')).toBe('http://elsewhere/x')
    await websocket.disconnect()
  })

  it('leaves a robot-hosted page, whose location is empty, with its own relative paths', async () => {
    const { resolveUrl } = await load([])

    expect(resolveUrl('/api/features')).toBe('/api/features')
  })
})
