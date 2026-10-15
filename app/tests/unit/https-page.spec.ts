// @vitest-environment-options { "url": "https://runeharlyk.github.io/Hexapod/" }
import { describe, it, expect, afterEach, beforeEach, vi } from 'vitest'
import { location } from '../../src/lib/stores/location-store'
import { resolveUrl } from '../../src/lib/proto-api'
import { websocket } from '../../src/lib/transport/websocket-adapter'

const opened: string[] = []

// Stays CONNECTING, like a socket waiting on Chromium's local network access prompt.
class PendingWebSocket {
  onopen: (() => void) | null = null
  onclose: (() => void) | null = null
  onerror: (() => void) | null = null
  onmessage: ((frame: MessageEvent) => void) | null = null
  binaryType = 'blob'
  readyState = 0
  constructor(url: string) {
    opened.push(url)
  }
  close() {}
}

// A browser that refuses ws:// from an https page throws from the constructor.
class RefusingWebSocket {
  constructor(url: string) {
    opened.push(url)
    throw new DOMException('The operation is insecure.', 'SecurityError')
  }
}

describe('the hosted https app', () => {
  beforeEach(() => {
    vi.useFakeTimers()
    opened.length = 0
    location.set('')
  })

  afterEach(async () => {
    await websocket.disconnect()
    vi.useRealTimers()
    vi.unstubAllGlobals()
  })

  it('reaches the robot over http, as it serves no TLS', () => {
    location.set('192.168.1.151')

    expect(resolveUrl('/api/features')).toBe('http://192.168.1.151/api/features')
  })

  it('refuses WiFi without a robot address instead of dialling its own host', async () => {
    vi.stubGlobal('WebSocket', PendingWebSocket)

    await expect(websocket.connect()).rejects.toThrow(/address/)
    expect(opened).toEqual([])
  })

  it("opens the robot's socket once an address is saved", async () => {
    vi.stubGlobal('WebSocket', PendingWebSocket)
    location.set('192.168.1.151')

    await websocket.connect()

    expect(opened).toEqual(['ws://192.168.1.151/api/ws'])
  })

  it('reports a browser that blocks ws:// and stops retrying', async () => {
    vi.stubGlobal('WebSocket', RefusingWebSocket)
    location.set('192.168.1.151')

    await expect(websocket.connect()).rejects.toThrow(/browser blocks/)
    await vi.advanceTimersByTimeAsync(60_000)

    expect(opened).toEqual(['ws://192.168.1.151/api/ws'])
  })
})
