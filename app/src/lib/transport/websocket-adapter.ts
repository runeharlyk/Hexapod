import { derived, get, writable } from 'svelte/store'
import { type ITransport, type LinkStatus } from '../interfaces/transport.interface'
import { canReachRobot, location } from '$lib/stores/location-store'
import { LatestFrames } from './latest-frames'

const RECONNECT_MIN_MS = 1000
const RECONNECT_MAX_MS = 15000
// The browser reports no event when the socket's buffer empties, so a frame waiting for it looks again this often.
const DRAIN_POLL_MS = 10

export const socketUrl = (host: string) => `ws://${host}/api/ws`

function createWebSocketAdapter(): ITransport {
  const dataCallbacks: ((data: Uint8Array) => void)[] = []
  const connectCallbacks: (() => void)[] = []
  const disconnectCallbacks: (() => void)[] = []
  const status = writable<LinkStatus>('disconnected')
  const connected = derived(status, $s => $s === 'connected')

  let ws: WebSocket | undefined
  let wantConnection = false
  let reconnectDelay = RECONNECT_MIN_MS
  let reconnectTimer: ReturnType<typeof setTimeout> | undefined
  let refusedByBrowser = false
  let drainTimer: ReturnType<typeof setTimeout> | undefined

  const latest = new LatestFrames({
    busy: () => (ws?.bufferedAmount ?? 0) > 0,
    write: data => ws?.send(data)
  })

  const watchDrain = () => {
    if (drainTimer || !latest.holding) return
    drainTimer = setTimeout(() => {
      drainTimer = undefined
      latest.drained()
      watchDrain()
    }, DRAIN_POLL_MS)
  }

  const forgetWaiting = () => {
    clearTimeout(drainTimer)
    drainTimer = undefined
    latest.clear()
  }

  const stopReconnecting = () => {
    if (reconnectTimer) clearTimeout(reconnectTimer)
    reconnectTimer = undefined
  }

  const scheduleReconnect = () => {
    if (!wantConnection || reconnectTimer) return
    const delay = reconnectDelay
    reconnectDelay = Math.min(reconnectDelay * 2, RECONNECT_MAX_MS)
    reconnectTimer = setTimeout(() => {
      reconnectTimer = undefined
      openSocket()
    }, delay)
  }

  // The link's end, once per socket: a frame waiting for this socket was for it alone.
  const lose = (socket: WebSocket) => {
    if (ws !== socket) return false
    ws = undefined
    forgetWaiting()
    status.set('disconnected')
    disconnectCallbacks.forEach(cb => cb())
    return true
  }

  const openSocket = () => {
    if (ws && (ws.readyState === WebSocket.CONNECTING || ws.readyState === WebSocket.OPEN)) return
    // A socket still closing ends here, since its close event will find another in its place.
    if (ws) lose(ws)

    const host = get(location) ? get(location) : window.location.host
    status.set('connecting')

    let socket: WebSocket
    try {
      socket = new WebSocket(socketUrl(host))
    } catch (error) {
      status.set('disconnected')
      // A browser without Chromium's local network exemption may refuse ws:// from an https page here;
      // retrying would only fail again.
      if (error instanceof DOMException && error.name === 'SecurityError') {
        wantConnection = false
        refusedByBrowser = true
        return
      }
      scheduleReconnect()
      return
    }

    ws = socket
    socket.binaryType = 'arraybuffer'

    socket.onopen = () => {
      reconnectDelay = RECONNECT_MIN_MS
      status.set('connected')
      connectCallbacks.forEach(cb => cb())
    }

    socket.onclose = () => {
      if (lose(socket)) scheduleReconnect()
    }

    socket.onmessage = frame => {
      if (!(frame.data instanceof ArrayBuffer)) return
      const bytes = new Uint8Array(frame.data)
      dataCallbacks.forEach(cb => cb(bytes))
    }

    socket.onerror = () => {
      /* onclose handles teardown + reconnect */
    }
  }

  const connect = async () => {
    if (!canReachRobot(new URL(window.location.href), get(location)))
      throw new Error("WiFi from this page needs the robot's address; enter it, or use BLE or USB")
    wantConnection = true
    refusedByBrowser = false
    reconnectDelay = RECONNECT_MIN_MS
    stopReconnecting()
    openSocket()
    if (refusedByBrowser)
      throw new Error(
        'this browser blocks WiFi from an https page; use Chrome or Edge, or BLE or USB'
      )
  }

  const disconnect = async () => {
    wantConnection = false
    stopReconnecting()
    const socket = ws
    ws = undefined
    forgetWaiting()
    status.set('disconnected')
    if (socket) {
      socket.close()
      disconnectCallbacks.forEach(cb => cb())
    }
  }

  const send = (data: Uint8Array) => {
    if (!ws || ws.readyState !== WebSocket.OPEN) return
    ws.send(data)
  }

  const sendLatest = (kind: string, data: Uint8Array) => {
    if (!ws || ws.readyState !== WebSocket.OPEN) return false
    latest.offer(kind, data)
    watchDrain()
    return true
  }

  return {
    status,
    connected,
    connect,
    disconnect,
    send,
    sendLatest,
    onData: cb => dataCallbacks.push(cb),
    onConnect: cb => connectCallbacks.push(cb),
    onDisconnect: cb => disconnectCallbacks.push(cb)
  }
}

export const websocket = createWebSocketAdapter()
