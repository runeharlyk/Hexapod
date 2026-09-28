import { derived, get, writable } from 'svelte/store'
import { type ITransport, type LinkStatus } from '../interfaces/transport.interface'
import { location } from '$lib/stores'

const RECONNECT_MIN_MS = 1000
const RECONNECT_MAX_MS = 15000

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

  const openSocket = () => {
    if (ws && (ws.readyState === WebSocket.CONNECTING || ws.readyState === WebSocket.OPEN)) return

    const host = get(location) ? get(location) : window.location.host
    status.set('connecting')

    let socket: WebSocket
    try {
      socket = new WebSocket(`ws://${host}/api/ws`)
    } catch {
      status.set('disconnected')
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
      if (ws !== socket) return
      ws = undefined
      status.set('disconnected')
      disconnectCallbacks.forEach(cb => cb())
      scheduleReconnect()
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
    // Mixed-content blocking refuses ws:// from an https page, so retrying would only fail again.
    if (window.location.protocol === 'https:')
      throw new Error(
        'WiFi needs the robot-hosted http page; this https page can only use BLE or USB'
      )
    wantConnection = true
    reconnectDelay = RECONNECT_MIN_MS
    stopReconnecting()
    openSocket()
  }

  const disconnect = async () => {
    wantConnection = false
    stopReconnecting()
    const socket = ws
    ws = undefined
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

  return {
    status,
    connected,
    connect,
    disconnect,
    send,
    onData: cb => dataCallbacks.push(cb),
    onConnect: cb => connectCallbacks.push(cb),
    onDisconnect: cb => disconnectCallbacks.push(cb)
  }
}

export const websocket = createWebSocketAdapter()
