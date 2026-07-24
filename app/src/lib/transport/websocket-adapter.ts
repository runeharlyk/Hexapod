import { encode, decode } from '@msgpack/msgpack'
import { derived, get, writable } from 'svelte/store'
import {
  MessageTopic,
  MessageType,
  type ITransport,
  type LinkStatus,
  type ServerMessage
} from '../interfaces/transport.interface'
import type { DataBrokerCallback } from './databroker'
import { location } from '$lib/stores'

const PING_INTERVAL_MS = 2000
const PONG_TIMEOUT_MS = 6000
const RECONNECT_MIN_MS = 1000
const RECONNECT_MAX_MS = 15000

let useBinary = false

const decodeMessage = (data: string | ArrayBuffer): ServerMessage | null => {
  useBinary = data instanceof ArrayBuffer

  try {
    if (useBinary) {
      return decode(new Uint8Array(data as ArrayBuffer)) as ServerMessage
    }
    return JSON.parse(data as string)
  } catch (error) {
    console.error(`Could not decode data: ${new Uint8Array(data as ArrayBuffer)} - ${error}`)
  }
  return null
}

const encodeMessage = (data: unknown) => {
  try {
    return useBinary ? encode(data) : JSON.stringify(data)
  } catch (error) {
    console.error(`Could not encode data: ${data} - ${error}`)
  }
}

function createWebSocketAdapter(): ITransport {
  const dataCallbacks: DataBrokerCallback<unknown>[] = []
  const connectCallbacks: (() => void)[] = []
  const disconnectCallbacks: (() => void)[] = []
  const status = writable<LinkStatus>('disconnected')
  const latencyMs = writable<number | null>(null)
  const connected = derived(status, $status => $status === 'connected')
  let hasEnabledProtocol = false
  let ws: WebSocket | undefined
  let wantConnection = false
  let reconnectDelay = RECONNECT_MIN_MS
  let reconnectTimer: ReturnType<typeof setTimeout> | undefined
  let pingTimer: ReturnType<typeof setInterval> | undefined
  let lastPingAt = 0
  let lastPongAt = 0

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

  const stopPinging = () => {
    if (pingTimer) clearInterval(pingTimer)
    pingTimer = undefined
  }

  const startPinging = () => {
    stopPinging()
    lastPongAt = performance.now()
    pingTimer = setInterval(() => {
      if (!ws || ws.readyState !== WebSocket.OPEN) return
      if (performance.now() - lastPongAt > PONG_TIMEOUT_MS) latencyMs.set(null)
      ping()
    }, PING_INTERVAL_MS)
  }

  // The firmware answers a PING in whichever encoding it was built with, so probing with
  // both lets the reply settle `useBinary` through decodeMessage.
  const probeEncoding = () => {
    ping()
    useBinary = true
    ping()
    useBinary = false
  }

  const openSocket = () => {
    if (ws && (ws.readyState === WebSocket.CONNECTING || ws.readyState === WebSocket.OPEN)) return

    const wsLocation = get(location) ? get(location) : window.location.host
    status.set('connecting')

    let socket: WebSocket
    try {
      socket = new WebSocket(`ws://${wsLocation}/api/ws`)
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
      probeEncoding()
      startPinging()
    }

    socket.onclose = () => {
      if (ws !== socket) return
      ws = undefined
      hasEnabledProtocol = false
      stopPinging()
      latencyMs.set(null)
      status.set('disconnected')
      disconnectCallbacks.forEach(cb => cb())
      scheduleReconnect()
    }

    socket.onmessage = frame => {
      const message = decodeMessage(frame.data)
      if (!message) return
      const [type, topic = undefined, payload = undefined] = message
      if (!hasEnabledProtocol) {
        hasEnabledProtocol = true
        connectCallbacks.forEach(cb => cb())
      }
      if (type === MessageType.PONG) {
        lastPongAt = performance.now()
        latencyMs.set(Math.max(0, Math.round(lastPongAt - lastPingAt)))
        return
      }
      if (topic !== undefined && payload !== undefined) {
        dataCallbacks.forEach(cb => cb(type, topic, payload))
      }
    }

    socket.onerror = () => {
      hasEnabledProtocol = false
    }
  }

  const connect = async () => {
    wantConnection = true
    reconnectDelay = RECONNECT_MIN_MS
    stopReconnecting()
    openSocket()
  }

  const disconnect = async () => {
    wantConnection = false
    stopReconnecting()
    stopPinging()
    hasEnabledProtocol = false
    latencyMs.set(null)
    const socket = ws
    ws = undefined
    status.set('disconnected')
    if (socket) {
      socket.close()
      disconnectCallbacks.forEach(cb => cb())
    }
  }

  const sendEvent = async (
    type: MessageType,
    topic?: MessageTopic,
    payload?: unknown,
    reliable?: boolean
  ) => {
    const data = [
      type,
      ...(topic !== undefined ? [topic] : []),
      ...(payload !== undefined ? [payload] : [])
    ]
    await send(data, reliable)
  }

  const send = async <T>(data: T, reliable?: boolean) => {
    if (!ws || ws.readyState !== WebSocket.OPEN) return
    const serialized = encodeMessage(data)
    if (!serialized) {
      console.error('Could not serialize data:', data)
      return
    }
    ws.send(serialized)
  }

  function ping() {
    const serialized = encodeMessage([MessageType.PING])
    if (!serialized) {
      console.error('Could not serialize message')
      return
    }
    lastPingAt = performance.now()
    ws?.send(serialized)
  }

  const onData = (data: (type: MessageType, topic: MessageTopic, payload: unknown) => void) =>
    dataCallbacks.push(data)

  const onConnect = (cb: () => void) => connectCallbacks.push(cb)

  const onDisconnect = (cb: () => void) => disconnectCallbacks.push(cb)

  return {
    connected,
    status,
    latencyMs,
    connect,
    disconnect,
    send,
    sendEvent,
    onData,
    onConnect,
    onDisconnect
  }
}

export const websocket = createWebSocketAdapter()
