import { encode, decode } from '@msgpack/msgpack'
import { writable } from 'svelte/store'
import {
  MessageTopic,
  MessageType,
  type ITransport,
  type ServerMessage
} from '../interfaces/transport.interface'
import type { DataBrokerCallback } from './databroker'

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
  const connected = writable(false)
  let ws: WebSocket | undefined

  const connect = async () => {
    const location = window.location.host //'192.168.0.221'
    const wsUrl = `ws://${window.location.host}/api/ws/events`
    ws = new WebSocket(wsUrl)
    ws.binaryType = 'arraybuffer'

    ws.onopen = () => {
      ping()
      useBinary = true
      ping()
      connected.set(true)
      connectCallbacks.forEach(cb => cb())
    }

    ws.onclose = () => {
      connected.set(false)
      disconnectCallbacks.forEach(cb => cb())
    }

    ws.onmessage = frame => {
      const message = decodeMessage(frame.data)
      if (!message) return
      const [type, topic = undefined, payload = undefined] = message
      if (topic && payload) dataCallbacks.forEach(cb => cb(type, topic, payload))
    }

    ws.onerror = error => {
      console.error('WebSocket error:', error)
    }
  }

  const disconnect = async () => {
    if (ws) {
      ws.close()
      connected.set(false)
      disconnectCallbacks.forEach(cb => cb())
    }
  }

  const sendEvent = async (type: MessageType, topic?: MessageTopic, payload?: unknown) => {
    const data = [
      type,
      ...(topic !== undefined ? [topic] : []),
      ...(payload !== undefined ? [payload] : [])
    ]
    await send(data)
  }

  const send = async <T>(data: T) => {
    if (!ws || ws.readyState !== WebSocket.OPEN) return
    const serialized = encodeMessage(data)
    if (!serialized) {
      console.error('Could not serialize data:', data)
      return
    }
    ws.send(serialized)
  }

  function ping() {
    const serialized = encodeMessage([3])
    if (!serialized) {
      console.error('Could not serialize message')
      return
    }
    ws?.send(serialized)
  }

  const onData = (data: (type: MessageType, topic: MessageTopic, payload: unknown) => void) =>
    dataCallbacks.push(data)

  const onConnect = (cb: () => void) => connectCallbacks.push(cb)

  const onDisconnect = (cb: () => void) => disconnectCallbacks.push(cb)

  return {
    connected,
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
