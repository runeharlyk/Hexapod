import { encode, decode } from '@msgpack/msgpack'
import { writable } from 'svelte/store'
import { MessageTopic, MessageType, type ITransport } from '../interfaces/transport.interface'
import type { DataBrokerCallback } from './databroker'

type Message = [MessageType, MessageTopic?, unknown?]

function createWebSocketAdapter(): ITransport {
  const dataCallbacks: DataBrokerCallback<unknown>[] = []
  const connectCallbacks: (() => void)[] = []
  const disconnectCallbacks: (() => void)[] = []
  const connected = writable(false)
  let ws: WebSocket | undefined

  const connect = async () => {
    const location = '192.168.0.221'
    const wsUrl = `ws://${location}/api/ws/events`
    ws = new WebSocket(wsUrl)

    ws.onopen = () => {
      connected.set(true)
      connectCallbacks.forEach(cb => cb())
    }

    ws.onclose = () => {
      connected.set(false)
      disconnectCallbacks.forEach(cb => cb())
    }

    ws.onmessage = event => {
      if (event.data instanceof ArrayBuffer) {
        const data = decode(new Uint8Array(event.data)) as Message
        const [type, topic, payload] = data
        if (topic && payload) dataCallbacks.forEach(cb => cb(type, topic, payload))
      } else {
        try {
          const data = JSON.parse(event.data) as Message
          const [type, topic, payload] = data
          if (topic && payload) dataCallbacks.forEach(cb => cb(type, topic, payload))
        } catch (error) {
          console.error('Failed to parse websocket message:', error)
        }
      }
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

    const payload = encode(data)
    ws.send(payload)
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
