import { encode, decode } from '@msgpack/msgpack'
import { derived, writable } from 'svelte/store'
import {
  MessageTopic,
  MessageType,
  type ITransport,
  type LinkStatus,
  type ServerMessage
} from '../interfaces/transport.interface'
import type { DataBrokerCallback } from './databroker'

export const SERVICE_UUID = '6e400001-b5a3-f393-e0a9-e50e24dcca9e'
const CHARACTERISTIC_TX_UUID = '6e400003-b5a3-f393-e0a9-e50e24dcca9e'
const CHARACTERISTIC_RX_UUID = '6e400002-b5a3-f393-e0a9-e50e24dcca9e'

const PING_INTERVAL_MS = 2000
const PONG_TIMEOUT_MS = 6000

function createBLEAdapter(): ITransport {
  const dataCallbacks: DataBrokerCallback<unknown>[] = []
  const connectCallbacks: (() => void)[] = []
  const disconnectCallbacks: (() => void)[] = []
  const status = writable<LinkStatus>('disconnected')
  const latencyMs = writable<number | null>(null)
  const connected = derived(status, $status => $status === 'connected')
  let device: BluetoothDevice | undefined
  let server: BluetoothRemoteGATTServer | undefined
  let service: BluetoothRemoteGATTService | undefined
  let tx: BluetoothRemoteGATTCharacteristic | undefined
  let rx: BluetoothRemoteGATTCharacteristic | undefined
  let writeQueue = Promise.resolve()
  let pingTimer: ReturnType<typeof setInterval> | undefined
  let lastPingAt = 0
  let lastPongAt = 0

  const stopPinging = () => {
    if (pingTimer) clearInterval(pingTimer)
    pingTimer = undefined
  }

  const startPinging = () => {
    stopPinging()
    lastPongAt = performance.now()
    pingTimer = setInterval(() => {
      if (!device?.gatt?.connected) return
      if (performance.now() - lastPongAt > PONG_TIMEOUT_MS) latencyMs.set(null)
      lastPingAt = performance.now()
      sendEvent(MessageType.PING)
    }, PING_INTERVAL_MS)
  }

  const markDisconnected = () => {
    stopPinging()
    latencyMs.set(null)
    status.set('disconnected')
    disconnectCallbacks.forEach(cb => cb())
  }

  const connect = async () => {
    if (!navigator.bluetooth) {
      throw new Error('Web Bluetooth API is not available')
    }

    status.set('connecting')

    try {
      device = await navigator.bluetooth.requestDevice({
        filters: [{ services: [SERVICE_UUID] }],
        optionalServices: [SERVICE_UUID]
      })

      if (!device?.gatt) {
        throw new Error('GATT service not available')
      }

      server = await device.gatt.connect()
      service = await server?.getPrimaryService(SERVICE_UUID)
      tx = await service?.getCharacteristic(CHARACTERISTIC_TX_UUID)
      rx = await service?.getCharacteristic(CHARACTERISTIC_RX_UUID)
      await tx?.startNotifications()
    } catch (error) {
      status.set('disconnected')
      throw error
    }

    tx?.addEventListener('characteristicvaluechanged', e => {
      const data = decode(new Uint8Array(e.target.value.buffer)) as ServerMessage
      const [type, topic, payload] = data
      if (type === MessageType.PONG) {
        lastPongAt = performance.now()
        latencyMs.set(Math.max(0, Math.round(lastPongAt - lastPingAt)))
        return
      }
      if (topic !== undefined && payload !== undefined) {
        dataCallbacks.forEach(cb => cb(type, topic, payload))
      }
    })

    device.addEventListener('gattserverdisconnected', markDisconnected)

    status.set('connected')
    startPinging()
    connectCallbacks.forEach(cb => cb())
  }

  const disconnect = async () => {
    if (device?.gatt?.connected) {
      await device.gatt.disconnect()
      markDisconnected()
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

  const send = async <T>(data: T, reliable = false) => {
    if (!rx || !device?.gatt?.connected) return

    const payload = encode(data)
    const writeTask = writeQueue.then(async () => {
      if (!rx || !device?.gatt?.connected) return

      try {
        // Use writeValueWithoutResponse for faster throughput if supported and reliable delivery is not requested
        if (!reliable && typeof rx.writeValueWithoutResponse === 'function') {
          await rx.writeValueWithoutResponse(payload)
        } else {
          await rx.writeValue(payload)
        }
      } catch (err) {
        console.error('BLE Write Error:', err)
      }
    })

    writeQueue = writeTask.catch(() => undefined)
    await writeTask
  }

  const onData = (data: (type: MessageType, topic: MessageTopic, payload: unknown) => void) => {
    dataCallbacks.push(data)
  }

  const onConnect = (cb: () => void) => {
    connectCallbacks.push(cb)
  }

  const onDisconnect = (cb: () => void) => {
    disconnectCallbacks.push(cb)
  }

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

export const ble = createBLEAdapter()
