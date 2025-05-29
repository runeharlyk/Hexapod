import msgpack from 'msgpack-lite'
import { writable } from 'svelte/store'
import { MessageTopic, MessageType, type ITransport } from '../interfaces/transport.interface'
import type { DataBrokerCallback } from './databroker'

const SERVICE_UUID = '6e400001-b5a3-f393-e0a9-e50e24dcca9e'
const CHARACTERISTIC_TX_UUID = '6e400003-b5a3-f393-e0a9-e50e24dcca9e'
const CHARACTERISTIC_RX_UUID = '6e400002-b5a3-f393-e0a9-e50e24dcca9e'

function createBLEAdapter(): ITransport {
  const dataCallbacks: DataBrokerCallback<unknown>[] = []
  const connectCallbacks: (() => void)[] = []
  const disconnectCallbacks: (() => void)[] = []
  const connected = writable(false)
  let device: BluetoothDevice | undefined
  let server: BluetoothRemoteGATTServer | undefined
  let service: BluetoothRemoteGATTService | undefined
  let tx: BluetoothRemoteGATTCharacteristic | undefined
  let rx: BluetoothRemoteGATTCharacteristic | undefined

  const connect = async () => {
    if (!navigator.bluetooth) {
      throw new Error('Web Bluetooth API is not available')
    }

    device = await navigator.bluetooth.requestDevice({
      acceptAllDevices: true,
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

    tx?.addEventListener('characteristicvaluechanged', e => {
      const data = msgpack.decode(new Uint8Array(e.target.value.buffer))
      const [type, topic, payload] = data

      dataCallbacks.forEach(cb => cb(type, topic, payload))
    })

    connected.set(true)
    connectCallbacks.forEach(cb => cb())
  }

  const disconnect = async () => {
    if (device?.gatt?.connected) {
      await device.gatt.disconnect()
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
    console.log('BLE send', data)
    const payload = msgpack.encode(data)
    const decoded = msgpack.decode(payload)
    console.log('BLE send decoded', decoded)
    await rx?.writeValue(payload)
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
