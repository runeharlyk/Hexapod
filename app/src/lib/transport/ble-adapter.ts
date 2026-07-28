import { derived, writable } from 'svelte/store'
import { type ITransport, type LinkStatus } from '../interfaces/transport.interface'

export const SERVICE_UUID = '6e400001-b5a3-f393-e0a9-e50e24dcca9e'
const CHARACTERISTIC_TX_UUID = '6e400003-b5a3-f393-e0a9-e50e24dcca9e'
const CHARACTERISTIC_RX_UUID = '6e400002-b5a3-f393-e0a9-e50e24dcca9e'

function createBLEAdapter(): ITransport {
  const dataCallbacks: ((data: Uint8Array) => void)[] = []
  const connectCallbacks: (() => void)[] = []
  const disconnectCallbacks: (() => void)[] = []
  const status = writable<LinkStatus>('disconnected')
  const connected = derived(status, $s => $s === 'connected')

  let device: BluetoothDevice | undefined
  let rx: BluetoothRemoteGATTCharacteristic | undefined
  let writeQueue = Promise.resolve()

  // Framing: [uint16 LE length][payload] chunked to the MTU; rxBuffer reassembles by length prefix.
  const CHUNK = 180
  let rxBuffer = new Uint8Array(0)

  const handleChunk = (chunk: Uint8Array) => {
    const merged = new Uint8Array(rxBuffer.length + chunk.length)
    merged.set(rxBuffer)
    merged.set(chunk, rxBuffer.length)
    rxBuffer = merged
    while (rxBuffer.length >= 2) {
      const len = rxBuffer[0] | (rxBuffer[1] << 8)
      if (rxBuffer.length < 2 + len) break
      const message = rxBuffer.slice(2, 2 + len)
      dataCallbacks.forEach(cb => cb(message))
      rxBuffer = rxBuffer.slice(2 + len)
    }
  }

  const markDisconnected = () => {
    status.set('disconnected')
    disconnectCallbacks.forEach(cb => cb())
  }

  const connect = async () => {
    if (!navigator.bluetooth) throw new Error('Web Bluetooth API is not available')
    status.set('connecting')

    try {
      device = await navigator.bluetooth.requestDevice({
        filters: [{ services: [SERVICE_UUID] }],
        optionalServices: [SERVICE_UUID]
      })
      if (!device?.gatt) throw new Error('GATT not available')

      const server = await device.gatt.connect()
      const service = await server.getPrimaryService(SERVICE_UUID)
      const tx = await service.getCharacteristic(CHARACTERISTIC_TX_UUID)
      rx = await service.getCharacteristic(CHARACTERISTIC_RX_UUID)
      await tx.startNotifications()
      rxBuffer = new Uint8Array(0)

      tx.addEventListener('characteristicvaluechanged', e => {
        const value = (e.target as BluetoothRemoteGATTCharacteristic).value
        if (!value) return
        handleChunk(new Uint8Array(value.buffer))
      })
      device.addEventListener('gattserverdisconnected', markDisconnected)
    } catch (error) {
      status.set('disconnected')
      throw error
    }

    status.set('connected')
    connectCallbacks.forEach(cb => cb())
  }

  const disconnect = async () => {
    if (device?.gatt?.connected) {
      await device.gatt.disconnect()
      markDisconnected()
    }
  }

  const send = (data: Uint8Array) => {
    if (!rx || !device?.gatt?.connected) return
    const characteristic = rx
    const framed = new Uint8Array(2 + data.length)
    framed[0] = data.length & 0xff
    framed[1] = (data.length >> 8) & 0xff
    framed.set(data, 2)
    for (let offset = 0; offset < framed.length; offset += CHUNK) {
      const chunk = framed.slice(offset, offset + CHUNK)
      writeQueue = writeQueue
        .then(() =>
          typeof characteristic.writeValueWithoutResponse === 'function' ?
            characteristic.writeValueWithoutResponse(chunk)
          : characteristic.writeValue(chunk)
        )
        .catch(err => console.error('BLE write error:', err))
    }
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

export const ble = createBLEAdapter()
