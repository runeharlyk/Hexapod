import { derived, writable, type Readable } from 'svelte/store'
import { type ITransport, type LinkStatus } from '../interfaces/transport.interface'
import { BleReassembler } from './ble-framing'
import { LatestFrames } from './latest-frames'

export const SERVICE_UUID = '6e400001-b5a3-f393-e0a9-e50e24dcca9e'
const CHARACTERISTIC_TX_UUID = '6e400003-b5a3-f393-e0a9-e50e24dcca9e'
const CHARACTERISTIC_RX_UUID = '6e400002-b5a3-f393-e0a9-e50e24dcca9e'

type BLEAdapter = ITransport & { deviceName: Readable<string | null> }

/** Reconnects tried after a link the user did not end drops, before the link stays down. */
export const BLE_RECONNECT_ATTEMPTS = 5
const RECONNECT_FIRST_MS = 500

function createBLEAdapter(): BLEAdapter {
  const dataCallbacks: ((data: Uint8Array) => void)[] = []
  const connectCallbacks: (() => void)[] = []
  const disconnectCallbacks: (() => void)[] = []
  const status = writable<LinkStatus>('disconnected')
  const deviceName = writable<string | null>(null)
  const connected = derived(status, $s => $s === 'connected')

  let device: BluetoothDevice | undefined
  let rx: BluetoothRemoteGATTCharacteristic | undefined
  let writeQueue = Promise.resolve()
  // Chunks queued and not yet written: each write waits for the one before it.
  let writesInFlight = 0
  const latest = new LatestFrames({ busy: () => writesInFlight > 0, write: data => send(data) })

  // Framing: [uint16 LE length][payload] chunked to the MTU (ble-framing.ts).
  const CHUNK = 180
  const reassembler = new BleReassembler()

  // Whether the connect callbacks ran for the link now up, so the disconnect ones run once for it.
  let linked = false
  // Whether the user wants the link: a link the robot or the radio ends is reconnected, one the user ends is not.
  let wantConnection = false
  let reconnectsLeft = BLE_RECONNECT_ATTEMPTS
  let reconnectTimer: ReturnType<typeof setTimeout> | undefined

  const markDisconnected = () => {
    if (!linked) return
    linked = false
    latest.clear()
    status.set('disconnected')
    deviceName.set(null)
    disconnectCallbacks.forEach(cb => cb())
  }

  const markConnected = () => {
    reconnectsLeft = BLE_RECONNECT_ATTEMPTS
    linked = true
    deviceName.set(device?.name ?? null)
    status.set('connected')
    connectCallbacks.forEach(cb => cb())
  }

  const stopReconnecting = () => {
    clearTimeout(reconnectTimer)
    reconnectTimer = undefined
    reconnectsLeft = BLE_RECONNECT_ATTEMPTS
  }

  // The device chosen once is reconnected without the chooser, which would need a user gesture. The delay doubles
  // from RECONNECT_FIRST_MS, and after BLE_RECONNECT_ATTEMPTS failures the link stays down until the user connects.
  const reconnectLater = () => {
    if (reconnectTimer !== undefined) return
    if (reconnectsLeft === 0) {
      wantConnection = false
      status.set('disconnected')
      console.warn(`BLE reconnect gave up after ${BLE_RECONNECT_ATTEMPTS} attempts`)
      return
    }
    const delay = RECONNECT_FIRST_MS * 2 ** (BLE_RECONNECT_ATTEMPTS - reconnectsLeft)
    reconnectsLeft--
    status.set('connecting')
    reconnectTimer = setTimeout(async () => {
      reconnectTimer = undefined
      if (!wantConnection || !device) return
      try {
        await openLink(device)
      } catch (error) {
        console.warn('BLE reconnect failed:', error)
        if (device.gatt?.connected) device.gatt.disconnect()
        reconnectLater()
        return
      }
      markConnected()
    }, delay)
  }

  // Runs from both an explicit disconnect() and the gattserverdisconnected event it triggers.
  const onLinkLost = () => {
    markDisconnected()
    if (wantConnection) reconnectLater()
  }

  // A stream out of step stays so for the rest of the link; a new link starts it clean, and the broker's subscribes
  // on the reconnect bring every state topic's current value.
  const resync = () => {
    console.warn('BLE stream out of step; reconnecting to resync')
    if (device?.gatt?.connected) device.gatt.disconnect()
    onLinkLost()
  }

  const onValue = (e: Event) => {
    const value = (e.target as BluetoothRemoteGATTCharacteristic).value
    if (!value) return
    const frame = reassembler.push(new Uint8Array(value.buffer, value.byteOffset, value.byteLength))
    if (frame === 'partial') return
    if (frame === 'out-of-step') return resync()
    dataCallbacks.forEach(cb => cb(frame))
  }

  const openLink = async (chosen: BluetoothDevice) => {
    if (!chosen.gatt) throw new Error('GATT not available')
    const server = await chosen.gatt.connect()
    const service = await server.getPrimaryService(SERVICE_UUID)
    const tx = await service.getCharacteristic(CHARACTERISTIC_TX_UUID)
    rx = await service.getCharacteristic(CHARACTERISTIC_RX_UUID)
    reassembler.reset()
    tx.addEventListener('characteristicvaluechanged', onValue)
    chosen.addEventListener('gattserverdisconnected', onLinkLost)
    await tx.startNotifications()
  }

  const connect = async () => {
    if (!navigator.bluetooth) throw new Error('Web Bluetooth API is not available')
    wantConnection = false
    stopReconnecting()
    status.set('connecting')

    try {
      device = await navigator.bluetooth.requestDevice({
        filters: [{ services: [SERVICE_UUID] }],
        optionalServices: [SERVICE_UUID]
      })
      await openLink(device)
    } catch (error) {
      if (device?.gatt?.connected) device.gatt.disconnect()
      status.set('disconnected')
      throw error
    }

    wantConnection = true
    markConnected()
  }

  const disconnect = async () => {
    wantConnection = false
    stopReconnecting()
    if (device?.gatt?.connected) device.gatt.disconnect()
    markDisconnected()
    status.set('disconnected')
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
      writesInFlight++
      writeQueue = writeQueue
        .then(() =>
          typeof characteristic.writeValueWithoutResponse === 'function' ?
            characteristic.writeValueWithoutResponse(chunk)
          : characteristic.writeValue(chunk)
        )
        .catch(err => console.error('BLE write error:', err))
        .finally(() => {
          writesInFlight--
          latest.drained()
        })
    }
  }

  const sendLatest = (kind: string, data: Uint8Array) => {
    if (!rx || !device?.gatt?.connected) return false
    latest.offer(kind, data)
    return true
  }

  return {
    status,
    connected,
    deviceName,
    connect,
    disconnect,
    send,
    sendLatest,
    onData: cb => dataCallbacks.push(cb),
    onConnect: cb => connectCallbacks.push(cb),
    onDisconnect: cb => disconnectCallbacks.push(cb)
  }
}

export const ble = createBLEAdapter()
