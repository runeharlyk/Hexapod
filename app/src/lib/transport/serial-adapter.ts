import { derived, writable } from 'svelte/store'
import { type ITransport, type LinkStatus } from '../interfaces/transport.interface'

// Framing matches the BLE adapter and firmware/src/communication/serial_adapter.cpp:
// [uint16 LE length][payload].
const MAX_FRAME = 2048

export const serialSupported = () => typeof navigator !== 'undefined' && 'serial' in navigator

function createSerialAdapter(): ITransport {
  const dataCallbacks: ((data: Uint8Array) => void)[] = []
  const connectCallbacks: (() => void)[] = []
  const disconnectCallbacks: (() => void)[] = []
  const status = writable<LinkStatus>('disconnected')
  const connected = derived(status, $s => $s === 'connected')

  let port: SerialPort | undefined
  let writer: WritableStreamDefaultWriter<Uint8Array> | undefined
  let reader: ReadableStreamDefaultReader<Uint8Array> | undefined
  let rx = new Uint8Array(0)

  const handleChunk = (chunk: Uint8Array) => {
    const merged = new Uint8Array(rx.length + chunk.length)
    merged.set(rx)
    merged.set(chunk, rx.length)
    rx = merged

    while (rx.length >= 2) {
      const len = rx[0] | (rx[1] << 8)
      if (len === 0 || len > MAX_FRAME) {
        // The port carries only our frames, so a bad length means we are out of step with the
        // stream rather than reading someone else's data. Drop it and resync.
        rx = new Uint8Array(0)
        return
      }
      if (rx.length < 2 + len) break
      const message = rx.slice(2, 2 + len)
      dataCallbacks.forEach(cb => cb(message))
      rx = rx.slice(2 + len)
    }
  }

  const readLoop = async () => {
    if (!port?.readable) return
    reader = port.readable.getReader()
    try {
      for (;;) {
        const { value, done } = await reader.read()
        if (done) break
        if (value) handleChunk(value)
      }
    } catch {
      // Unplugged mid-read; disconnect() below does the teardown.
    } finally {
      reader.releaseLock()
      reader = undefined
      if (port) await disconnect()
    }
  }

  const connect = async () => {
    if (!serialSupported())
      throw new Error('This browser has no Web Serial support. Try Chrome or Edge.')
    status.set('connecting')
    try {
      // requestPort must be called from a user gesture; the caller is a click handler.
      port = await navigator.serial.requestPort()
      // The ESP32-S3 native USB port is CDC, so the rate is ignored -- but the API demands one.
      await port.open({ baudRate: 921600 })
      writer = port.writable?.getWriter()
      rx = new Uint8Array(0)
      status.set('connected')
      connectCallbacks.forEach(cb => cb())
      void readLoop()
    } catch (e) {
      port = undefined
      status.set('disconnected')
      throw e
    }
  }

  const disconnect = async () => {
    const closing = port
    port = undefined
    try {
      writer?.releaseLock()
      await reader?.cancel()
    } catch {
      // Already gone.
    }
    writer = undefined
    try {
      await closing?.close()
    } catch {
      // Already gone.
    }
    rx = new Uint8Array(0)
    status.set('disconnected')
    disconnectCallbacks.forEach(cb => cb())
  }

  const send = (data: Uint8Array) => {
    if (!writer || data.length > MAX_FRAME) return
    const framed = new Uint8Array(2 + data.length)
    framed[0] = data.length & 0xff
    framed[1] = (data.length >> 8) & 0xff
    framed.set(data, 2)
    void writer.write(framed).catch(() => disconnect())
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

export const serial = createSerialAdapter()
