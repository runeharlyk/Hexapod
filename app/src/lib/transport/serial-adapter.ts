import { derived, writable } from 'svelte/store'
import { Message } from '$lib/platform_shared/message'
import { type ITransport, type LinkStatus } from '../interfaces/transport.interface'
import { SERIAL_MAX_MESSAGE, SerialFrameDecoder, encodeSerialFrame } from './serial-framing'
import { LatestFrames } from './latest-frames'

// The ESP32-S3 native USB port is CDC, so the rate is ignored -- but the API demands one.
const BAUD_RATE = 921600
// How often a robot that has not answered yet is pinged; it may still be booting.
export const SERIAL_PROBE_INTERVAL_MS = 1000

const PING_FRAME = encodeSerialFrame(Message.encode(Message.create({ pingmsg: {} })).finish())

export const serialSupported = () => typeof navigator !== 'undefined' && 'serial' in navigator

const sameDevice = (a: SerialPort, b: SerialPort) => {
  const x = a.getInfo()
  const y = b.getInfo()
  return (
    x.usbVendorId !== undefined &&
    x.usbVendorId === y.usbVendorId &&
    x.usbProductId === y.usbProductId
  )
}

// Resolves to `granted` once it is present, or to the same device under a new port: a native USB
// board leaves the bus while it resets and may come back as another port. Needs no user gesture.
const presentPort = (granted: SerialPort, signal: AbortSignal) =>
  new Promise<SerialPort>((resolve, reject) => {
    const matches = (port: SerialPort) => port === granted || sameDevice(port, granted)
    const settle = (outcome: () => void) => {
      navigator.serial.removeEventListener('connect', onConnect)
      signal.removeEventListener('abort', onAbort)
      outcome()
    }
    const onConnect = (event: Event) => {
      const port = event.target as SerialPort
      if (matches(port)) settle(() => resolve(port))
    }
    const onAbort = () => settle(() => reject(signal.reason))
    // Listening before asking, so a port arriving in between is not missed.
    navigator.serial.addEventListener('connect', onConnect)
    signal.addEventListener('abort', onAbort)
    navigator.serial.getPorts().then(ports => {
      const port = ports.find(p => p === granted) ?? ports.find(matches)
      if (port) settle(() => resolve(port))
    }, reject)
  })

// A port this origin was granted before; a picker is needed only when there is none or several.
const grantedPort = async () => {
  const ports = await navigator.serial.getPorts()
  return ports.length === 1 ? ports[0] : undefined
}

export type SerialTransport = ITransport & { choosePort: () => Promise<void> }

// The link counts as connected once the robot answers a ping: a board may reset when its port opens
// and stays silent while it boots, and whatever is sent meanwhile is lost. After a reset or an
// unplug the same device is reopened by itself until the user disconnects.
export function createSerialAdapter(): SerialTransport {
  const dataCallbacks: ((data: Uint8Array) => void)[] = []
  const connectCallbacks: (() => void)[] = []
  const disconnectCallbacks: (() => void)[] = []
  const status = writable<LinkStatus>('disconnected')
  const connected = derived(status, $s => $s === 'connected')

  let port: SerialPort | undefined
  let opened: SerialPort | undefined
  let wanted = false
  let answering = false
  let reader: ReadableStreamDefaultReader<Uint8Array> | undefined
  let reading = Promise.resolve()
  let writeQueue = Promise.resolve()
  // Frames queued and not yet written: each write waits for the one before it.
  let writesInFlight = 0
  const latest = new LatestFrames({
    busy: () => writesInFlight > 0,
    write: data => write(encodeSerialFrame(data))
  })
  let probeId: ReturnType<typeof setInterval> | undefined
  let waiting: AbortController | undefined

  const decoder = new SerialFrameDecoder(message => {
    // The first frame only answers a probe: nothing else is sent before it.
    if (!answering) {
      answering = true
      clearInterval(probeId)
      status.set('connected')
      connectCallbacks.forEach(cb => cb())
      return
    }
    dataCallbacks.forEach(cb => cb(message))
  })

  const write = (frame: Uint8Array) => {
    const target = opened?.writable
    if (!target) return
    writesInFlight++
    writeQueue = writeQueue
      .then(async () => {
        const writer = target.getWriter()
        try {
          await writer.write(frame)
        } finally {
          writer.releaseLock()
        }
      })
      .finally(() => {
        writesInFlight--
        latest.drained()
      })
      .catch(error => console.error('Serial write failed:', error))
  }

  const lose = () => {
    clearInterval(probeId)
    latest.clear()
    const wasAnswering = answering
    answering = false
    decoder.reset()
    if (wasAnswering) disconnectCallbacks.forEach(cb => cb())
  }

  const readUntilLost = async (open: SerialPort) => {
    while (open.readable && opened === open) {
      reader = open.readable.getReader()
      try {
        for (let chunk = await reader.read(); !chunk.done; chunk = await reader.read()) {
          decoder.feed(chunk.value)
        }
        break
      } catch {
        // A framing error hands over a fresh readable; a lost device leaves none.
      } finally {
        reader.releaseLock()
        reader = undefined
      }
    }
    if (opened !== open) return
    opened = undefined
    lose()
    await writeQueue
    await open.close().catch(() => {})
    if (wanted) void reopen()
    else status.set('disconnected')
  }

  const open = async (chosen: SerialPort) => {
    await chosen.open({ baudRate: BAUD_RATE })
    // Asserted, these lines can hold an ESP32 in reset or in its bootloader.
    await chosen.setSignals({ dataTerminalReady: false, requestToSend: false }).catch(() => {})
    opened = chosen
    decoder.reset()
    reading = readUntilLost(chosen)
    write(PING_FRAME)
    probeId = setInterval(() => write(PING_FRAME), SERIAL_PROBE_INTERVAL_MS)
  }

  const reopen = async () => {
    if (!port) return
    status.set('connecting')
    waiting = new AbortController()
    try {
      port = await presentPort(port, waiting.signal)
      if (wanted) await open(port)
    } catch (error) {
      if (!waiting.signal.aborted) console.error('Reopening the serial port failed:', error)
      if (!opened) status.set('disconnected')
    }
  }

  const connectTo = async (choose: boolean) => {
    if (!serialSupported())
      throw new Error('This browser has no Web Serial support. Try Chrome or Edge.')
    if (opened) return
    status.set('connecting')
    try {
      // requestPort must be called from a user gesture; the caller is a click handler.
      const chosen =
        (!choose && (port ?? (await grantedPort()))) || (await navigator.serial.requestPort())
      port = chosen
      wanted = true
      await open(chosen)
    } catch (e) {
      wanted = false
      status.set('disconnected')
      throw e
    }
  }

  const disconnect = async () => {
    wanted = false
    waiting?.abort()
    const closing = opened
    opened = undefined
    lose()
    if (closing) {
      await reader?.cancel().catch(() => {})
      // The read loop must release its lock before the port can close.
      await reading
      await writeQueue
      await closing.close().catch(() => {})
    }
    status.set('disconnected')
  }

  const send = (data: Uint8Array) => {
    if (!answering || data.length > SERIAL_MAX_MESSAGE) return
    write(encodeSerialFrame(data))
  }

  const sendLatest = (kind: number, data: Uint8Array) => {
    if (!answering || !opened?.writable || data.length > SERIAL_MAX_MESSAGE) return false
    latest.offer(kind, data)
    return true
  }

  return {
    status,
    connected,
    connect: () => connectTo(false),
    choosePort: async () => {
      await disconnect()
      port = undefined
      await connectTo(true)
    },
    disconnect,
    send,
    sendLatest,
    onData: cb => dataCallbacks.push(cb),
    onConnect: cb => connectCallbacks.push(cb),
    onDisconnect: cb => disconnectCallbacks.push(cb)
  }
}

export const serial = createSerialAdapter()
