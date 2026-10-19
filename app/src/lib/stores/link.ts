import { derived, type Readable } from 'svelte/store'
import { notifications } from '$lib/components/toasts/notifications'
import type { LinkStatus } from '$lib/interfaces/transport.interface'
import { ble } from '$lib/transport/ble-adapter'
import { websocket } from '$lib/transport/websocket-adapter'
import { serial, serialSupported } from '$lib/transport/serial-adapter'
import { dataBroker } from '$lib/transport/databroker'
import { get } from 'svelte/store'
import { robotData } from '$lib/robot/data'
import { location } from './location-store'

export const latencyMs = dataBroker.latencyMs

export type TransportKind = 'websocket' | 'bluetooth' | 'serial'

export interface LinkState {
  status: LinkStatus
  transport: TransportKind | null
  latencyMs: number | null
  responsive: boolean
}

export const transportLabels: Record<TransportKind, string> = {
  websocket: 'WiFi',
  bluetooth: 'BLE',
  serial: 'USB'
}

export const link: Readable<LinkState> = derived(
  [websocket.status, ble.status, serial.status, dataBroker.latencyMs],
  ([wsStatus, bleStatus, serialStatus, latency]): LinkState => {
    // USB first: it is the only link that carries full-rate telemetry from the https build, so when
    // the cable is in it is the one you meant.
    if (serialStatus === 'connected')
      return {
        status: 'connected',
        transport: 'serial',
        latencyMs: latency,
        responsive: latency !== null
      }

    if (wsStatus === 'connected')
      return {
        status: 'connected',
        transport: 'websocket',
        latencyMs: latency,
        responsive: latency !== null
      }

    if (bleStatus === 'connected')
      return {
        status: 'connected',
        transport: 'bluetooth',
        latencyMs: latency,
        responsive: latency !== null
      }

    return {
      status:
        wsStatus === 'connecting' || bleStatus === 'connecting' || serialStatus === 'connecting' ?
          'connecting'
        : 'disconnected',
      transport: null,
      latencyMs: null,
      responsive: false
    }
  }
)

export const isLinked = derived(link, $link => $link.status === 'connected')

/**
 * Which robot the link reaches, null while no link is up: the transport, and for WiFi the address it was reached at. A
 * change means the robot may be another one.
 */
export const reachedRobot: Readable<string | null> = derived(
  [link, location],
  ([$link, $location]) => {
    if ($link.status !== 'connected') return null
    return $link.transport === 'websocket' ? `websocket ${$location}` : $link.transport
  }
)

export const connectWebsocket = () => {
  websocket.connect().catch(error => notifications.error(`WiFi connect failed: ${error}`))
}

/** Connects the WebSocket to the robot at `address`, which is another robot unless it is the address connected to. */
export const connectToAddress = async (address: string) => {
  const next = address.trim()
  if (next !== get(location)) robotData.reset()
  location.set(next)
  await websocket.disconnect()
  connectWebsocket()
}

// The device chosen may be another robot; a closed chooser chose none.
export const connectBluetooth = () =>
  ble.connect().then(robotData.reset, error => {
    if (error instanceof DOMException && error.name === 'NotFoundError') return
    notifications.error(`Bluetooth connect failed: ${error}`)
  })

export const serialAvailable = serialSupported

const reportSerialError = (error: unknown) => {
  // The port picker resolves with NotFoundError when the user closes it without choosing.
  if (error instanceof DOMException && error.name === 'NotFoundError') return
  notifications.error(`USB connect failed: ${error}`)
}

export const connectSerial = () => serial.connect().catch(reportSerialError)

// The port chosen may lead to another robot.
export const chooseSerialPort = () => serial.choosePort().then(robotData.reset, reportSerialError)
