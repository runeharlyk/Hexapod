import { derived, type Readable } from 'svelte/store'
import { notifications } from '$lib/components/toasts/notifications'
import type { LinkStatus } from '$lib/interfaces/transport.interface'
import { ble } from '$lib/transport/ble-adapter'
import { websocket } from '$lib/transport/websocket-adapter'
import { serial, serialSupported } from '$lib/transport/serial-adapter'
import { dataBroker } from '$lib/transport/databroker'

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

export const connectWebsocket = () => {
  websocket.connect().catch(error => notifications.error(`WiFi connect failed: ${error}`))
}

export const connectBluetooth = () => {
  ble.connect().catch(error => {
    if (error instanceof DOMException && error.name === 'NotFoundError') return
    notifications.error(`Bluetooth connect failed: ${error}`)
  })
}

export const serialAvailable = serialSupported

export const connectSerial = () => {
  serial.connect().catch(error => {
    // The port picker resolves with NotFoundError when the user closes it without choosing.
    if (error instanceof DOMException && error.name === 'NotFoundError') return
    notifications.error(`USB connect failed: ${error}`)
  })
}
