import { derived, type Readable } from 'svelte/store'
import { notifications } from '$lib/components/toasts/notifications'
import type { LinkStatus } from '$lib/interfaces/transport.interface'
import { ble } from '$lib/transport/ble-adapter'
import { websocket } from '$lib/transport/websocket-adapter'
import { dataBroker } from '$lib/transport/databroker'

export const latencyMs = dataBroker.latencyMs

export type TransportKind = 'websocket' | 'bluetooth'

export interface LinkState {
  status: LinkStatus
  transport: TransportKind | null
  latencyMs: number | null
  responsive: boolean
}

export const transportLabels: Record<TransportKind, string> = {
  websocket: 'WiFi',
  bluetooth: 'BLE'
}

export const link: Readable<LinkState> = derived(
  [websocket.status, ble.status, dataBroker.latencyMs],
  ([wsStatus, bleStatus, latency]): LinkState => {
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
        wsStatus === 'connecting' || bleStatus === 'connecting' ? 'connecting' : 'disconnected',
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
