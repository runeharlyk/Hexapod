import { derived, writable } from 'svelte/store'
import { persistentStore } from '$lib/utilities'

// Stub: the NET_STATUS / NET_COMMAND feature is stashed pending a port to the protobuf schema.
// Exports are kept so the add-robot page compiles.
export interface RobotNetwork {
  staConnected: boolean
  apActive: boolean
  staSsid: string
  staIp: string
  apSsid: string
  apIp: string
}

export const robotNetwork = writable<RobotNetwork | null>(null)
export const preferBluetooth = persistentStore('prefer_bluetooth', false)

export const robotWifiAddress = derived(robotNetwork, $net =>
  $net?.staConnected ? $net.staIp
  : $net?.apActive ? $net.apIp
  : null
)

export const requestNetworkStatus = () => {}
export const forceAccessPoint = () => {}
export const restoreAutomaticAccessPoint = () => {}
export const listenForNetworkStatus = () => {}
