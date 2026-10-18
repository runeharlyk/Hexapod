import type { LinkStatus } from '$lib/interfaces/transport.interface'

export type WifiAction = 'connect' | 'stop' | 'disconnect'

export const linkStatusLabels: Record<LinkStatus, string> = {
  connected: 'Connected',
  connecting: 'Connecting',
  disconnected: 'Disconnected'
}

// The address field is a draft: the socket keeps reconnecting to the address it was opened with, so
// a different address in the field means switching robots rather than leaving the current one.
export const wifiAction = (status: LinkStatus, target: string, draft: string): WifiAction => {
  if (draft.trim() !== target) return 'connect'
  if (status === 'connected') return 'disconnect'
  if (status === 'connecting') return 'stop'
  return 'connect'
}
