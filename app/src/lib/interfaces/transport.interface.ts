import { type Readable } from 'svelte/store'

export type LinkStatus = 'disconnected' | 'connecting' | 'connected'

export interface ITransport {
  status: Readable<LinkStatus>
  connected: Readable<boolean>
  connect: () => Promise<void>
  disconnect: () => Promise<void>
  send: (data: Uint8Array) => void
  onData: (cb: (data: Uint8Array) => void) => void
  onConnect: (cb: () => void) => void
  onDisconnect: (cb: () => void) => void
}
