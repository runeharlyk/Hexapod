import { type Readable } from 'svelte/store'

export type LinkStatus = 'disconnected' | 'connecting' | 'connected'

export interface ITransport {
  status: Readable<LinkStatus>
  connected: Readable<boolean>
  connect: () => Promise<void>
  disconnect: () => Promise<void>
  send: (data: Uint8Array) => void
  /** Sends only when nothing is waiting to go out, so a frame that must be current is dropped rather than queued. */
  trySend: (data: Uint8Array) => boolean
  onData: (cb: (data: Uint8Array) => void) => void
  onConnect: (cb: () => void) => void
  onDisconnect: (cb: () => void) => void
}
