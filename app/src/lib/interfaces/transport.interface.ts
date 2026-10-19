import { type Readable } from 'svelte/store'

export type LinkStatus = 'disconnected' | 'connecting' | 'connected'

export interface ITransport {
  status: Readable<LinkStatus>
  connected: Readable<boolean>
  connect: () => Promise<void>
  disconnect: () => Promise<void>
  send: (data: Uint8Array) => void
  /**
   * A frame that must be current (transport/latest-frames.ts): sent at once when nothing is being written, else only
   * the newest of its `kind` goes out once the writes under way complete. False, and nothing kept, while the link is
   * down.
   */
  sendLatest: (kind: number, data: Uint8Array) => boolean
  onData: (cb: (data: Uint8Array) => void) => void
  onConnect: (cb: () => void) => void
  onDisconnect: (cb: () => void) => void
}
