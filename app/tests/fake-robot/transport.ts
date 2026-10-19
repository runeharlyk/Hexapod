import { derived, writable } from 'svelte/store'
import type { ITransport, LinkStatus } from '../../src/lib/interfaces/transport.interface'
import { FakeRobot, type Client } from './robot'

/**
 * A link to the fake robot as the data broker takes it. The robot's frames arrive on a later microtask, in order, as a
 * real link's do, never inside the send that caused them.
 */
export const fakeLink = (robot: FakeRobot) => {
  const status = writable<LinkStatus>('disconnected')
  const callbacks = {
    data: [] as ((bytes: Uint8Array) => void)[],
    connect: [] as (() => void)[],
    disconnect: [] as (() => void)[]
  }
  let client: Client | undefined
  let hangUp = () => {}

  const transport: ITransport = {
    status,
    connected: derived(status, s => s === 'connected'),
    connect: async () => {
      const connection: Client = {
        deliver: bytes =>
          queueMicrotask(() => {
            if (client === connection) callbacks.data.forEach(cb => cb(bytes))
          })
      }
      client = connection
      hangUp = robot.connect(connection)
      status.set('connected')
      callbacks.connect.forEach(cb => cb())
    },
    disconnect: async () => {
      hangUp()
      client = undefined
      status.set('disconnected')
      callbacks.disconnect.forEach(cb => cb())
    },
    send: bytes => {
      if (client) robot.receive(client, bytes)
    },
    sendLatest: (_kind, bytes) => {
      if (!client) return false
      robot.receive(client, bytes)
      return true
    },
    onData: cb => callbacks.data.push(cb),
    onConnect: cb => callbacks.connect.push(cb),
    onDisconnect: cb => callbacks.disconnect.push(cb)
  }
  return transport
}

export { FakeRobot }
