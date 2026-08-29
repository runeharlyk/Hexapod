import { writable } from 'svelte/store'
import type { ITransport } from '$lib/interfaces/transport.interface'
import {
  CorrelationRequest,
  type CorrelationResponse,
  Message,
  PingMsg,
  SubscribeNotification,
  UnsubscribeNotification,
  type MessageFns
} from '$lib/platform_shared/message'
import { decodeMessage, encodeMessage, keyOf, tagOf } from './message-codec'

const PING_INTERVAL_MS = 4000
const PONG_TIMEOUT_MS = 12000

export class DataBroker {
  private transports: ITransport[] = []
  private listeners = new Map<number, Set<(data: unknown) => void>>()

  readonly latencyMs = writable<number | null>(null)
  private lastPingAt = 0
  private lastPongAt = 0
  private pingTimer: ReturnType<typeof setInterval> | undefined

  private correlationId = 0
  private pending = new Map<
    number,
    {
      resolve: (r: CorrelationResponse) => void
      reject: (e: Error) => void
      timer: ReturnType<typeof setTimeout>
    }
  >()
  // Held while no transport is connected, sent once one connects, so requests aren't lost.
  private deferred: Array<{ id: number; send: () => void; reject: (e: Error) => void }> = []
  private static readonly MAX_DEFERRED = 16

  request(
    payload: Omit<CorrelationRequest, 'correlationId'>,
    opts?: { signal?: AbortSignal }
  ): Promise<CorrelationResponse> {
    return new Promise((resolve, reject) => {
      if (opts?.signal?.aborted) return reject(new DOMException('Aborted', 'AbortError'))
      const id = ++this.correlationId

      const cancel = () => {
        this.deferred = this.deferred.filter(d => d.id !== id)
        const p = this.pending.get(id)
        if (p) {
          clearTimeout(p.timer)
          this.pending.delete(id)
        }
        reject(new DOMException('Aborted', 'AbortError'))
      }
      opts?.signal?.addEventListener('abort', cancel, { once: true })

      // Timeout starts only once the request is on the wire, so a deferred request doesn't
      // expire before it's sent.
      const send = () => {
        const timer = setTimeout(() => {
          this.pending.delete(id)
          reject(new Error(`request ${id} timed out`))
        }, 15000)
        this.pending.set(id, { resolve, reject, timer })
        this.emit(CorrelationRequest, { correlationId: id, ...payload })
      }
      if (this.anyConnected()) {
        send()
        return
      }
      this.deferred.push({ id, send, reject })
      if (this.deferred.length > DataBroker.MAX_DEFERRED) {
        this.deferred.shift()!.reject(new Error('request dropped: not connected'))
      }
    })
  }

  private flushDeferred() {
    const queued = this.deferred
    this.deferred = []
    queued.forEach(({ send }) => send())
  }

  addTransport(transport: ITransport) {
    this.transports.push(transport)
    transport.onData(bytes => this.handleIncoming(bytes))
    transport.onConnect(() => {
      this.resubscribeAll()
      this.startPinging()
      this.flushDeferred()
    })
    transport.onDisconnect(() => {
      if (!this.anyConnected()) this.stopPinging()
    })
  }

  on<T>(fns: MessageFns<T>, callback: (data: T) => void): () => void {
    const tag = tagOf(fns)
    let set = this.listeners.get(tag)
    if (!set) {
      set = new Set()
      this.listeners.set(tag, set)
      this.sendSubscribe(tag)
    }
    set.add(callback as (data: unknown) => void)
    return () => this.off(tag, callback as (data: unknown) => void)
  }

  private off(tag: number, callback: (data: unknown) => void) {
    const set = this.listeners.get(tag)
    if (!set) return
    set.delete(callback)
    if (set.size === 0) {
      this.listeners.delete(tag)
      this.sendUnsubscribe(tag)
    }
  }

  emit<T>(fns: MessageFns<T>, data: T) {
    const message = Message.create({ [keyOf(fns)]: data })
    this.broadcast(encodeMessage(message))
  }

  private handleIncoming(bytes: Uint8Array) {
    const decoded = decodeMessage(bytes)
    if (!decoded) return
    if (decoded.key === 'pongmsg') {
      this.lastPongAt = performance.now()
      this.latencyMs.set(Math.max(0, Math.round(this.lastPongAt - this.lastPingAt)))
      return
    }
    if (decoded.key === 'correlationResponse') {
      const res = decoded.value as CorrelationResponse
      const p = this.pending.get(res.correlationId)
      if (p) {
        clearTimeout(p.timer)
        this.pending.delete(res.correlationId)
        p.resolve(res)
      }
      return
    }
    this.listeners.get(decoded.tag)?.forEach(listener => listener(decoded.value))
  }

  private sendSubscribe(tag: number) {
    this.broadcast(
      encodeMessage(Message.create({ subNotif: SubscribeNotification.create({ tag }) }))
    )
  }

  private sendUnsubscribe(tag: number) {
    this.broadcast(
      encodeMessage(Message.create({ unsubNotif: UnsubscribeNotification.create({ tag }) }))
    )
  }

  private resubscribeAll() {
    for (const tag of this.listeners.keys()) this.sendSubscribe(tag)
  }

  private broadcast(bytes: Uint8Array) {
    this.transports.forEach(t => t.send(bytes))
  }

  private anyConnected() {
    let connected = false
    this.transports.forEach(t => {
      const unsub = t.connected.subscribe(v => (connected = connected || v))
      unsub()
    })
    return connected
  }

  private startPinging() {
    if (this.pingTimer) return
    this.lastPongAt = performance.now()
    this.pingTimer = setInterval(() => {
      if (performance.now() - this.lastPongAt > PONG_TIMEOUT_MS) this.latencyMs.set(null)
      this.ping()
    }, PING_INTERVAL_MS)
    this.ping()
  }

  private stopPinging() {
    if (this.pingTimer) clearInterval(this.pingTimer)
    this.pingTimer = undefined
    this.latencyMs.set(null)
  }

  private ping() {
    this.lastPingAt = performance.now()
    this.broadcast(encodeMessage(Message.create({ pingmsg: PingMsg.create({}) })))
  }
}

export const dataBroker = new DataBroker()
