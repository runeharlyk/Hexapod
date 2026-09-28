import { get, writable } from 'svelte/store'
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
const REQUEST_TIMEOUT_MS = 15000

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
      const signal = opts?.signal
      if (signal?.aborted) return reject(new DOMException('Aborted', 'AbortError'))
      const id = ++this.correlationId

      const forget = () => {
        this.deferred = this.deferred.filter(d => d.id !== id)
        const p = this.pending.get(id)
        if (p) {
          clearTimeout(p.timer)
          this.pending.delete(id)
        }
        signal?.removeEventListener('abort', cancel)
      }
      const settle = {
        resolve: (response: CorrelationResponse) => {
          forget()
          resolve(response)
        },
        reject: (error: Error) => {
          forget()
          reject(error)
        }
      }
      const cancel = () => settle.reject(new DOMException('Aborted', 'AbortError'))
      signal?.addEventListener('abort', cancel, { once: true })

      // Timeout starts only once the request is on the wire, so a deferred request doesn't
      // expire before it's sent.
      const send = () => {
        const timer = setTimeout(
          () => settle.reject(new Error(`request ${id} timed out`)),
          REQUEST_TIMEOUT_MS
        )
        this.pending.set(id, { ...settle, timer })
        this.emit(CorrelationRequest, { correlationId: id, ...payload })
      }
      if (this.activeTransport()) {
        send()
        return
      }
      this.deferred.push({ id, send, reject: settle.reject })
      if (this.deferred.length > DataBroker.MAX_DEFERRED) {
        this.deferred[0].reject(new Error('request dropped: not connected'))
      }
    })
  }

  private flushDeferred() {
    const queued = this.deferred
    this.deferred = []
    queued.forEach(({ send }) => send())
  }

  private rejectPending() {
    for (const { reject } of [...this.pending.values()]) reject(new Error('disconnected'))
  }

  // Registration order is priority order; keep it in step with stores/link.ts so requests and
  // subscriptions travel on the link the UI reports.
  addTransport(transport: ITransport) {
    this.transports.push(transport)
    transport.onData(bytes => this.handleIncoming(bytes))
    transport.onConnect(() => {
      this.resubscribeAll()
      this.startPinging()
      this.flushDeferred()
    })
    transport.onDisconnect(() => {
      if (this.activeTransport()) {
        this.resubscribeAll()
        return
      }
      this.stopPinging()
      this.rejectPending()
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
      this.pending.get(res.correlationId)?.resolve(res)
      return
    }
    // One throwing handler must neither starve the others nor unwind into a transport's read loop.
    this.listeners.get(decoded.tag)?.forEach(listener => {
      try {
        listener(decoded.value)
      } catch (error) {
        console.error(`Handler for message ${decoded.key} failed:`, error)
      }
    })
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
    this.activeTransport()?.send(bytes)
  }

  private activeTransport() {
    return this.transports.find(t => get(t.connected))
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
