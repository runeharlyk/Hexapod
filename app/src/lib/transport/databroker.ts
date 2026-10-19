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
import { RequestFailed } from './reply'
import { StateEntry, type StateLink, type Watched } from './topics'
import { SETTING_GRACE_MS, kindOfRequest } from './request-kinds'

const PING_INTERVAL_MS = 4000
const PONG_TIMEOUT_MS = 12000
const REQUEST_TIMEOUT_MS = 15000

export class DataBroker {
  private transports: ITransport[] = []
  // Listeners also get the frame as it arrived, for a recorder that keeps the robot's bytes.
  private listeners = new Map<number, Set<(data: unknown, frame: Uint8Array) => void>>()

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
  // The state topics followed (follow()), and the listener each feeds its values through.
  private states = new Map<number, StateEntry<unknown>>()
  private stateListeners = new Map<number, (data: unknown) => void>()
  // The transport the subscriptions went out on, and what has been heard: every frame, and whether that link acked.
  private carrier: ITransport | undefined
  private framesHeard = 0
  private ackHeard = false
  private readonly stateLink: StateLink = {
    linked: () => this.activeTransport() !== undefined,
    subscribe: tag => this.sendSubscribe(tag),
    framesHeard: () => this.framesHeard,
    ackHeard: () => this.ackHeard
  }

  // Held while no transport is connected, sent once one connects, so requests aren't lost.
  private deferred: Array<{
    id: number
    send: () => void
    sent: () => void
    reject: (e: Error) => void
  }> = []
  private static readonly MAX_DEFERRED = 16

  request(
    payload: Omit<CorrelationRequest, 'correlationId'>,
    opts?: { signal?: AbortSignal }
  ): Promise<CorrelationResponse> {
    return new Promise((resolve, reject) => {
      const signal = opts?.signal
      if (signal?.aborted) return reject(new DOMException('Aborted', 'AbortError'))
      const id = ++this.correlationId

      let grace: ReturnType<typeof setTimeout> | undefined
      const forget = () => {
        this.deferred = this.deferred.filter(d => d.id !== id)
        const p = this.pending.get(id)
        if (p) {
          clearTimeout(p.timer)
          this.pending.delete(id)
        }
        clearTimeout(grace)
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

      // The timeout counts from the call, so a read held for a link fails in the same time as one sent at once.
      const timer = setTimeout(
        () => settle.reject(new RequestFailed({ kind: 'timeout' })),
        REQUEST_TIMEOUT_MS
      )
      this.pending.set(id, { ...settle, timer })
      const send = () => this.emit(CorrelationRequest, { correlationId: id, ...payload })
      if (this.activeTransport()) {
        send()
        return
      }
      // While no link is up, the request's kind decides (transport/request-kinds.ts).
      const kind = kindOfRequest(payload)
      const notSent = () => settle.reject(new RequestFailed({ kind: 'disconnected' }))
      if (kind === 'action') return notSent()
      if (kind === 'setting') grace = setTimeout(notSent, SETTING_GRACE_MS)
      this.deferred.push({ id, send, sent: () => clearTimeout(grace), reject: settle.reject })
      if (this.deferred.length > DataBroker.MAX_DEFERRED) {
        this.deferred[0].reject(new RequestFailed({ kind: 'disconnected' }))
      }
    })
  }

  // A settings write sent here is no longer held: its grace ends with the wait.
  private flushDeferred() {
    const queued = this.deferred
    this.deferred = []
    queued.forEach(({ send, sent }) => {
      sent()
      send()
    })
  }

  // The requests on the wire; one still held for a link waits on.
  private rejectPending() {
    const held = new Set(this.deferred.map(d => d.id))
    for (const [id, { reject }] of [...this.pending.entries()])
      if (!held.has(id)) reject(new RequestFailed({ kind: 'disconnected' }))
  }

  // Registration order is priority order; keep it in step with stores/link.ts so requests and
  // subscriptions travel on the link the UI reports.
  addTransport(transport: ITransport) {
    this.transports.push(transport)
    transport.onData(bytes => this.handleIncoming(bytes))
    transport.onConnect(() => {
      // A link of lower priority than the one carrying the subscriptions changes nothing for them.
      if (this.activeTransport() !== this.carrier) this.moveSubscriptions(false)
      this.startPinging()
      this.flushDeferred()
    })
    transport.onDisconnect(() => {
      if (this.activeTransport()) {
        if (transport === this.carrier) this.moveSubscriptions(true)
        return
      }
      this.carrier = undefined
      this.states.forEach(entry => entry.linkLost())
      this.stopPinging()
      this.rejectPending()
    })
  }

  // The subscriptions go to the link that now carries them, and every state topic waits for its snapshot there; after a
  // fail-over the values held are stale until then.
  private moveSubscriptions(failover: boolean) {
    this.carrier = this.activeTransport()
    this.ackHeard = false
    if (failover) this.states.forEach(entry => entry.linkLost())
    this.resubscribeAll()
    this.states.forEach(entry => entry.subscribed())
  }

  on<T>(fns: MessageFns<T>, callback: (data: T, frame: Uint8Array) => void): () => void {
    const tag = tagOf(fns)
    if (this.addListener(tag, callback as (data: unknown, frame: Uint8Array) => void))
      this.sendSubscribe(tag)
    return () => this.off(tag, callback as (data: unknown, frame: Uint8Array) => void)
  }

  /**
   * Follows a state topic (transport/topics.ts): `watcher` gets its state now and on every change. Every follower of a
   * tag shares one entry and one subscription; the robot is asked for the current value when the first follows, even
   * where a stream listener of the tag already subscribed. Returns the function that stops following.
   */
  follow<T>(fns: MessageFns<T>, watcher: (state: Watched<T>) => void): () => void {
    const tag = tagOf(fns)
    let entry = this.states.get(tag) as StateEntry<T> | undefined
    if (!entry) {
      const created = new StateEntry<T>(tag, this.stateLink)
      const listener = (data: unknown) => created.received(data as T)
      this.states.set(tag, created as StateEntry<unknown>)
      this.stateListeners.set(tag, listener)
      this.addListener(tag, listener)
      created.subscribe()
      entry = created
    }
    const remove = entry.add(watcher)
    return () => {
      if (!remove()) return
      this.states.delete(tag)
      const listener = this.stateListeners.get(tag)
      this.stateListeners.delete(tag)
      if (listener) this.off(tag, listener)
    }
  }

  /** A write the robot answers with a push, such as a mode, is pending until that push comes or PENDING_MS pass. */
  markPending<T>(fns: MessageFns<T>) {
    this.states.get(tagOf(fns))?.markPending()
  }

  /** Forgets every state topic's value and asks the robot again, for a link that may reach another robot. */
  resetStates() {
    this.states.forEach(entry => entry.reset())
  }

  private addListener(tag: number, callback: (data: unknown, frame: Uint8Array) => void) {
    let set = this.listeners.get(tag)
    const first = !set
    if (!set) {
      set = new Set()
      this.listeners.set(tag, set)
    }
    set.add(callback)
    return first
  }

  private off(tag: number, callback: (data: unknown, frame: Uint8Array) => void) {
    const set = this.listeners.get(tag)
    if (!set) return
    set.delete(callback)
    if (set.size === 0) {
      this.listeners.delete(tag)
      this.sendUnsubscribe(tag)
    }
  }

  /** Sends on the active link; false, and nothing is sent or held, while no link is up. */
  emit<T>(fns: MessageFns<T>, data: T): boolean {
    const message = Message.create({ [keyOf(fns)]: data })
    return this.broadcast(encodeMessage(message))
  }

  /**
   * Motion (sticks, heartbeats, animation preview poses): sent only when the link can take it now, and otherwise
   * dropped, never queued or sent after a reconnect, since a frame that waited would move the robot with the past.
   * False when it was dropped.
   */
  emitVolatile<T>(fns: MessageFns<T>, data: T): boolean {
    const transport = this.activeTransport()
    return (
      transport !== undefined &&
      transport.trySend(encodeMessage(Message.create({ [keyOf(fns)]: data })))
    )
  }

  private handleIncoming(bytes: Uint8Array) {
    const decoded = decodeMessage(bytes)
    if (!decoded) return
    this.framesHeard++
    if (decoded.key === 'subNotif') {
      // The robot echoes a subscribe as its ack, after a state topic's current value.
      this.ackHeard = true
      this.states.get((decoded.value as SubscribeNotification).tag)?.acked()
      return
    }
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
        listener(decoded.value, bytes)
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
    const transport = this.activeTransport()
    transport?.send(bytes)
    return transport !== undefined
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
