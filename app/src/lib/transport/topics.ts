import {
  AnglesData,
  AnimationStatus,
  GaitData,
  GoalStatus,
  ModeData,
  OtaStatusData,
  type MessageFns
} from '$lib/platform_shared/message'
import { APStatus, WifiStatus } from '$lib/platform_shared/api'
import type { Failure } from './reply'

/**
 * The robot's state topics, as platform_shared/topics.json lists them (tests/unit/topics.spec.ts checks this table
 * against it). A state topic holds one current value: the robot answers a subscribe with that value and then the
 * subscribe echoed as its ack, and pushes every change. A stream, any other topic, is a sequence of frames followed
 * with the data broker's `on`, and its subscribe is acked alone.
 */
export const STATE_TOPICS = [
  ModeData,
  AnglesData,
  GaitData,
  WifiStatus,
  APStatus,
  OtaStatusData,
  AnimationStatus,
  GoalStatus
] as const

/** A state topic's message type: `watch(IMUData)` does not type-check. */
export type StateTopic = (typeof STATE_TOPICS)[number]

/** The value a state topic carries. */
export type ValueOf<F> = F extends MessageFns<infer T> ? T : never

/** How long a subscribe waits for its ack before it is sent again. */
export const ACK_TIMEOUT_MS = 2000
/** Subscribes without a current value (no ack, or an ack with no value before it) before a watch gives up. */
export const SUBSCRIBE_TRIES = 3

/**
 * A state topic as a page sees it. `value` is the robot's last value, undefined until the first arrives. `status` is
 * `live` once the robot acked the subscribe after sending its value, `subscribing` until then, and `error` when the
 * robot never sent one; `failure` then says why. `stale` marks a value kept from a link that dropped, until the next
 * link's snapshot replaces it. The design's fourth field, `pending` (a mode or gait sent and not yet reported, cleared
 * after 1 s), is left to phase 2, whose `watchSettings.save()` counts its writes into it and is its first reader.
 */
export interface Watched<T> {
  value: T | undefined
  status: 'subscribing' | 'live' | 'error'
  stale: boolean
  failure: Failure | null
}

/** What a state entry needs of the data broker. */
export interface StateLink {
  linked(): boolean
  subscribe(tag: number): void
  /** Frames heard on the links so far, of any kind. */
  framesHeard(): number
  /** Whether the link up now has acked any subscribe. */
  ackHeard(): boolean
}

/**
 * One state topic's cache, shared by every watch of it. A value is applied whenever it arrives; the status changes only
 * on the ack, the ack timeouts and the link. A subscribe's ack with no value before it means the snapshot was lost, and
 * a subscribe without an ack in time means it or its ack was: both send it again, until SUBSCRIBE_TRIES without a value
 * end in `error`. A link that carries frames but never acks is firmware from before the snapshots, named as too old.
 */
export class StateEntry<T> {
  private state: Watched<T> = {
    value: undefined,
    status: 'subscribing',
    stale: false,
    failure: null
  }
  private readonly watchers = new Set<(state: Watched<T>) => void>()
  private awaitingAck = false
  private valueSinceSubscribe = false
  private tries = 0
  private heardAtFirstTry = 0
  private ackTimer: ReturnType<typeof setTimeout> | undefined

  constructor(
    private readonly tag: number,
    private readonly link: StateLink
  ) {}

  /** Adds a watcher and tells it the state at once; the returned function removes it, true when it was the last. */
  add(watcher: (state: Watched<T>) => void): () => boolean {
    this.watchers.add(watcher)
    watcher(this.state)
    return () => {
      this.watchers.delete(watcher)
      if (this.watchers.size > 0) return false
      this.stopTimers()
      return true
    }
  }

  /** Sends a subscribe, when a link is up, and waits for its ack. */
  subscribe() {
    if (!this.link.linked()) return
    this.link.subscribe(this.tag)
    this.subscribed()
  }

  /** A subscribe went out on the broker's own account, on a link up or a fail-over: its ack is awaited. */
  subscribed() {
    if (this.tries === 0) this.heardAtFirstTry = this.link.framesHeard()
    this.awaitingAck = true
    this.valueSinceSubscribe = false
    clearTimeout(this.ackTimer)
    this.ackTimer = setTimeout(() => this.retry('no-ack'), ACK_TIMEOUT_MS)
  }

  received(value: T) {
    this.valueSinceSubscribe = true
    this.update({ value, stale: false })
  }

  acked() {
    if (!this.awaitingAck) return
    this.awaitingAck = false
    clearTimeout(this.ackTimer)
    if (!this.valueSinceSubscribe) return this.retry('no-value')
    this.tries = 0
    this.update({ status: 'live', failure: null })
  }

  /** The last link dropped, or the one carrying the topic gave way to another: the value is kept, as stale. */
  linkLost() {
    this.stopTimers()
    this.awaitingAck = false
    this.tries = 0
    this.update({
      status: 'subscribing',
      stale: this.state.value !== undefined,
      failure: null
    })
  }

  /** Another robot, maybe: nothing known is kept, and the robot is asked again. */
  reset() {
    this.stopTimers()
    this.awaitingAck = false
    this.tries = 0
    this.update({
      value: undefined,
      status: 'subscribing',
      stale: false,
      failure: null
    })
    this.subscribe()
  }

  private retry(reason: 'no-ack' | 'no-value') {
    this.awaitingAck = false
    this.tries++
    if (this.tries < SUBSCRIBE_TRIES) return this.subscribe()
    const carriesFramesButNeverAcks =
      reason === 'no-ack' && !this.link.ackHeard() && this.link.framesHeard() > this.heardAtFirstTry
    this.update({
      status: 'error',
      failure: carriesFramesButNeverAcks ? { kind: 'too-old' } : { kind: 'timeout' }
    })
  }

  private stopTimers() {
    clearTimeout(this.ackTimer)
  }

  private update(change: Partial<Watched<T>>) {
    this.state = { ...this.state, ...change }
    this.watchers.forEach(watcher => watcher(this.state))
  }
}
