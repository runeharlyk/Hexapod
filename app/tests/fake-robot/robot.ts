import {
  AnglesData,
  AnimationStatus,
  CorrelationRequest,
  CorrelationResponse,
  GaitData,
  GoalStatus,
  Message,
  ModeData,
  OtaStatusData,
  SubscribeNotification,
  type MessageFns
} from '../../src/lib/platform_shared/message'
import {
  APSettings,
  APStatus,
  MDNSSettings,
  PeripheralSettings,
  ServoSettings,
  WifiSettings,
  WifiStatus
} from '../../src/lib/platform_shared/api'
import { encodeMessage, keyOf, tagOf } from '../../src/lib/transport/message-codec'
import type { StateTopic, ValueOf } from '../../src/lib/transport/topics'
import topicsJson from '../../../platform_shared/topics.json' with { type: 'json' }

/** A connection to the robot: what the robot sends it goes to `deliver`. */
export interface Client {
  deliver(bytes: Uint8Array): void
}

export type RequestKey = Exclude<keyof CorrelationRequest, 'correlationId'>
export type Answer = Partial<Omit<CorrelationResponse, 'correlationId'>>
export type Responder = (request: CorrelationRequest) => Answer | Promise<Answer>

const SHARED: { topics: { name: string; tag: number; kind: string }[] } = topicsJson

const camel = (name: string) => name.replace(/_(\w)/g, (_, c: string) => c.toUpperCase())

// Each state topic's value at boot, keyed by its field in the Message oneof: the idle values the firmware reports.
const BOOT_VALUES: Record<string, unknown> = {
  mode: ModeData.create(),
  angles: AnglesData.create(),
  gait: GaitData.create(),
  wifiStatus: WifiStatus.create(),
  apStatus: APStatus.create(),
  otaStatus: OtaStatusData.create(),
  animationStatus: AnimationStatus.create(),
  goalStatus: GoalStatus.create(),
  servoSettings: ServoSettings.create(),
  peripheralSettings: PeripheralSettings.create(),
  wifiSettings: WifiSettings.create(),
  apSettings: APSettings.create(),
  mdnsSettings: MDNSSettings.create()
}

const requestKeyOf = (request: CorrelationRequest) =>
  Object.entries(request).find(
    ([key, value]) => key !== 'correlationId' && value !== undefined
  )?.[0] as RequestKey | undefined

/**
 * The robot as a client reaches it, on the ts-proto Message, whatever carries the bytes (transport.ts for Vitest).
 * It answers as the firmware does (firmware/test/test_topic_snapshot): a subscribe to a state topic with the topic's
 * current value and then the subscribe echoed as the ack, a subscribe to a stream with the ack alone, and a repeated
 * subscribe again without doubling the pushes. Every state topic of platform_shared/topics.json holds a value from the
 * start, or construction fails, as the firmware's table does without a snapshot. A request goes to its responder, else
 * to the fallback, and with neither is answered 501, as by a firmware that does not know it. Pings get pongs.
 */
export class FakeRobot {
  private readonly values = new Map<number, { key: string; value: unknown }>()
  private readonly clients = new Map<Client, Set<number>>()
  private readonly responders = new Map<RequestKey, Responder>()
  private fallback: Responder = () => ({ statusCode: 501 })
  private readonly dropping = new Map<number, number>()
  private acking = true
  private stalled = false
  /** Every frame the robot received, decoded, in order. */
  readonly received: Message[] = []

  constructor(bootValues: Record<string, unknown> = BOOT_VALUES) {
    for (const { name, tag, kind } of SHARED.topics) {
      if (kind !== 'state') continue
      const key = camel(name)
      if (!(key in bootValues))
        throw new Error(`the fake robot has no value for state topic ${name}`)
      this.values.set(tag, { key, value: bootValues[key] })
    }
  }

  /** A client connects; the returned function disconnects it, and its subscriptions end with it. */
  connect(client: Client): () => void {
    this.clients.set(client, new Set())
    return () => this.clients.delete(client)
  }

  receive(client: Client, bytes: Uint8Array) {
    const message = Message.decode(bytes)
    this.received.push(message)
    if (this.stalled) return
    const subscribed = this.clients.get(client)
    if (!subscribed) return
    if (message.subNotif) {
      const tag = message.subNotif.tag
      subscribed.add(tag)
      const state = this.values.get(tag)
      if (!this.acking) return
      if (state) this.send(client, tag, Message.create({ [state.key]: state.value }))
      this.send(client, null, Message.create({ subNotif: SubscribeNotification.create({ tag }) }))
    } else if (message.unsubNotif) {
      subscribed.delete(message.unsubNotif.tag)
    } else if (message.pingmsg) {
      this.send(client, null, Message.create({ pongmsg: {} }))
    } else if (message.correlationRequest) {
      void this.answer(client, message.correlationRequest)
    }
  }

  /** A state topic changes: stored, and pushed to its subscribers. */
  set<F extends StateTopic>(topic: F, value: ValueOf<F>) {
    const tag = tagOf(topic as MessageFns<unknown>)
    const key = keyOf(topic as MessageFns<unknown>)
    this.values.set(tag, { key, value })
    this.pushToSubscribers(tag, Message.create({ [key]: value }))
  }

  /** A stream frame, to its subscribers. */
  push<T>(stream: MessageFns<T>, value: T) {
    const fns = stream as MessageFns<unknown>
    this.pushToSubscribers(tagOf(fns), Message.create({ [keyOf(fns)]: value }))
  }

  respond(key: RequestKey, responder: Responder) {
    this.responders.set(key, responder)
  }

  /** Answers every request that has no responder of its own. */
  respondOtherwise(responder: Responder) {
    this.fallback = responder
  }

  refuse(key: RequestKey, statusCode: number, errorMessage = '') {
    this.respond(key, () => ({ statusCode, errorMessage }))
  }

  /** The next `frames` frames of a tag, snapshots or pushes, are lost on their way. */
  dropNext(tag: number, frames = 1) {
    this.dropping.set(tag, (this.dropping.get(tag) ?? 0) + frames)
  }

  /** Behaves as firmware from before the snapshots: a subscribe is recorded and answered with nothing. */
  ackless() {
    this.acking = false
  }

  /** Stops answering anything. */
  stall() {
    this.stalled = true
  }

  /** The tags subscribed to, in the order the subscribes arrived, repeats included. */
  subscriptions(): number[] {
    return this.received.flatMap(m => (m.subNotif ? [m.subNotif.tag] : []))
  }

  requests(key: RequestKey): CorrelationRequest[] {
    return this.received.flatMap(m =>
      m.correlationRequest && requestKeyOf(m.correlationRequest) === key ?
        [m.correlationRequest]
      : []
    )
  }

  private async answer(client: Client, request: CorrelationRequest) {
    const key = requestKeyOf(request)
    const answer = await ((key && this.responders.get(key)) || this.fallback)(request)
    const response = CorrelationResponse.create({
      correlationId: request.correlationId,
      statusCode: 200,
      ...answer
    })
    if (this.clients.has(client))
      this.send(client, null, Message.create({ correlationResponse: response }))
  }

  private pushToSubscribers(tag: number, message: Message) {
    for (const [client, tags] of this.clients) if (tags.has(tag)) this.send(client, tag, message)
  }

  private send(client: Client, tag: number | null, message: Message) {
    const dropping = tag === null ? 0 : (this.dropping.get(tag) ?? 0)
    if (dropping > 0) return void this.dropping.set(tag!, dropping - 1)
    client.deliver(encodeMessage(message))
  }
}
