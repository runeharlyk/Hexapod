import {
  MessageType,
  type ITransport,
  type MessageTopic
} from '$lib/interfaces/transport.interface'

export type DataBrokerCallback<T> = (type: MessageType, topic: MessageTopic, payload: T) => void

export class DataBroker {
  private transports: Map<ITransport, string> = new Map()
  private subscriptions: Map<MessageTopic, Map<string, DataBrokerCallback<unknown>>> = new Map()
  private subscriptionCounter = 0

  private dispatchTransport(
    transport: ITransport,
    type: MessageType,
    topic?: MessageTopic,
    payload?: unknown
  ) {
    transport.sendEvent(type, topic, payload).catch(error => {
      console.error('Transport send failed', { type, topic, payload, error })
    })
  }

  constructor() {
    this.transports = new Map()
  }

  addTransport(transport: ITransport) {
    const transportId = `transport_${this.subscriptionCounter++}`
    this.transports.set(transport, transportId)

    transport.onConnect(() => {
      this.subscribeTransport(transport)
    })

    transport.onData((type: MessageType, topic: MessageTopic, payload: unknown) => {
      this.emit(topic, payload, transportId)
    })
  }

  private subscribeTransport(transport: ITransport) {
    const activeTopics = Array.from(this.subscriptions.keys())
    activeTopics.forEach(topic => {
      this.dispatchTransport(transport, MessageType.CONNECT, topic)
    })
  }

  on<T>(topic: MessageTopic, callback: (data: T) => void): string {
    const subscriptionId = `sub_${this.subscriptionCounter++}`

    const topicSubscriptions =
      this.subscriptions.get(topic) || new Map<string, DataBrokerCallback<unknown>>()

    topicSubscriptions.set(subscriptionId, (_t, _tp, payload) => {
      callback(payload as T)
    })

    this.subscriptions.set(topic, topicSubscriptions)

    this.transports.forEach((_, transport) => {
      this.dispatchTransport(transport, MessageType.CONNECT, topic)
    })

    return subscriptionId
  }

  off(subscriptionId: string) {
    for (const [topic, subscriptions] of this.subscriptions.entries()) {
      if (subscriptions.delete(subscriptionId)) {
        if (subscriptions.size === 0) {
          this.subscriptions.delete(topic)
          this.transports.forEach((_, transport) => {
            this.dispatchTransport(transport, MessageType.DISCONNECT, topic)
          })
        }
        break
      }
    }
  }

  send<T>(topic: MessageTopic, data: T) {
    this.transports.forEach((_, transport) => {
      this.dispatchTransport(transport, MessageType.EVENT, topic, data)
    })
  }

  emit<T>(topic: MessageTopic, data: T, excludeSubscriptionId?: string) {
    this.transports.forEach((transportId, transport) => {
      if (transportId !== excludeSubscriptionId) {
        this.dispatchTransport(transport, MessageType.EVENT, topic, data)
      }
    })
    const subscriptions = this.subscriptions.get(topic)
    if (!subscriptions) return

    subscriptions.forEach((callback, subscriptionId) => {
      if (subscriptionId !== excludeSubscriptionId) {
        callback(MessageType.EVENT, topic, data)
      }
    })
  }
}

export const dataBroker = new DataBroker()
