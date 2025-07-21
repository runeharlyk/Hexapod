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
      console.log('data', type, topic, payload)
      this.emit(topic, payload, transportId)
    })
  }

  private subscribeTransport(transport: ITransport) {
    const activeTopics = Array.from(this.subscriptions.keys())
    activeTopics.forEach(topic => {
      transport.sendEvent(MessageType.CONNECT, topic)
    })
  }

  on<T>(topic: MessageTopic, callback: (data: T) => void): string {
    const subscriptionId = `sub_${this.subscriptionCounter++}`
    const topicSubscriptions = this.subscriptions.get(topic) || new Map()
    topicSubscriptions.set(subscriptionId, (data: unknown) => callback(data as T))
    this.subscriptions.set(topic, topicSubscriptions)

    this.transports.forEach((_, transport) => {
      transport.sendEvent(MessageType.CONNECT, topic)
    })

    return subscriptionId
  }

  off(subscriptionId: string) {
    for (const [topic, subscriptions] of this.subscriptions.entries()) {
      if (subscriptions.delete(subscriptionId)) {
        if (subscriptions.size === 0) {
          this.subscriptions.delete(topic)
          this.transports.forEach((_, transport) => {
            transport.sendEvent(MessageType.DISCONNECT, topic)
          })
        }
        break
      }
    }
  }

  emit<T>(topic: MessageTopic, data: T, excludeSubscriptionId?: string) {
    this.transports.forEach((transportId, transport) => {
      if (transportId !== excludeSubscriptionId) {
        transport.sendEvent(MessageType.EVENT, topic, data)
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
