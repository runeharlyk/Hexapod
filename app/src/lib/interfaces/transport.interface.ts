import type { DataBrokerCallback } from '$lib/transport/databroker'
import { type Writable } from 'svelte/store'

export enum MessageType {
  CONNECT = 0,
  DISCONNECT = 1,
  EVENT = 2,
  PING = 3,
  PONG = 4
}

export enum MessageTopic {
  SERVO = 1,
  COMMAND = 2,
  MODE = 3,
  GAIT = 4,
  ANGLE = 6,
  SERVO_SETTINGS = 9
}

export type Temp = {
  value: number
}

export type Command = {
  command: string
  extra: {
    [key: string]: unknown
  }
}

export type SubscribeMsg = [MessageType.CONNECT, MessageTopic]
export type UnsubscribeMsg = [MessageType.DISCONNECT, MessageTopic]
export type DataMsg = [MessageType.EVENT, MessageTopic, unknown]
export type PingMsg = [MessageType.PING]
export type PongMsg = [MessageType.PONG]

export type ServerMessage = SubscribeMsg | UnsubscribeMsg | DataMsg | PingMsg | PongMsg

export interface ITransport {
  connected: Writable<boolean>
  connect: () => Promise<void>
  disconnect: () => Promise<void>
  send: <T>(data: T) => Promise<void>
  onData: (data: DataBrokerCallback<unknown>) => void
  onConnect: (cb: () => void) => void
  onDisconnect: (cb: () => void) => void
  sendEvent: (type: MessageType, topic?: MessageTopic, payload?: unknown) => Promise<void>
}
