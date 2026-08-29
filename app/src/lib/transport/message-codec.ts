import { Message, protoMetadata, type MessageFns } from '$lib/platform_shared/message'
import { protoMetadata as apiProtoMetadata } from '$lib/platform_shared/api'

const TYPE_TO_TAG = new Map<MessageFns<unknown>, number>()
const TYPE_TO_KEY = new Map<MessageFns<unknown>, string>()
const TAG_TO_KEY = new Map<number, string>()
const KEY_TO_TAG = new Map<string, number>()

{
  // The Message oneof carries imported api.* types as well as socket_message ones, so resolve
  // field types against both files' reference maps.
  const references = {
    ...(apiProtoMetadata.references as Record<string, MessageFns<unknown>>),
    ...(protoMetadata.references as Record<string, MessageFns<unknown>>)
  }
  const messageDesc = protoMetadata.fileDescriptor.messageType?.find(m => m.name === 'Message')
  for (const field of messageDesc?.field ?? []) {
    const fns = field.typeName ? references[field.typeName] : undefined
    if (fns && field.jsonName && field.number) {
      TYPE_TO_TAG.set(fns, field.number)
      TYPE_TO_KEY.set(fns, field.jsonName)
      TAG_TO_KEY.set(field.number, field.jsonName)
      KEY_TO_TAG.set(field.jsonName, field.number)
    }
  }
}

export const tagOf = <T>(fns: MessageFns<T>): number => {
  const tag = TYPE_TO_TAG.get(fns as MessageFns<unknown>)
  if (tag === undefined) throw new Error('Message type is not a field of the Message oneof')
  return tag
}

export const keyOf = <T>(fns: MessageFns<T>): string => {
  const key = TYPE_TO_KEY.get(fns as MessageFns<unknown>)
  if (key === undefined) throw new Error('Message type is not a field of the Message oneof')
  return key
}

export interface Decoded {
  tag: number
  key: string
  value: unknown
}

export const decodeMessage = (bytes: Uint8Array): Decoded | null => {
  let msg: Record<string, unknown>
  try {
    msg = Message.decode(bytes) as Record<string, unknown>
  } catch (error) {
    // A truncated frame or a firmware built against a stale schema must cost one frame,
    // not the whole connection.
    console.error('Dropping undecodable message frame:', error)
    return null
  }
  for (const [key, value] of Object.entries(msg)) {
    if (value === undefined) continue
    const tag = KEY_TO_TAG.get(key)
    if (tag === undefined) return null
    return { tag, key, value }
  }
  return null
}

export const encodeMessage = (msg: Message): Uint8Array => Message.encode(msg).finish()
