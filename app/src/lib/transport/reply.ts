import { protoMetadata, type CorrelationRequest } from '$lib/platform_shared/message'
import { protoMetadata as apiProtoMetadata } from '$lib/platform_shared/api'

/** A request as the data broker sends it (transport/databroker.ts), which numbers it itself. */
export type RequestPayload = Omit<CorrelationRequest, 'correlationId'>

/** What every reply carries, whatever it answers. */
export interface Reply {
  statusCode: number
  errorMessage: string
}

export type Failure =
  | { kind: 'too-old' }
  | { kind: 'refused'; status: number; message: string }
  | { kind: 'timeout' }
  | { kind: 'disconnected' }

export type Outcome<R extends Reply> = { kind: 'ok'; reply: R } | Failure

// The firmware answers a request it does not know with a 501 (the `default:` of firmware/src/main.cpp's correlation
// handler), which it answers to nothing else, so a 501 always means firmware older than the request.
const UNKNOWN_REQUEST = 501

// Firmware up to v0.1.1, the current release, answers a request it does not know with a bare 400 instead, which it also
// answers to a malformed one. A request whose message declares no field has nothing to be malformed, so for these
// requests alone a bare 400 is read as firmware too old; for any other it stays a refusal. The set is read from the
// schema rather than listed by hand, so a request that gains a field leaves it by itself. This fallback serves robots
// that run those releases and can go once none does.
const FIELDLESS_REQUESTS: ReadonlySet<string> = (() => {
  const fieldCounts = new Map<string, number>()
  for (const file of [protoMetadata.fileDescriptor, apiProtoMetadata.fileDescriptor])
    for (const type of file.messageType)
      fieldCounts.set(`.${file.package}.${type.name}`, type.field.length)
  const request = protoMetadata.fileDescriptor.messageType.find(
    type => type.name === 'CorrelationRequest'
  )
  return new Set(
    (request?.field ?? [])
      .filter(field => fieldCounts.get(field.typeName) === 0)
      .map(field => field.jsonName)
  )
})()

const requestKey = (payload: object) =>
  Object.entries(payload).find(([, value]) => value !== undefined)?.[0]

/** Whether a bare 400 to this request, from firmware up to v0.1.1, can only mean that the firmware does not know it. */
export const isFieldless = (payload: object) => FIELDLESS_REQUESTS.has(requestKey(payload) ?? '')

const isSuccess = (status: number) => status >= 200 && status < 300

/**
 * What a reply means. Any 2xx is an answer (202 is a scan or goal still running, which the caller reads on), as is a
 * status the caller names in `accept`, such as the 422 that carries an animation's report.
 */
export const interpret = <R extends Reply>(
  payload: object,
  reply: R,
  accept: readonly number[] = []
): Outcome<R> => {
  if (isSuccess(reply.statusCode) || accept.includes(reply.statusCode)) return { kind: 'ok', reply }
  if (reply.statusCode === UNKNOWN_REQUEST) return { kind: 'too-old' }
  if (reply.statusCode === 400 && !reply.errorMessage && isFieldless(payload))
    return { kind: 'too-old' }
  return { kind: 'refused', status: reply.statusCode, message: reply.errorMessage }
}

/**
 * The sentence a page shows for a failure; `feature` names what the request was for, such as "the task table", or the
 * step that failed, such as a chunk at an offset. A refusal without a reason leaves out the HTTP status, which says
 * nothing to the person reading it.
 */
export const describeFailure = (failure: Failure, feature: string): string => {
  switch (failure.kind) {
    case 'too-old':
      return `the robot's firmware does not support ${feature}; update it`
    case 'refused':
      return failure.message || `the robot refused ${feature}`
    case 'timeout':
      return `the robot did not answer in time for ${feature}`
    case 'disconnected':
      return 'the robot is not connected'
  }
}

/** A request that got no answer to read on; `failure` says why and `message` says it for a page. */
export class RequestFailed extends Error {
  constructor(
    readonly failure: Failure,
    feature = 'the request'
  ) {
    super(describeFailure(failure, feature))
    this.name = 'RequestFailed'
  }
}

/** Anything that sends requests the way the data broker does: the broker itself, or a narrow link a test fakes. */
export interface RequestLink<P, R extends Reply> {
  request(payload: P, opts?: { signal?: AbortSignal }): Promise<R>
}

/**
 * Sends a request and resolves to the reply when it answers; otherwise rejects with a RequestFailed whose message names
 * `feature`. An abort rejects with the AbortError as the broker raised it.
 */
export const ask = async <P extends object, R extends Reply>(
  link: RequestLink<P, R>,
  payload: P,
  feature: string,
  opts: { signal?: AbortSignal; accept?: readonly number[] } = {}
): Promise<R> => {
  let reply: R
  try {
    reply = await link.request(payload, opts.signal ? { signal: opts.signal } : undefined)
  } catch (error) {
    // The broker's own timeout and drop carry no feature; they are named here, where it is known.
    if (error instanceof RequestFailed) throw new RequestFailed(error.failure, feature)
    throw error
  }
  const outcome = interpret(payload, reply, opts.accept)
  if (outcome.kind === 'ok') return outcome.reply
  throw new RequestFailed(outcome, feature)
}
