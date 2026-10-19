import {
  AnalyticsData,
  AnglesData,
  AnimationStatus,
  GaitData,
  GoalStatus,
  IMUData,
  ModeData,
  OtaStatusData,
  TelemetryBatch,
  type MessageFns
} from '$lib/platform_shared/message'
import { APStatus, WifiStatus } from '$lib/platform_shared/api'
import { tagOf } from './message-codec'

/**
 * The robot's server-to-client topics, as platform_shared/topics.json lists them (tests/unit/topics.spec.ts checks this
 * table against it). A state topic holds one current value and every change of it is pushed; a stream is a sequence of
 * frames.
 */
export const STATE_TOPICS = [
  ModeData,
  GaitData,
  WifiStatus,
  APStatus,
  OtaStatusData,
  AnimationStatus,
  GoalStatus
] as const

export const STREAM_TOPICS = [IMUData, AnalyticsData, AnglesData, TelemetryBatch] as const

/** A state topic's message type: `watch(IMUData)` does not type-check. */
export type StateTopic = (typeof STATE_TOPICS)[number]

/** The value a state topic carries. */
export type ValueOf<F> = F extends MessageFns<infer T> ? T : never

const tagsOf = (topics: readonly unknown[]): ReadonlySet<number> =>
  new Set(topics.map(fns => tagOf(fns as MessageFns<unknown>)))
const STATE_TAGS = tagsOf(STATE_TOPICS)
const STREAM_TAGS = tagsOf(STREAM_TOPICS)

/** A tag's kind, or undefined for a tag that is no topic. */
export const kindOf = (tag: number): 'state' | 'stream' | undefined =>
  STATE_TAGS.has(tag) ? 'state'
  : STREAM_TAGS.has(tag) ? 'stream'
  : undefined
