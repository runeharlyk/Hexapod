import { derived, readable } from 'svelte/store'
import type { WatchedSettingsSource } from '$lib/components/layout/settings.svelte'
import type { Together } from '$lib/components/layout/merge'
import { RequestFailed, type RequestPayload } from '$lib/transport/reply'
import type { SettingsTopic, ValueOf, Watched } from '$lib/transport/topics'
import { featureFlagsState, useFeatureFlags } from '$lib/stores/featureFlags'
import { watch } from './watch'
import { writeSettings } from './settings-write'

const WAITING: Watched<never> = {
  value: undefined,
  status: 'subscribing',
  stale: false,
  failure: null,
  pending: false
}

const TOO_OLD: Watched<never> = { ...WAITING, status: 'error', failure: { kind: 'too-old' } }

interface TopicSourceOptions<F extends SettingsTopic> {
  /** What the group is, as a failure names it: "Could not read {what}", "the robot refused {what}". */
  what: string
  /** The toast after a save. */
  saved: string
  /** The request that stores a draft. */
  update: (value: ValueOf<F>) => RequestPayload
  /** The groups of keys that are one field (merge.ts). */
  together: Together
}

/**
 * Whether the robot's settings topics can be followed: `waiting` until its feature flags are in, `too-old` when they
 * say it serves none, else `follow`. A robot whose flags could not be read is followed all the same.
 */
const settingsTopicsGate = () =>
  derived([featureFlagsState, useFeatureFlags()], ([state, { settings_topics }]) =>
    state.kind === 'pending' ? 'waiting'
    : state.kind === 'known' && !settings_topics ? 'too-old'
    : 'follow'
  )

/**
 * Resolves once the settings topics can be followed, and rejects with the too-old failure when the robot serves none:
 * for a reader that asks for a topic's snapshot rather than following it.
 */
export const settingsTopicsServed = (): Promise<void> =>
  new Promise((resolve, reject) => {
    let settled = false
    const stop = settingsTopicsGate().subscribe(reading => {
      if (reading === 'waiting' || settled) return
      settled = true
      if (reading === 'follow') resolve()
      else reject(new RequestFailed({ kind: 'too-old' }))
      queueMicrotask(() => stop())
    })
  })

/**
 * A settings group as a form follows it: the topic's value and pushes, written by request. The topic is followed once
 * the robot's feature flags are in and say it serves settings topics; firmware without them is reported as too old and
 * is not subscribed to, since it would ack the subscribe without a value. A robot whose flags could not be read is
 * followed all the same. Flags read again, as on a fail-over or a followed rename, keep a followed topic followed while
 * they are out, so the form keeps its draft.
 */
export const topicSource = <F extends SettingsTopic>(
  topic: F,
  { what, saved, update, together }: TopicSourceOptions<F>
): WatchedSettingsSource<ValueOf<F>> => {
  const followed = watch(topic)
  const gate = settingsTopicsGate()
  const watched = readable<Watched<ValueOf<F>>>(WAITING, set => {
    let unfollow: (() => void) | null = null
    const stopGate = gate.subscribe(reading => {
      if (reading === 'follow') return void (unfollow ??= followed.subscribe(set))
      if (reading === 'waiting' && unfollow) return
      unfollow?.()
      unfollow = null
      set(reading === 'waiting' ? WAITING : TOO_OLD)
    })
    return () => {
      stopGate()
      unfollow?.()
    }
  })
  return {
    what,
    saved,
    watched,
    write: value => writeSettings(topic, update(value), what),
    together
  }
}
