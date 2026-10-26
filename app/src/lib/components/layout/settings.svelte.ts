import type { Readable } from 'svelte/store'
import { notifications } from '$lib/components/toasts/notifications'
import { robotData } from '$lib/robot/data'
import { describeFailure } from '$lib/transport/reply'
import type { Watched } from '$lib/transport/topics'
import { clone, copyField, fieldOf, fieldValue, mergePush, sameField, type Together } from './merge'

export interface SettingsSource<T> {
  /** What is read, as the failure names it: "Could not read {what}". */
  what: string
  read: () => Promise<T>
  /** Stores the value; resolves with what the robot kept, when it says, which becomes the stored value. */
  write: (value: T) => Promise<T | undefined>
  /** The toast after a save. */
  saved: string
}

export interface WatchedSettingsSource<T> {
  what: string
  /** The toast after a save. */
  saved: string
  /** The topic the robot pushes the group on (robot/watch.ts); its every value is the baseline. */
  watched: Readable<Watched<T>>
  /** Sends the value; the reply settles the write, the value shown comes only from `watched`. */
  write: (value: T) => Promise<unknown>
  together: Together
}

const TOAST_MS = 5000

const messageOf = (error: unknown) => (error instanceof Error ? error.message : String(error))

/**
 * A group of settings the robot stores, edited as a draft and written back on save. The draft is dirty while it differs
 * from the baseline; reset returns it there. A read-once source is read by `load` and adopts the reply to a save as its
 * baseline. A watched source is followed by `start`: every value of the topic is the baseline, merged into the draft
 * field by field (merge.ts), so that a field in focus or changed by the user keeps its value and a changed one the robot
 * also changed carries a note; a value lost after the first, as when the robot is asked again, leaves the draft as it
 * is, and the next one is merged into it. Only the user's choice of another robot (robot/data.ts) forgets the draft:
 * that robot's first value is adopted. A failed read is kept, to be shown with a retry; a failed write leaves the
 * draft as it is and says why in a toast.
 */
export class Settings<T extends object> {
  status = $state<'loading' | 'ready' | 'failed'>('loading')
  error = $state('')
  /** A write is in flight: any to the watched topic (Watched.pending), or the save of a read-once source. */
  pending = $state(false)
  value = $state<T>() as T
  /** The robot's value of each field the user changed that the robot changed as well, by field (merge.ts). */
  notes = $state.raw<Record<string, unknown>>({})
  /** What the draft is edited against, as JSON: the last value of the topic, but for the field in focus. */
  #stored = $state.raw('')
  #latest = ''
  #focused: string | null = null

  dirty = $derived(
    this.#stored !== '' && JSON.stringify($state.snapshot(this.value)) !== this.#stored
  )

  constructor(private readonly source: SettingsSource<T> | WatchedSettingsSource<T>) {}

  get what() {
    return this.source.what
  }

  /** Whether the settings follow a topic (`start`) rather than being read once (`load`). */
  get follows() {
    return 'watched' in this.source
  }

  /** Follows the watched topic until the returned function is called. */
  start(): () => void {
    if (!('watched' in this.source)) throw new Error('Only a watched source can be followed')
    const { watched, what } = this.source
    const unregister = robotData.register(() => this.#forget())
    const unfollow = watched.subscribe(state => this.#follow(state, what))
    return () => {
      unfollow()
      unregister()
    }
  }

  async load() {
    if (!('read' in this.source)) throw new Error('Only a read-once source can be read')
    this.status = 'loading'
    try {
      this.#adopt(await this.source.read())
      this.status = 'ready'
    } catch (error) {
      this.error = `Could not read ${this.source.what}: ${messageOf(error)}`
      this.status = 'failed'
    }
  }

  focus(path: string) {
    this.#focused = path
  }

  /** A field that kept its text while in focus takes the latest value, or notes what the robot changed under an edit. */
  blur(path: string) {
    if (this.#focused === path) this.#focused = null
    const together = this.#together
    if (this.status !== 'ready') return
    const field = fieldOf(path, together)
    const draft = $state.snapshot(this.value) as T
    const base = JSON.parse(this.#stored) as T
    const latest = JSON.parse(this.#latest) as T
    if (sameField(latest, base, field, together)) return
    this.#stored = JSON.stringify(this.#withField(base, latest, field))
    if (sameField(draft, base, field, together)) {
      copyField(draft, latest, field, together)
      this.#show(draft)
    } else if (sameField(draft, latest, field, together)) {
      this.#dropNote(field)
    } else {
      this.notes = { ...this.notes, [field]: clone(fieldValue(latest, field)) }
    }
  }

  reset() {
    this.#stored = this.#latest
    this.value = JSON.parse(this.#stored)
    this.notes = {}
  }

  async save() {
    // A save leaves the field in focus without a blur (Enter, or the fieldset it disables), so it settles it first.
    if (this.#focused !== null) this.blur(this.#focused)
    const sent = $state.snapshot(this.value) as T
    if (!this.follows) this.pending = true
    try {
      if ('watched' in this.source) {
        await this.source.write(sent)
        // Answered: every field was sent, so each is clean against what the robot holds, as it normalised it.
        this.reset()
      } else this.#adopt((await this.source.write(sent)) ?? sent)
      notifications.success(this.source.saved, TOAST_MS)
    } catch (error) {
      notifications.error(`Not saved: ${messageOf(error)}`, TOAST_MS)
    } finally {
      if (!this.follows) this.pending = false
    }
  }

  get #together(): Together {
    return 'watched' in this.source ? this.source.together : []
  }

  #dropNote(field: string) {
    this.notes = Object.fromEntries(Object.entries(this.notes).filter(([key]) => key !== field))
  }

  #follow(state: Watched<T>, what: string) {
    this.pending = state.pending
    const followed = this.#latest !== ''
    if (state.value === undefined) {
      this.error =
        state.failure ? `Could not read ${what}: ${describeFailure(state.failure, what)}` : ''
      this.status =
        state.failure ? 'failed'
        : followed ? 'ready'
        : 'loading'
      return
    }
    const next = clone(state.value)
    if (!followed) this.#adopt(next)
    else if (JSON.stringify(next) !== this.#latest) this.#push(next)
    this.status = 'ready'
  }

  #push(next: T) {
    const together = this.#together
    const draft = $state.snapshot(this.value) as T
    const base = JSON.parse(this.#stored) as T
    const merged = mergePush(draft, base, next, this.#focused, together)
    const held = this.#focused === null ? null : fieldOf(this.#focused, together)
    const keptBase = held !== null && sameField(draft, base, held, together)
    this.#latest = JSON.stringify(next)
    this.#stored = JSON.stringify(keptBase ? this.#withField(next, base, held) : next)
    this.#show(merged.draft)
    this.notes = Object.fromEntries(
      Object.entries({ ...this.notes, ...merged.notes }).filter(
        ([field]) => !sameField(merged.draft, JSON.parse(this.#stored), field, together)
      )
    )
  }

  /** A copy of `value` with one field taken from `from`. */
  #withField(value: T, from: T, field: string) {
    const copy = clone(value)
    copyField(copy, from, field, this.#together)
    return copy
  }

  /** Shows a merged draft by replacing only the top-level keys that changed, so the rest of the form stays as it is. */
  #show(draft: T) {
    const shown = $state.snapshot(this.value) as T
    for (const key of Object.keys(draft) as (keyof T)[]) {
      if (JSON.stringify(draft[key]) !== JSON.stringify(shown[key])) this.value[key] = draft[key]
    }
  }

  /** Another robot chosen: nothing of this one is kept, and the next value is adopted. */
  #forget() {
    this.#stored = this.#latest = ''
    this.notes = {}
    this.status = 'loading'
  }

  #adopt(value: T) {
    this.#stored = this.#latest = JSON.stringify(value)
    this.value = JSON.parse(this.#stored)
    this.notes = {}
  }
}
