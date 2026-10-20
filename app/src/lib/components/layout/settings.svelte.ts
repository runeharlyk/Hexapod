import { notifications } from '$lib/components/toasts/notifications'

export interface SettingsSource<T> {
  /** What is read, as the failure names it: "Could not read {what}". */
  what: string
  read: () => Promise<T>
  /** Stores the value; resolves with what the robot kept, when it says, which becomes the stored value. */
  write: (value: T) => Promise<T | undefined>
  /** The toast after a save. */
  saved: string
}

const TOAST_MS = 5000

const messageOf = (error: unknown) => (error instanceof Error ? error.message : String(error))

/**
 * A group of settings the robot stores: read once, edited as a draft, written back on save. The draft is dirty while it
 * differs from what the robot last reported; reset returns it there. A failed read is kept, to be shown with a retry; a
 * failed write leaves the draft as it is and says why in a toast.
 */
export class Settings<T> {
  status = $state<'loading' | 'ready' | 'failed'>('loading')
  error = $state('')
  saving = $state(false)
  value = $state<T>() as T
  #stored = $state.raw('')

  dirty = $derived(
    this.status === 'ready' && JSON.stringify($state.snapshot(this.value)) !== this.#stored
  )

  constructor(private readonly source: SettingsSource<T>) {}

  get what() {
    return this.source.what
  }

  async load() {
    this.status = 'loading'
    try {
      this.#adopt(await this.source.read())
      this.status = 'ready'
    } catch (error) {
      this.error = `Could not read ${this.source.what}: ${messageOf(error)}`
      this.status = 'failed'
    }
  }

  reset() {
    this.value = JSON.parse(this.#stored)
  }

  async save() {
    this.saving = true
    try {
      const kept = await this.source.write($state.snapshot(this.value) as T)
      this.#adopt(kept ?? ($state.snapshot(this.value) as T))
      notifications.success(this.source.saved, TOAST_MS)
    } catch (error) {
      notifications.error(`Not saved: ${messageOf(error)}`, TOAST_MS)
    } finally {
      this.saving = false
    }
  }

  #adopt(value: T) {
    this.#stored = JSON.stringify(value)
    this.value = JSON.parse(this.#stored)
  }
}
