import { writable, type Readable, type Writable } from 'svelte/store'
import type { AnimationStatus } from '$lib/platform_shared/message'
import type { body_state_t } from '$lib/kinematic'

const status = writable<AnimationStatus | null>(null)

export const animationStatus: Readable<AnimationStatus | null> = { subscribe: status.subscribe }
export const setAnimationStatus = (s: AnimationStatus | null) => status.set(s)

export const animationPreview: Writable<{ angles: number[]; body: body_state_t } | null> =
  writable(null)
