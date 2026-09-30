import { writable, type Readable, type Writable } from 'svelte/store'
import type { AnimationStatus } from '$lib/platform_shared/message'
import type { body_state_t } from '$lib/kinematic'

const status = writable<AnimationStatus | null>(null)

export const animationStatus: Readable<AnimationStatus | null> = { subscribe: status.subscribe }
export const setAnimationStatus = (s: AnimationStatus | null) => status.set(s)

// angles in radians, IK leg order; mask is the evaluator's 18-bit clamp mask, bit leg * 3 + joint.
export type AnimationPreview = { angles: number[]; body: body_state_t; mask?: number }

export const animationPreview: Writable<AnimationPreview | null> = writable(null)
