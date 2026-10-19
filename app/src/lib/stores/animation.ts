import { writable, type Writable } from 'svelte/store'
import type { body_state_t } from '$lib/kinematic'

// angles in radians, IK leg order; mask is the evaluator's 18-bit clamp mask, bit leg * 3 + joint.
// handleFeet places the foot handles when they should not follow body.feet.
export type AnimationPreview = {
  angles: number[]
  body: body_state_t
  mask?: number
  handleFeet?: number[][]
}

export const animationPreview: Writable<AnimationPreview | null> = writable(null)
