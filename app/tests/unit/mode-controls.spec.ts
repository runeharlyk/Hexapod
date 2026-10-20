// What the controller shows in each mode (src/lib/mode-controls.ts, rendered by routes/controller/Controls.svelte): the
// mode buttons always, and beside them only the controls the mode reads.
import { describe, expect, it } from 'vitest'
import { MotionModes } from '$lib/motion'
import { controlsFor, type ModeControls } from '$lib/mode-controls'

const none: ModeControls = {
  posture: false,
  sticks: false,
  gait: false,
  walk: false,
  joints: false,
  animation: false
}

const expected: Record<MotionModes, ModeControls> = {
  [MotionModes.DEACTIVATED]: none,
  [MotionModes.IDLE]: { ...none, joints: true },
  [MotionModes.STAND]: { ...none, posture: true, sticks: true },
  [MotionModes.WALK]: { ...none, posture: true, sticks: true, gait: true, walk: true },
  [MotionModes.WALK_NN]: { ...none, posture: true, sticks: true, walk: true },
  [MotionModes.ANIMATE]: { ...none, animation: true }
}

describe('controlsFor', () => {
  for (const mode of Object.values(MotionModes))
    it(`shows only ${mode}'s own controls`, () => expect(controlsFor(mode)).toEqual(expected[mode]))

  it('gives the sticks and the posture sliders only to the modes that stand on the feet', () => {
    const standing = Object.values(MotionModes).filter(mode => controlsFor(mode).sticks)
    expect(standing).toEqual([MotionModes.STAND, MotionModes.WALK, MotionModes.WALK_NN])
    expect(Object.values(MotionModes).filter(mode => controlsFor(mode).posture)).toEqual(standing)
  })
})
