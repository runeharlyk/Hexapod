// What the controller shows in each mode (src/lib/mode-controls.ts, rendered by routes/controller/Controls.svelte): the
// mode buttons always, and beside them only the controls the mode reads.
import { describe, expect, it } from 'vitest'
import { MotionModes } from '$lib/motion'
import { controlsFor, type ModeControls } from '$lib/mode-controls'

const none: ModeControls = {
  height: false,
  feetDistance: false,
  sticks: false,
  gait: false,
  walk: false,
  joints: false,
  animation: false
}

const standing: ModeControls = { ...none, height: true, feetDistance: true, sticks: true }

// WALK_NN's policy rides at its own height, lift, stride and cadence and reads only the sticks and the feet distance
// (firmware test_walk_nn_reads_the_sticks_and_the_feet_distance_alone).
const expected: Record<MotionModes, ModeControls> = {
  [MotionModes.DEACTIVATED]: none,
  [MotionModes.IDLE]: { ...none, joints: true },
  [MotionModes.STAND]: standing,
  [MotionModes.WALK]: { ...standing, gait: true, walk: true },
  [MotionModes.WALK_NN]: { ...none, feetDistance: true, sticks: true },
  [MotionModes.ANIMATE]: { ...none, animation: true }
}

describe('controlsFor', () => {
  for (const mode of Object.values(MotionModes))
    it(`shows only ${mode}'s own controls`, () => expect(controlsFor(mode)).toEqual(expected[mode]))

  it('gives the sticks and the feet distance only to the modes that stand on the feet', () => {
    const standingModes = Object.values(MotionModes).filter(mode => controlsFor(mode).sticks)
    expect(standingModes).toEqual([MotionModes.STAND, MotionModes.WALK, MotionModes.WALK_NN])
    expect(Object.values(MotionModes).filter(mode => controlsFor(mode).feetDistance)).toEqual(
      standingModes
    )
  })

  it('gives WALK_NN neither the height, the gait, the speed, the step height nor the stride', () => {
    const { height, gait, walk } = controlsFor(MotionModes.WALK_NN)
    expect({ height, gait, walk }).toEqual({ height: false, gait: false, walk: false })
  })
})
