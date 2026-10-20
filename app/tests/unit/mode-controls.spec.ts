// What the controller shows in each mode (src/lib/mode-controls.ts, rendered by routes/controller/Controls.svelte): the
// mode buttons always, and beside them only the controls the mode reads, as the firmware reads them.
import { describe, expect, it } from 'vitest'
import { MotionModes } from '$lib/motion'
import { controlsFor, type ModeControls } from '$lib/mode-controls'

const MODES = Object.values(MotionModes)

const shown = (mode: MotionModes | null) =>
  (Object.entries(controlsFor(mode)) as [keyof ModeControls, boolean][])
    .filter(([, on]) => on)
    .map(([control]) => control)
    .sort()

const modesShowing = (control: keyof ModeControls) =>
  MODES.filter(mode => controlsFor(mode)[control])

describe('controlsFor', () => {
  it('shows nothing beside the mode buttons in DEACTIVATED, whose servos are asleep', () =>
    expect(shown(MotionModes.DEACTIVATED)).toEqual([]))

  it('drives the joints in IDLE alone, the only mode the firmware takes a joint in', () =>
    expect(modesShowing('joints')).toEqual([MotionModes.IDLE]))

  it('shows the posture where the classic gait stands on it, STAND and WALK', () => {
    expect(modesShowing('height')).toEqual([MotionModes.STAND, MotionModes.WALK])
    expect(modesShowing('feetDistance')).toEqual([MotionModes.STAND, MotionModes.WALK])
  })

  it('shows the gait, speed, step height and stride in WALK alone', () => {
    expect(modesShowing('gait')).toEqual([MotionModes.WALK])
    expect(modesShowing('walk')).toEqual([MotionModes.WALK])
  })

  // The firmware's test_walk_nn_reads_the_sticks_alone: the policy walks the trained stance at its own height.
  it('gives WALK_NN the sticks alone', () => expect(shown(MotionModes.WALK_NN)).toEqual(['sticks']))

  it('gives ANIMATE the animation controls alone, and no sticks the firmware ignores there', () =>
    expect(shown(MotionModes.ANIMATE)).toEqual(['animation']))

  it('shows nothing beside the mode buttons in a mode this app does not know', () =>
    expect(shown(null)).toEqual([]))
})
