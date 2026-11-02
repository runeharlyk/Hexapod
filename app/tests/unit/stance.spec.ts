// The stance as the app sends and shows it (src/lib/stance.ts), and what the selectors offer in each stance: the robot
// refuses AUTO, TUNED and WALK_NN in the crab (firmware test_stance), so the app does not offer them there.
import { describe, expect, it } from 'vitest'
import { StanceEnum } from '$lib/platform_shared/message'
import { GaitType } from '$lib/gait'
import { MotionModes } from '$lib/motion'
import { Stance, gaitsOffered, modesOffered, stanceFromWire, stanceToWire } from '$lib/stance'

describe('the stance', () => {
  it("crosses the wire as the proto's StanceEnum value", () => {
    expect(stanceToWire(Stance.DEFAULT)).toBe(StanceEnum.STANCE_DEFAULT)
    expect(stanceToWire(Stance.CRAB)).toBe(StanceEnum.STANCE_CRAB)
    expect(stanceFromWire(StanceEnum.STANCE_DEFAULT)).toBe(Stance.DEFAULT)
    expect(stanceFromWire(StanceEnum.STANCE_CRAB)).toBe(Stance.CRAB)
  })

  it('reads a value this app knows no stance for as none', () =>
    expect(stanceFromWire(2)).toBeUndefined())

  it('offers every gait on the default stance', () =>
    expect(gaitsOffered(Stance.DEFAULT)).toEqual(Object.values(GaitType)))

  it('offers the classic gaits alone in the crab, which the robot walks it with', () =>
    expect(gaitsOffered(Stance.CRAB)).toEqual([
      GaitType.TRI_GATE,
      GaitType.BI_GATE,
      GaitType.WAVE,
      GaitType.RIPPLE
    ]))

  it('offers every mode on the default stance, and every mode but WALK_NN in the crab', () => {
    expect(modesOffered(Stance.DEFAULT)).toEqual(Object.values(MotionModes))
    expect(modesOffered(Stance.CRAB)).toEqual(
      Object.values(MotionModes).filter(mode => mode !== MotionModes.WALK_NN)
    )
  })
})
