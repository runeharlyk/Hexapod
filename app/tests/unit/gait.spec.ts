import { describe, expect, it } from 'vitest'
import { GaitType, strideApplies } from '../../src/lib/gait'

describe('stride slider', () => {
  it('is off for TUNED alone, whose stroke the slider does not pick', () => {
    const off = Object.values(GaitType).filter(gait => !strideApplies(gait))
    expect(off).toEqual([GaitType.TUNED])
  })
})
