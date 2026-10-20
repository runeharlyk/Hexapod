// The modes as they cross the wire (src/lib/motion.ts): each by its ModesEnum value, which the firmware's MOTION_STATE
// carries too (firmware/include/motion_state_wire.h). POSE is gone and its 2 reserved, so the later modes keep their
// numbers and a robot that reports 2 reports no mode this app knows.
import { describe, expect, it } from 'vitest'
import { ModesEnum } from '$lib/platform_shared/message'
import { MotionModes, modeFromWire, modeToWire } from '$lib/motion'

describe('MotionModes on the wire', () => {
  it('sends each mode as its ModesEnum value and reads it back', () => {
    expect(Object.values(MotionModes).map(modeToWire)).toEqual([
      ModesEnum.DEACTIVATED,
      ModesEnum.IDLE,
      ModesEnum.STAND,
      ModesEnum.WALK,
      ModesEnum.WALK_NN,
      ModesEnum.ANIMATE
    ])
    for (const mode of Object.values(MotionModes)) expect(modeFromWire(modeToWire(mode))).toBe(mode)
  })

  it('keeps the numbers the modes after POSE had, so a deployed robot reads them as before', () => {
    expect(
      [MotionModes.STAND, MotionModes.WALK, MotionModes.WALK_NN, MotionModes.ANIMATE].map(
        modeToWire
      )
    ).toEqual([3, 4, 5, 6])
  })

  it('has no POSE, and reads its reserved 2 and any unknown value as no mode', () => {
    expect(Object.keys(MotionModes)).not.toContain('POSE')
    expect(modeFromWire(2)).toBeUndefined()
    expect(modeFromWire(7)).toBeUndefined()
    expect(modeFromWire(ModesEnum.UNRECOGNIZED)).toBeUndefined()
  })
})
