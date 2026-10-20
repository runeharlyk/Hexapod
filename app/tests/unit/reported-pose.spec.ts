// The pose the 3D view draws from the angles the robot reports (src/lib/motion.ts reportedPose): in IDLE the robot
// reports the angles its servos hold, on entering IDLE, after each joint written and after centring, so the model
// shows exactly those and a drag starts from them.
import { describe, expect, it } from 'vitest'
import { reportedPose } from '$lib/motion'

const held = Array.from({ length: 18 }, (_, servo) => [12, 40, -110][servo % 3] + servo)

describe('reportedPose', () => {
  it('is the angles the robot reports, servo by servo', () =>
    expect(reportedPose(held)).toEqual(held))

  it('is no pose for a robot that reports none yet, an empty snapshot', () =>
    expect(reportedPose([])).toBeNull())

  it("keeps the dragged servo at the drag's angle, which the robot's echo trails", () => {
    const pose = reportedPose(held, { servo: 4, degrees: 33.5 })
    expect(pose?.[4]).toBe(33.5)
    expect(pose?.filter((_, servo) => servo !== 4)).toEqual(held.filter((_, servo) => servo !== 4))
  })
})
