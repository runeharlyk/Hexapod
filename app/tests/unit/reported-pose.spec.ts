// The pose the 3D view draws from the angles the robot reports (src/lib/motion.ts reportedPose): in IDLE the robot
// reports the angles its servos hold, on entering IDLE, after each joint written and after centring, so the model
// shows exactly those and a drag starts from them. A servo the robot reports limp holds no known angle: the model keeps
// the angle it draws for it, so a drag of it starts there and not at the report's 0.
import { describe, expect, it } from 'vitest'
import { ALL_SERVOS, limpOf, reportedPose } from '$lib/motion'

const held = Array.from({ length: 18 }, (_, servo) => [12, 40, -110][servo % 3] + servo)
const drawn = Array.from({ length: 18 }, (_, servo) => [-3, 25, -95][servo % 3] - servo)

describe('reportedPose', () => {
  it('is the angles the robot reports, servo by servo', () =>
    expect(reportedPose({ angles: held, limp: 0 }, drawn)).toEqual(held))

  it('is no pose for a robot that reports none, an empty snapshot or every servo limp', () =>
    expect(reportedPose({ angles: [], limp: 0 }, drawn)).toBeNull())

  it('keeps the drawn angle of every servo reported limp, whose reported 0 is no angle', () => {
    const angles = held.map((angle, servo) => (servo === 4 ? angle : 0))
    const pose = reportedPose({ angles, limp: ALL_SERVOS & ~(1 << 4) }, drawn)
    expect(pose).toEqual(drawn.map((angle, servo) => (servo === 4 ? held[4] : angle)))
  })

  it("keeps the dragged servo at the drag's angle, which the robot's echo trails", () => {
    const pose = reportedPose({ angles: held, limp: 0 }, drawn, { servo: 4, degrees: 33.5 })
    expect(pose?.[4]).toBe(33.5)
    expect(pose?.filter((_, servo) => servo !== 4)).toEqual(held.filter((_, servo) => servo !== 4))
  })
})

describe('limpOf', () => {
  it('is every servo for an empty report, which the robot sends while all are limp', () =>
    expect(limpOf({ angles: [], limp: 0 })).toBe(ALL_SERVOS))

  it('is the servos the robot marks limp alongside its angles', () =>
    expect(limpOf({ angles: held, limp: 0b101 })).toBe(0b101))

  it('is none for a robot that drives every servo, as older firmware always reports', () =>
    expect(limpOf({ angles: held, limp: 0 })).toBe(0))
})
