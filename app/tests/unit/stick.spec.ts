import { describe, expect, it } from 'vitest'
import { gamepadSticks, shapeStick } from '../../src/lib/utilities/stick'

const polar = (radius: number, degrees: number) => ({
  x: radius * Math.cos((degrees * Math.PI) / 180),
  y: radius * Math.sin((degrees * Math.PI) / 180)
})
const angle = (v: { x: number; y: number }) => (Math.atan2(v.y, v.x) * 180) / Math.PI
const length = (v: { x: number; y: number }) => Math.hypot(v.x, v.y)

describe('shapeStick', () => {
  it('ignores a stick resting near the centre, in any direction', () => {
    expect(shapeStick({ x: 0.1, y: 0.1 })).toEqual({ x: 0, y: 0 })
    expect(shapeStick({ x: -0.14, y: 0 })).toEqual({ x: 0, y: 0 })
    expect(shapeStick({ x: 0.1, y: -0.1 })).toEqual({ x: 0, y: 0 })
  })

  it('treats a non-finite reading as a stick at rest', () => {
    expect(shapeStick({ x: NaN, y: 0.5 })).toEqual({ x: 0, y: 0 })
    expect(shapeStick({ x: 0, y: Infinity })).toEqual({ x: 0, y: 0 })
  })

  it('reaches full speed at the rim, rising from zero at the dead zone', () => {
    expect(length(shapeStick({ x: 0, y: 1 }))).toBeCloseTo(1)
    expect(length(shapeStick(polar(1, 45)))).toBeCloseTo(1)
    expect(length(shapeStick(polar(0.151, 45)))).toBeLessThan(0.01)
    expect(length(shapeStick(polar(0.575, 45)))).toBeCloseTo(0.5)
  })

  it('never exceeds 1 when a square-gated stick reports the corner', () => {
    expect(length(shapeStick({ x: 1, y: 1 }))).toBeCloseTo(1)
  })

  it('drops the sideways drift of a stick pushed mostly forward', () => {
    const shaped = shapeStick({ x: -0.14, y: 1 })
    expect(shaped.x).toBe(0)
    expect(shaped.y).toBeCloseTo(1)
  })

  it('snaps to every axis, sideways and backwards too', () => {
    expect(shapeStick(polar(1, 5)).y).toBe(0)
    expect(shapeStick(polar(1, 185)).y).toBe(0)
    expect(shapeStick(polar(1, -95)).x).toBe(0)
    expect(shapeStick(polar(1, 85)).x).toBe(0)
  })

  it('keeps a deliberate diagonal for strafing', () => {
    expect(angle(shapeStick(polar(1, 45)))).toBeCloseTo(45)
    expect(angle(shapeStick(polar(1, -135)))).toBeCloseTo(-135)
    expect(angle(shapeStick(polar(1, 30)))).toBeCloseTo(30)
  })

  it('turns the direction without a jump across the edge of the snap', () => {
    let previous = angle(shapeStick(polar(1, 0)))
    for (let degrees = 0.5; degrees <= 45; degrees += 0.5) {
      const current = angle(shapeStick(polar(1, degrees)))
      expect(current - previous).toBeGreaterThanOrEqual(0)
      expect(current - previous).toBeLessThan(1.1)
      previous = current
    }
    expect(angle(shapeStick(polar(1, 20)))).toBeCloseTo(20)
  })
})

describe('gamepadSticks', () => {
  // The standard mapping reports a stick pushed up as -1 on its second axis.
  const pushedUp = [0, -1, 0.6, 0]

  it('shapes both sticks, up as +1 as the firmware takes it', () => {
    const sticks = gamepadSticks(pushedUp, true)
    expect(sticks?.left.y).toBeCloseTo(1)
    expect(sticks?.right).toEqual(shapeStick({ x: 0.6, y: -0 }))
  })

  it('takes no sticks in a mode that shows none, as the keys take none there', () =>
    expect(gamepadSticks(pushedUp, false)).toBeNull())
})
