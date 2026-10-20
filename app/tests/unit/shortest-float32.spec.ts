import { describe, expect, it } from 'vitest'
import { shortestFloat32 } from '../../src/lib/utilities/math-utilities'

describe('shortestFloat32', () => {
  it('reads a float32 back as the decimal that was stored', () => {
    expect(shortestFloat32(Math.fround(2.2))).toBe(2.2)
    expect(shortestFloat32(Math.fround(-0.35))).toBe(-0.35)
    expect(shortestFloat32(Math.fround(306))).toBe(306)
  })

  it('stays the same float32, so a save sends the robot what it has', () => {
    for (const value of [2.2, 0.1, 1 / 3, 123.456, 1e-7, -179.99])
      expect(Math.fround(shortestFloat32(Math.fround(value)))).toBe(Math.fround(value))
  })
})
