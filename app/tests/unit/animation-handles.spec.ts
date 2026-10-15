import { describe, expect, it } from 'vitest'
import {
  HANDLE_NEUTRAL,
  HANDLE_RED,
  handleColor,
  legMask,
  sceneToOffset,
  stanceToScene
} from '$lib/animation/handles'
import { DEFAULT_FEET } from '$lib/animation/evaluator'
import type { Vec3 } from '$lib/animation/model'

const stance: Vec3 = [122, 152, -66]

const add = (a: Vec3, b: Vec3): Vec3 => [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
const expectClose = (actual: Vec3, expected: Vec3) =>
  actual.forEach((v, i) => expect(v).toBeCloseTo(expected[i], 9))

describe('foot handle mapping', () => {
  it('reads a stance foot back as a zero offset', () => {
    expect(sceneToOffset(stanceToScene(stance), stance)).toEqual([0, 0, 0])
  })

  it('reads every axis of a moved scene point back as the body-frame offset', () => {
    for (const offset of [
      [10, 0, 0],
      [0, 10, 0],
      [0, 0, 10],
      [-7.5, 3.25, -12]
    ] as Vec3[]) {
      const delta = stanceToScene(add(stance, offset)).map(
        (v, i) => v - stanceToScene(stance)[i]
      ) as Vec3
      const moved = add(stanceToScene(stance), delta)
      expectClose(sceneToOffset(moved, stance), offset)
    }
  })

  it('places a hand-computed point by the orient_robot mapping [y, z + 66, x] / 12', () => {
    expectClose(stanceToScene([12, 24, -66]), [2, 0, 1])
    expectClose(sceneToOffset([2, 0, 1], [0, 0, -66]), [12, 24, 0])
  })

  it('maps body x, y and z onto distinct scene axes with z on the scene up axis', () => {
    const origin = stanceToScene(stance)
    const axisOf = (offset: Vec3) =>
      stanceToScene(add(stance, offset))
        .map((v, i) => v - origin[i])
        .map(v => Math.abs(v) > 1e-12)
    expect(axisOf([10, 0, 0])).toEqual([false, false, true])
    expect(axisOf([0, 10, 0])).toEqual([true, false, false])
    expect(axisOf([0, 0, 10])).toEqual([false, true, false])
    expect(stanceToScene(add(stance, [0, 0, 10]))[1]).toBeGreaterThan(origin[1])
  })

  it('places a default stance foot on the ground plane', () => {
    for (const foot of DEFAULT_FEET) expect(stanceToScene(foot)[1]).toBeCloseTo(0, 9)
  })
})

describe('foot handle colour', () => {
  it('is neutral for a clean leg and red for any flagged joint', () => {
    expect(handleColor(0)).toBe(HANDLE_NEUTRAL)
    expect(handleColor(0x6)).toBe(HANDLE_RED)
    expect(handleColor(0x1)).toBe(HANDLE_RED)
    expect(HANDLE_NEUTRAL).not.toBe(HANDLE_RED)
  })

  it('extracts the three bits of one leg from the 18-bit mask', () => {
    const mask = (0b110 << (5 * 3)) | (0b001 << (1 * 3))
    expect([0, 1, 2, 3, 4, 5].map(leg => legMask(mask, leg))).toEqual([0, 1, 0, 0, 0, 6])
  })
})
