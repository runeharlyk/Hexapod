import { readFileSync } from 'node:fs'
import { resolve } from 'node:path'
import { describe, expect, it } from 'vitest'
import { Animation, Ease } from '$lib/platform_shared/animation'
import {
  froundAnimation,
  legOf,
  legTarget,
  loadAnimationJson,
  stanceLeg
} from '$lib/animation/model'

const validation: { case: string; fragments: string[]; animation: unknown }[] = JSON.parse(
  readFileSync(resolve(__dirname, '../../../animations/fixtures/validation.json'), 'utf8')
)

const two = (extra: Parameters<typeof Animation.fromPartial>[0] = {}): Animation =>
  Animation.fromPartial({
    name: 't',
    schema: 1,
    keyframes: [{ time: 0 }, { time: 1 }],
    ...extra
  })

// The cases, and the fragments each first error must contain, come from the Python reference
// (simulation/gen_animation_fixtures.py); the firmware's test reads the same cases.
describe('validate', () => {
  it.each(validation.map(c => [c.case, c] as const))('%s', (_, c) => {
    const loaded = loadAnimationJson(JSON.stringify(c.animation))
    if (c.fragments.length === 0) expect(loaded).toHaveProperty('animation')
    else
      for (const fragment of c.fragments)
        expect(loaded).toHaveProperty('error', expect.stringContaining(fragment))
  })
})

describe('legs and rounding', () => {
  it('reads a leg target as foot or joints and an empty target as stance', () => {
    expect(legOf({ joints: { coxa: 1, femur: 2, tibia: 3 } })).toEqual({
      joints: true,
      v: [1, 2, 3]
    })
    expect(legOf({ foot: { x: 0, y: 0, z: 5 } })).toEqual({ joints: false, v: [0, 0, 5] })
    expect(legOf({})).toEqual(stanceLeg())
    expect(legTarget({ time: 0, ease: Ease.LINEAR, body: undefined, legs: [] }, 4)).toEqual(
      stanceLeg()
    )
  })
  it('rounds every number to float32', () => {
    const a = froundAnimation(
      two({
        entryTime: 0.4,
        keyframes: [
          { time: 0 },
          { time: 0.53, body: { roll: 0.1, pitch: 0, yaw: 0, x: 0, y: 0, z: 15 } }
        ]
      })
    )
    expect(a.entryTime).toBe(Math.fround(0.4))
    expect(a.keyframes[1].time).toBe(Math.fround(0.53))
    expect(a.keyframes[1].body?.roll).toBe(Math.fround(0.1))
  })
  it('loads JSON and reports a validation error instead of a document', () => {
    const ok = loadAnimationJson('{"name":"x","schema":1,"keyframes":[{"time":0}]}')
    expect('animation' in ok && ok.animation.name).toBe('x')
    const bad = loadAnimationJson(
      '{"name":"x","schema":1,"keyframes":[{"time":0,"legs":[{},{},{}]}]}'
    )
    expect('error' in bad && bad.error).toContain('0 or 6')
    const junk = loadAnimationJson('{not json')
    expect('error' in junk && junk.error).toMatch(/JSON/)
  })
})
