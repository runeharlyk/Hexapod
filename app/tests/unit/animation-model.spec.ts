import { describe, expect, it } from 'vitest'
import { Animation, Ease, ParamId } from '$lib/platform_shared/animation'
import {
  froundAnimation,
  legOf,
  legTarget,
  loadAnimationJson,
  stanceLeg,
  validate
} from '$lib/animation/model'

const two = (extra: Parameters<typeof Animation.fromPartial>[0] = {}): Animation =>
  Animation.fromPartial({
    name: 't',
    schema: 1,
    keyframes: [{ time: 0 }, { time: 1 }],
    ...extra
  })

describe('validate', () => {
  it('accepts a minimal animation', () => {
    expect(validate(two())).toBeNull()
  })
  it.each<[string, (a: Animation) => void, string]>([
    ['schema', a => void (a.schema = 2), 'schema'],
    ['name charset', a => void (a.name = 'Bad Name'), 'name'],
    ['name length', a => void (a.name = 'x'.repeat(33)), 'name'],
    ['description bytes', a => void (a.description = 'e'.repeat(97)), 'description'],
    ['no keyframes', a => void (a.keyframes = []), 'keyframe'],
    ['first time', a => void (a.keyframes[0].time = 0.1), 'time 0'],
    ['increasing', a => void (a.keyframes[1].time = 0), 'increase'],
    ['legs count', a => void (a.keyframes[1].legs = [{}, {}, {}]), '0 or 6'],
    ['ease range', a => void (a.keyframes[1].ease = 4 as Ease), 'ease'],
    ['nan time', a => void (a.keyframes[1].time = NaN), 'finite'],
    ['loop and hold', a => void ((a.loop = true), (a.holdEnd = true)), 'loop'],
    [
      'overlay channel',
      a => a.overlays.push({ amplitude: 1, frequency: 1, phase: 0, start: 0, end: 1 }),
      'channel'
    ],
    [
      'overlay body axis',
      a =>
        a.overlays.push({
          bodyAxis: 6,
          amplitude: 1,
          frequency: 1,
          phase: 0,
          start: 0,
          end: 1
        }),
      'body_axis'
    ],
    [
      'overlay window',
      a =>
        a.overlays.push({
          bodyAxis: 0,
          amplitude: 1,
          frequency: 1,
          phase: 0,
          start: 0.5,
          end: 0.5
        }),
      'start'
    ],
    [
      'overlay end',
      a =>
        a.overlays.push({
          bodyAxis: 0,
          amplitude: 1,
          frequency: 1,
          phase: 0,
          start: 0,
          end: 1.5
        }),
      'end'
    ],
    [
      'param unique',
      a =>
        a.params.push(
          { id: ParamId.SPEED, min: 0.5, defaultValue: 1, max: 2 },
          { id: ParamId.SPEED, min: 0.5, defaultValue: 1, max: 2 }
        ),
      'unique'
    ],
    [
      'param order',
      a => a.params.push({ id: ParamId.BODY_Z, min: 0.5, defaultValue: 3, max: 2 }),
      'min <= default_value <= max'
    ],
    [
      'speed min',
      a => a.params.push({ id: ParamId.SPEED, min: 0, defaultValue: 1, max: 2 }),
      'SPEED'
    ],
    [
      'repeat min',
      a => a.params.push({ id: ParamId.REPEAT, min: 0, defaultValue: 1, max: 2 }),
      'REPEAT'
    ],
    [
      'param id',
      a => a.params.push({ id: 10 as ParamId, min: 0.5, defaultValue: 1, max: 2 }),
      'param id'
    ],
    ['ride height', a => void (a.rideHeight = Infinity), 'ride_height']
  ])('reports %s', (_, mutate, fragment) => {
    const a = two()
    mutate(a)
    expect(validate(a)).toContain(fragment)
  })
  it('bounds the counts', () => {
    expect(
      validate(two({ keyframes: Array.from({ length: 33 }, (_, i) => ({ time: i })) }))
    ).toContain('32')
    expect(
      validate(
        two({
          overlays: Array(9).fill({
            bodyAxis: 0,
            amplitude: 1,
            frequency: 1,
            phase: 0,
            start: 0,
            end: 1
          })
        })
      )
    ).toContain('8')
    expect(
      validate(
        two({
          params: Array.from({ length: 11 }, (_, i) => ({
            id: i % 10,
            min: 1,
            defaultValue: 1,
            max: 1
          }))
        })
      )
    ).toContain('10')
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
