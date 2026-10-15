import { readFileSync } from 'node:fs'
import { resolve } from 'node:path'
import { describe, expect, it } from 'vitest'
import { Animation, Ease, ParamId, type LegTarget } from '$lib/platform_shared/animation'
import Kinematics from '$lib/kinematic'
import { config } from '$lib/components/config'
import {
  BodyAxis,
  STEP_ARC_FULL_TRAVEL_MM,
  STEP_ARC_MM,
  clonePose,
  loadAnimationJson,
  stancePose,
  type Pose
} from '$lib/animation/model'
import { DEFAULT_FEET, easeValue, poseToAngles } from '$lib/animation/evaluator'
import { Player, State } from '$lib/animation/player'

const kin = new Kinematics(config)
const DT = 0.02
const RUNNER_DT = 0.005 // the firmware control tick
const STAND_SMOOTHING = 0.06
const WAVE = resolve(__dirname, '../../../animations/wave.json')

const stanceLegs = (n: number): LegTarget[] =>
  Array.from({ length: n }, () => ({ foot: { x: 0, y: 0, z: 0 } }))
const relocated = (): LegTarget[] => [{ foot: { x: 0, y: 30, z: 20 } }, ...stanceLegs(5)]

// Leg 0 relocates 30 mm forward and 20 mm up over 1 s, body crouches 20 mm.
const liftedAnim = (extra: Parameters<typeof Animation.fromPartial>[0] = {}): Animation =>
  Animation.fromPartial({
    name: 'lift',
    schema: 1,
    keyframes: [
      { time: 0, legs: relocated() },
      { time: 1, body: { z: 20 }, legs: relocated() }
    ],
    ...extra
  })

// Whole control steps, with one step of slack past every transition so float accumulation in
// the blend clock cannot flip an assertion.
const run = (player: Player, steps: number): Pose[] =>
  Array.from({ length: steps }, () => player.update(DT))
const last = (poses: Pose[]) => poses[poses.length - 1]

const approx = (actual: number, expected: number, abs = 1e-6) =>
  expect(Math.abs(actual - expected)).toBeLessThanOrEqual(abs)
// numpy.allclose defaults: |a - b| <= 1e-8 + 1e-5 * |b|
const allClose = (a: readonly number[], b: readonly number[]) =>
  expect(a.every((v, i) => Math.abs(v - b[i]) <= 1e-8 + 1e-5 * Math.abs(b[i]))).toBe(true)

const basedAngles = (pose: Pose, base: number) => {
  const out = clonePose(pose)
  out.body[BodyAxis.Z] += base
  return poseToAngles(out, kin, DEFAULT_FEET).angles
}

describe('Player', () => {
  it('enters from the live pose and arcs a relocating foot', () => {
    const p = new Player(kin, DEFAULT_FEET)
    p.play(liftedAnim({ entryTime: 0.4 }), undefined, undefined)
    expect(p.state).toBe(State.ENTRY)
    const mid = last(run(p, 10)) // u = 0.5, easeInOut(0.5) = 0.5, sin(pi/2) = 1
    const travel = 30
    const expectedZ = 10 + STEP_ARC_MM * Math.min(1, travel / STEP_ARC_FULL_TRAVEL_MM)
    approx(mid.legs[0].v[1], 15)
    approx(mid.legs[0].v[2], expectedZ)
    approx(mid.legs[1].v[2], 0) // planted feet get no arc
    const end = last(run(p, 11))
    expect(p.state).toBe(State.PLAYING)
    allClose(end.legs[0].v, [0, 30, 20])
  })

  it('freezes on the last keyframe with hold_end', () => {
    const p = new Player(kin, DEFAULT_FEET)
    p.play(liftedAnim({ entryTime: 0.2, holdEnd: true }), undefined, undefined)
    run(p, 75)
    expect(p.state).toBe(State.HOLD)
    const pose = last(run(p, 50))
    approx(pose.body[BodyAxis.Z], 20)
    p.stop()
    expect(p.state).toBe(State.EXIT)
  })

  it('wraps a loop and counts REPEAT plays', () => {
    const p = new Player(kin, DEFAULT_FEET)
    p.play(liftedAnim({ entryTime: 0.2, loop: true }), undefined, undefined)
    run(p, 11)
    run(p, 125)
    expect(p.state).toBe(State.PLAYING)
    expect(p.t).toBeGreaterThanOrEqual(0)
    expect(p.t).toBeLessThan(1)
    const q = new Player(kin, DEFAULT_FEET)
    q.play(
      liftedAnim({
        entryTime: 0.2,
        params: [{ id: ParamId.REPEAT, min: 1, defaultValue: 2, max: 5 }]
      }),
      undefined,
      undefined
    )
    run(q, 11)
    run(q, 74)
    expect(q.state).toBe(State.PLAYING) // second play under way
    run(q, 30)
    expect(q.state).toBe(State.EXIT)
  })

  it('reaches hold or exit with zero dt and a single keyframe', () => {
    const one = Animation.fromPartial({
      name: 'one',
      schema: 1,
      entryTime: 0.2,
      keyframes: [{ time: 0, body: { z: 10 } }]
    })
    const p = new Player(kin, DEFAULT_FEET)
    p.play(one, undefined, undefined)
    p.update(0)
    expect(p.state).toBe(State.ENTRY)
    run(p, 13)
    expect(p.state).toBe(State.EXIT)
    const q = new Player(kin, DEFAULT_FEET)
    q.play({ ...one, holdEnd: true }, undefined, undefined)
    run(q, 13)
    expect(q.state).toBe(State.HOLD)
  })

  it('enters from the current exit blend without a jump when played during exit', () => {
    const p = new Player(kin, DEFAULT_FEET)
    p.play(liftedAnim({ entryTime: 0.2, exitTime: 0.4 }), undefined, undefined)
    run(p, 11)
    run(p, 55)
    const before = last(run(p, 5))
    expect(p.state).toBe(State.EXIT)
    p.play(liftedAnim({ entryTime: 0.4 }), undefined, undefined)
    const after = p.update(0)
    expect(p.state).toBe(State.ENTRY)
    allClose(after.body, before.body)
    allClose(after.legs[0].v, before.legs[0].v)
  })

  it('blends from the live pose when stopped before the first update', () => {
    const live = stancePose()
    live.body = [0.05, 0, 0, 0, 0, 12]
    live.legs[0].v = [10, 0, 5]
    const p = new Player(kin, DEFAULT_FEET)
    p.play(liftedAnim(), undefined, live)
    p.stop()
    const pose = p.update(0)
    allClose(pose.body, live.body)
    allClose(pose.legs[0].v, live.legs[0].v)
  })

  it('meets playing on the seam after a fixed ride-height entry', () => {
    // The runner moves its base along the Entry blend to the base Entry converted at, then eases
    // it toward the clip's height; a short Entry far from that height must still land on the seam.
    const r = loadAnimationJson(readFileSync(WAVE, 'utf8'))
    if ('error' in r) throw new Error(r.error)
    const a = { ...r.animation, rideHeight: 40, entryTime: 0.1 }
    const start = -40
    let base = start
    const p = new Player(kin, DEFAULT_FEET)
    p.play(a, undefined, stancePose(), base)
    let endOfEntry: number[] = []
    while (p.state === State.ENTRY) {
      const pose = p.update(RUNNER_DT, base)
      base = start + (p.entryBase - start) * easeValue(Ease.EASE_IN_OUT, p.blendFraction())
      endOfEntry = basedAngles(pose, base)
    }
    approx(base, 40)
    base += (a.rideHeight - base) * STAND_SMOOTHING
    const firstPlaying = basedAngles(p.update(RUNNER_DT, base), base)
    expect(p.state).toBe(State.PLAYING)
    const jump = Math.max(...endOfEntry.map((v, i) => Math.abs(v - firstPlaying[i])))
    expect(jump).toBeLessThan(0.05)
  })
})
