import { readFileSync } from 'node:fs'
import { resolve } from 'node:path'
import { describe, expect, it } from 'vitest'
import { ParamId } from '$lib/platform_shared/animation'
import Kinematics from '$lib/kinematic'
import { config } from '$lib/components/config'
import { loadAnimationJson } from '$lib/animation/model'
import {
  DEFAULT_FEET,
  capturePose,
  evaluate,
  poseToAngles,
  resolveParams
} from '$lib/animation/evaluator'
import { Player, State } from '$lib/animation/player'

const FIXTURES = resolve(__dirname, '../../../animations/fixtures')
const expected = JSON.parse(readFileSync(resolve(FIXTURES, 'expected.json'), 'utf8'))
const clip = (name: string) => {
  const r = loadAnimationJson(readFileSync(resolve(FIXTURES, `${name}.json`), 'utf8'))
  if ('error' in r) throw new Error(r.error)
  return r.animation
}
const params = (values: Record<string, number>) =>
  new Map(Object.entries(values).map(([k, v]) => [ParamId[k as keyof typeof ParamId], v]))
const kin = new Kinematics(config)

describe('parity with the reference fixtures', () => {
  it('matches every evaluate sample', () => {
    let checked = 0
    for (const c of expected.evaluate) {
      const a = clip(c.animation)
      const p = resolveParams(a, params(c.params))
      for (const s of c.samples) {
        const pose = evaluate(a, p, s.t, kin, DEFAULT_FEET)
        const { angles, mask } = poseToAngles(pose, kin, DEFAULT_FEET)
        expect(mask, `${c.animation} t=${s.t}`).toBe(s.mask)
        angles.forEach((v, j) =>
          expect(v, `${c.animation} t=${s.t} joint ${j}`).toBeCloseTo(s.angles[j], 3)
        )
        checked++
      }
    }
    expect(checked).toBe(59)
  })
  it('matches every player trace step', () => {
    let steps = 0
    for (const [ci, c] of expected.player.entries()) {
      const player = new Player(kin, DEFAULT_FEET)
      const live = {
        omega: c.live.body[0],
        phi: c.live.body[1],
        psi: c.live.body[2],
        xm: c.live.body[3],
        ym: c.live.body[4],
        zm: c.live.body[5],
        feet: DEFAULT_FEET.map((f, i) => [
          f[0] + c.live.feet[i][0],
          f[1] + c.live.feet[i][1],
          f[2] + c.live.feet[i][2],
          1
        ]),
        cumulative_x: 0,
        cumulative_y: 0,
        cumulative_z: 0,
        cumulative_roll: 0,
        cumulative_pitch: 0,
        cumulative_yaw: 0
      }
      player.play(clip(c.animation), params(c.params), capturePose(live, DEFAULT_FEET))
      for (const [step, row] of c.trace.entries()) {
        for (const e of c.events) {
          if (e.step !== step) continue
          if (e.action === 'play') player.play(clip(e.animation), params(e.params), undefined)
          else player.stop()
        }
        const pose = player.update(c.dt)
        const { angles, mask } = poseToAngles(pose, kin, DEFAULT_FEET)
        const where = `case ${ci} step ${step}`
        expect(State[player.state], where).toBe(row.state)
        expect(mask, where).toBe(row.mask)
        angles.forEach((v, j) => expect(v, `${where} joint ${j}`).toBeCloseTo(row.angles[j], 3))
        steps++
      }
    }
    expect(steps).toBe(819)
  })
})
