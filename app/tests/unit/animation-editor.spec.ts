import { readFileSync } from 'node:fs'
import { resolve } from 'node:path'
import { beforeEach, describe, expect, it, vi } from 'vitest'
import { get } from 'svelte/store'

vi.mock('$lib/stores', async () => {
  const { writable } = await import('svelte/store')
  return { outControllerData: writable([0, 0, 0, 0, 0, 0, 0, 0]) }
})

import { Animation, Ease, ParamId } from '$lib/platform_shared/animation'
import { loadAnimationJson, serializeAnimation, type Vec3 } from '$lib/animation/model'
import { DEFAULT_FEET } from '$lib/animation/evaluator'
import { builtIn } from '$lib/animation/library'
import { createEditor, shown, stanceFor } from '$lib/stores/animation-editor'

const crouch = () => {
  const found = builtIn.find(a => a.name === 'crouch')
  if (!found) throw new Error('crouch is not bundled')
  return found
}

describe('editor store', () => {
  let ed: ReturnType<typeof createEditor>

  beforeEach(() => {
    ed = createEditor()
    ed.newDocument()
  })

  it('starts a new document valid, with one keyframe at 0', () => {
    const s = get(ed)
    expect(s.document.keyframes.map(k => k.time)).toEqual([0])
    expect(get(ed.validation)).toBeNull()
    expect(s.dirty).toBe(false)
  })

  it('adds a keyframe half a second after the given one and selects it', () => {
    ed.addKeyframe(0)
    const s = get(ed)
    expect(s.document.keyframes.map(k => k.time)).toEqual([0, 0.5])
    expect(s.selected).toBe(1)
    expect(s.dirty).toBe(true)
  })

  it('inserts between two keyframes that are closer than half a second', () => {
    ed.addKeyframe(0)
    ed.setKeyframeTime(1, 0.3)
    ed.addKeyframe(0)
    expect(get(ed).document.keyframes.map(k => k.time)).toEqual([0, 0.15, 0.3])
  })

  it('refuses a keyframe time that breaks the order and leaves the document unchanged', () => {
    ed.addKeyframe(0)
    ed.addKeyframe(1)
    const before = Animation.toJSON(get(ed).document)
    ed.setKeyframeTime(2, 0.2)
    expect(Animation.toJSON(get(ed).document)).toEqual(before)
    expect(get(ed).error).toMatch(/between/)
    ed.setKeyframeTime(0, 0.1)
    expect(Animation.toJSON(get(ed).document)).toEqual(before)
    ed.setKeyframeTime(2, 1.5)
    expect(get(ed).document.keyframes[2].time).toBe(1.5)
    expect(get(ed).error).toBeNull()
  })

  it('round-trips a foot through joints and back within 1e-3 mm', () => {
    ed.addKeyframe(0)
    ed.setBody(1, 0, 0.1)
    ed.setLeg(1, 2, { joints: false, v: [0, 0, 30] })
    ed.setLeg(1, 2, { joints: true, v: [0, 0, 0] })
    const joints = get(ed).document.keyframes[1].legs[2].joints
    expect(joints).toBeDefined()
    expect(Math.abs(joints!.femur)).toBeGreaterThan(1)
    ed.setLeg(1, 2, { joints: false, v: [0, 0, 0] })
    const foot = get(ed).document.keyframes[1].legs[2].foot!
    expect(foot.x).toBeCloseTo(0, 3)
    expect(foot.y).toBeCloseTo(0, 3)
    expect(foot.z).toBeCloseTo(30, 3)
  })

  it('fills the other legs with stance when a keyframe gets its first leg target', () => {
    ed.setLeg(0, 4, { joints: false, v: [5, 6, 7] })
    const legs = get(ed).document.keyframes[0].legs
    expect(legs).toHaveLength(6)
    expect(legs[4].foot).toEqual({ x: 5, y: 6, z: 7 })
    expect(legs[0].foot).toEqual({ x: 0, y: 0, z: 0 })
  })

  it('changes the angles when scrubbing into a crouch', () => {
    expect(ed.open(crouch())).toBeNull()
    const atStart = get(ed.angles)
    ed.setScrub(0.25)
    const scrubbed = get(ed.angles)
    expect(scrubbed).toHaveLength(18)
    const moved = scrubbed.some((v, i) => Math.abs(v - atStart[i]) > 1)
    expect(moved).toBe(true)
    expect(get(ed.pose).body[5]).toBeGreaterThan(0)
  })

  it('reports a keyframe with three legs as invalid without throwing', () => {
    const broken = Animation.fromPartial({
      name: 'broken',
      schema: 1,
      keyframes: [{ time: 0, legs: [{ foot: {} }, { foot: {} }, { foot: {} }] }]
    })
    const other = createEditor(broken)
    expect(get(other.validation)).toMatch(/0 or 6 legs/)
    expect(get(other.angles)).toHaveLength(18)
  })

  it('refuses to open an invalid document and keeps the current one', () => {
    ed.setMeta({ description: 'kept' })
    const invalid = Animation.fromPartial({ name: 'Bad Name', schema: 1, keyframes: [{ time: 0 }] })
    expect(ed.open(invalid)).toMatch(/name/)
    expect(get(ed).document.description).toBe('kept')
    expect(get(ed).error).toMatch(/name/)
  })

  it('previews against float32 values, as the robot does', () => {
    ed.addKeyframe(0)
    ed.setBody(1, 5, 0.1)
    ed.setScrub(0.5)
    expect(get(ed.pose).body[5]).toBe(Math.fround(0.1))
  })

  it('shows the player pose while previewing and the scrub pose after', () => {
    expect(ed.open(crouch())).toBeNull()
    const player = { body: [0, 0, 0, 0, 0, 42], legs: get(ed.pose).legs }
    ed.setPlayback(player)
    expect(get(ed).playing).toBe(true)
    expect(get(ed.pose).body[5]).toBe(42)
    ed.setPlayback(null)
    expect(get(ed).playing).toBe(false)
    expect(get(ed.pose).body[5]).toBe(0)
  })

  it('keeps the ease and removes keyframes but never the first', () => {
    ed.addKeyframe(0)
    ed.setEase(1, Ease.EASE_OUT)
    ed.duplicateKeyframe(1)
    expect(get(ed).document.keyframes.map(k => [k.time, k.ease])).toEqual([
      [0, Ease.LINEAR],
      [0.5, Ease.EASE_OUT],
      [1, Ease.EASE_OUT]
    ])
    ed.deleteKeyframe(0)
    expect(get(ed).error).toMatch(/first/)
    ed.deleteKeyframe(1)
    expect(get(ed).document.keyframes.map(k => k.time)).toEqual([0, 1])
    expect(get(ed).selected).toBe(0)
  })

  it('refuses a duplicate parameter and keeps overlays editable', () => {
    ed.addKeyframe(0)
    ed.addParam(0)
    ed.addParam(0)
    expect(get(ed).document.params).toHaveLength(1)
    expect(get(ed).error).toMatch(/SPEED/)
    ed.addOverlay()
    ed.updateOverlay(0, { bodyAxis: undefined, footChannel: 5, amplitude: 10 })
    expect(get(ed).document.overlays[0]).toMatchObject({ footChannel: 5, amplitude: 10, end: 0.5 })
    expect(get(ed.validation)).toBeNull()
    ed.removeOverlay(0)
    expect(get(ed).document.overlays).toHaveLength(0)
  })

  it('round-trips a document through JSON unchanged', () => {
    expect(ed.open(crouch())).toBeNull()
    const json = JSON.stringify(Animation.toJSON(get(ed).document))
    const reopened = createEditor()
    expect(reopened.open(Animation.fromJSON(JSON.parse(json)))).toBeNull()
    expect(Animation.toJSON(get(reopened).document)).toEqual(Animation.toJSON(crouch()))
  })
})

describe('stanceFor', () => {
  it('mirrors the firmware feet-distance scale on x and y only', () => {
    expect(stanceFor(0)).toEqual(DEFAULT_FEET.map(([x, y, z, w]) => [x, y, z, w]))
    expect(stanceFor(-1)[0]).toEqual([122 * 0.75, 152 * 0.75, -66, 1])
    expect(stanceFor(1)[1]).toEqual([171 * 1.25, 0, -66, 1])
    expect(stanceFor(3)).toEqual(stanceFor(1))
  })
})

describe('built-in library', () => {
  it('bundles the seven animations by file name', () => {
    expect(builtIn.map(a => a.name)).toEqual([
      'body_roll_test',
      'crouch',
      'play_dead',
      'spooked',
      'stretch',
      'wave',
      'wiggle'
    ])
  })
})

describe('shown', () => {
  it('prints float32 values and FK residues as typed', () => {
    expect(shown(Math.fround(1.2))).toBe(1.2)
    expect(shown(-3.7e-15)).toBe(0)
    expect(Object.is(shown(-3.7e-15), -0)).toBe(false)
    expect(shown(30.92571)).toBe(30.9257)
  })
})

describe('dragging a foot handle', () => {
  const doc = Animation.fromPartial({
    name: 'drag',
    schema: 1,
    keyframes: [
      { time: 0 },
      {
        time: 1,
        ease: Ease.LINEAR,
        legs: [
          { foot: { x: 10, y: 5, z: 20 } },
          { joints: { coxa: 5, femur: 35, tibia: -100 } },
          { foot: {} },
          { foot: {} },
          { foot: {} },
          { foot: {} }
        ]
      }
    ],
    overlays: [
      { footChannel: 2, amplitude: 15, frequency: 1, phase: 0.5, start: 0, end: 1 },
      { footChannel: 3, amplitude: 12, frequency: 1, phase: 0.5, start: 0, end: 1 }
    ],
    params: [{ id: ParamId.FOOT_LIFT, min: 0.5, defaultValue: 1, max: 2 }]
  })
  const delta: Vec3 = [3, -4, 6]

  const offKeyframe = () => {
    const ed = createEditor()
    expect(ed.open(doc)).toBeNull()
    ed.setValue(ParamId.FOOT_LIFT, 1.7)
    ed.selectKeyframe(1)
    ed.setScrub(0.4)
    return ed
  }

  const dragBy = (ed: ReturnType<typeof createEditor>, leg: number) => {
    const handle = get(ed.frame).preview.handleFeet![leg]
    const offset = [0, 1, 2].map(a => handle[a] - DEFAULT_FEET[leg][a] + delta[a]) as Vec3
    expect(ed.dragFoot(leg, offset)).toBeNull()
  }

  it('adds only the drag delta to a foot leg, free of interpolation, overlays and params', () => {
    const ed = offKeyframe()
    const { handleFeet, body } = get(ed.frame).preview
    expect(handleFeet![0].slice(0, 3)).toEqual([122 + 10, 152 + 5, -66 + 20])
    expect(Math.abs(body.feet[0][2] - handleFeet![0][2])).toBeGreaterThan(1)
    dragBy(ed, 0)
    const foot = get(ed).document.keyframes[1].legs[0].foot!
    expect(foot.x).toBeCloseTo(13, 9)
    expect(foot.y).toBeCloseTo(1, 9)
    expect(foot.z).toBeCloseTo(26, 9)
    expect(get(ed).scrub).toBe(1)
  })

  it('turns a joint leg into a foot at its keyframe FK plus the delta', () => {
    const reference = createEditor()
    expect(reference.open(doc)).toBeNull()
    reference.setLeg(1, 1, { joints: false, v: [0, 0, 0] })
    const fk = get(reference).document.keyframes[1].legs[1].foot!
    const ed = offKeyframe()
    dragBy(ed, 1)
    const foot = get(ed).document.keyframes[1].legs[1].foot!
    expect(foot.x).toBeCloseTo(fk.x + delta[0], 6)
    expect(foot.y).toBeCloseTo(fk.y + delta[1], 6)
    expect(foot.z).toBeCloseTo(fk.z + delta[2], 6)
  })

  it('ignores drags while the preview plays', () => {
    const ed = offKeyframe()
    ed.setPlayback(get(ed.pose))
    const before = Animation.toJSON(get(ed).document)
    expect(ed.dragFoot(0, [1, 2, 3])).toBeNull()
    expect(Animation.toJSON(get(ed).document)).toEqual(before)
  })
})

describe('serializeAnimation', () => {
  it('writes float32 values in their shortest decimal form', () => {
    const text = readFileSync(resolve(__dirname, '../../../animations/wave.json'), 'utf8')
    const loaded = loadAnimationJson(text)
    if ('error' in loaded) throw new Error(loaded.error)
    const saved = serializeAnimation(loaded.animation)
    expect(JSON.parse(saved).keyframes[1].body.pitch).toBe(0.08)
    expect(saved).toContain('0.08')
    expect(saved).not.toContain('0.0799')
    expect(saved.endsWith('}\n')).toBe(true)
    expect(saved).toContain('\n  "name": "wave"')
  })

  it('round-trips every built-in exactly', () => {
    for (const a of builtIn) {
      const loaded = loadAnimationJson(serializeAnimation(a))
      if ('error' in loaded) throw new Error(`${a.name}: ${loaded.error}`)
      expect(loaded.animation).toEqual(a)
    }
  })
})
