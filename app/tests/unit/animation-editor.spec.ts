import { readFileSync } from 'node:fs'
import { resolve } from 'node:path'
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { get } from 'svelte/store'

vi.mock('$lib/stores', async () => {
  const { writable } = await import('svelte/store')
  return { outControllerData: writable([0, 0, 0, 0, 0, 0, 0, 0]) }
})

import { Animation, Ease, ParamId } from '$lib/platform_shared/animation'
import { loadAnimationJson, serializeAnimation, type Vec3 } from '$lib/animation/model'
import { DEFAULT_FEET } from '$lib/animation/evaluator'
import {
  builtIn,
  deleteDraft,
  draftAnimations,
  overwritesDraft,
  reloadDrafts,
  saveDraft,
  type DraftRef
} from '$lib/animation/library'
import { Player, State } from '$lib/animation/player'
import {
  commitNumber,
  createEditor,
  kinematics,
  previewClip,
  shown,
  stanceFor
} from '$lib/stores/animation-editor'
import { outControllerData } from '$lib/stores'

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
    const atStart = get(ed.frame).preview.angles
    ed.setScrub(0.25)
    const scrubbed = get(ed.frame).preview.angles
    expect(scrubbed).toHaveLength(18)
    const moved = scrubbed.some((v, i) => Math.abs(v - atStart[i]) > 1 * (Math.PI / 180))
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
    expect(get(other.frame).preview.angles).toHaveLength(18)
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

  it('stops the preview playback when another document opens', () => {
    ed.addKeyframe(0)
    ed.setPlayback({ body: [0, 0, 0, 0, 0, 42], legs: get(ed.pose).legs })
    ed.setScrub(0.3)
    expect(ed.open(crouch())).toBeNull()
    expect(get(ed).playing).toBe(false)
    expect(get(ed).scrub).toBe(0)
    expect(get(ed.pose).body[5]).toBe(0)
    ed.setPlayback({ body: [0, 0, 0, 0, 0, 42], legs: get(ed.pose).legs })
    ed.newDocument()
    expect(get(ed).playing).toBe(false)
  })

  it('mirrors on the robot at the slider height, else at the file ride height', () => {
    ed.setMeta({ rideHeight: 0 })
    ed.setBody(0, 5, 10)
    outControllerData.set([0, 0, 0, 0, 0.6, 0, 0, 0])
    try {
      expect(get(ed.frame).preview.body.zm).toBeCloseTo(10, 6)
      ed.setShowOnRobot(true)
      expect(get(ed.frame).preview.body.zm).toBeCloseTo(40, 6)
      expect(get(ed.pose).body[5]).toBeCloseTo(10, 6)
      ed.setShowOnRobot(false)
      expect(get(ed.frame).preview.body.zm).toBeCloseTo(10, 6)
    } finally {
      outControllerData.set([0, 0, 0, 0, 0, 0, 0, 0])
    }
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

const refuseStorage = () =>
  vi.spyOn(Storage.prototype, 'setItem').mockImplementation(() => {
    throw new DOMException('quota exceeded', 'QuotaExceededError')
  })

describe('drafts', () => {
  afterEach(() => {
    vi.restoreAllMocks()
    localStorage.clear()
    reloadDrafts()
  })

  const named = (name: string) => ({ ...crouch(), name })

  // Another tab writing the same storage; this tab only hears of it through a storage event.
  const writeFromAnotherTab = (drafts: Record<string, unknown>) =>
    localStorage.setItem('animation_drafts', JSON.stringify(drafts))

  it('stores a saved draft in the browser', () => {
    expect(saveDraft(named('stored'))).toEqual({
      name: 'stored',
      fingerprint: JSON.stringify(Animation.toJSON(named('stored')))
    })
    const stored = JSON.parse(localStorage.getItem('animation_drafts')!)
    expect(stored.stored.name).toBe('stored')
    expect(get(draftAnimations).map(a => a.name)).toContain('stored')
  })

  it('reports a refused save and does not list the draft', () => {
    refuseStorage()
    expect(saveDraft(named('refused'))).toBeNull()
    expect(get(draftAnimations).map(a => a.name)).not.toContain('refused')
  })

  it('reports a refused delete and keeps listing the draft', () => {
    expect(saveDraft(named('kept'))).not.toBeNull()
    refuseStorage()
    expect(deleteDraft('kept')).toBe(false)
    expect(get(draftAnimations).map(a => a.name)).toContain('kept')
  })

  it('asks before saving over another document draft, not over its own', () => {
    const own = saveDraft(named('taken'))!
    const other = saveDraft(named('other'))!
    expect(overwritesDraft('taken', null)).toBe(true)
    expect(overwritesDraft('taken', other)).toBe(true)
    expect(overwritesDraft('taken', own)).toBe(false)
    expect(overwritesDraft('free', null)).toBe(false)
  })

  it('asks before saving over its own draft once another tab has changed it', () => {
    const own = saveDraft(named('shared'))!
    writeFromAnotherTab({
      shared: Animation.toJSON({ ...named('shared'), description: 'newer, from another tab' })
    })
    expect(overwritesDraft('shared', own)).toBe(true)
  })

  it('asks before saving under a name another tab has taken meanwhile', () => {
    writeFromAnotherTab({ fresh: Animation.toJSON(named('fresh')) })
    expect(overwritesDraft('fresh', null)).toBe(true)
  })

  it('keeps the drafts another tab saved when this tab saves one', () => {
    saveDraft(named('mine'))
    writeFromAnotherTab({
      mine: Animation.toJSON(named('mine')),
      theirs: Animation.toJSON(named('theirs'))
    })
    saveDraft(named('later'))
    expect(Object.keys(JSON.parse(localStorage.getItem('animation_drafts')!)).sort()).toEqual([
      'later',
      'mine',
      'theirs'
    ])
  })

  it('lists a draft another tab saved when the storage event arrives', () => {
    writeFromAnotherTab({ theirs: Animation.toJSON(named('theirs')) })
    expect(get(draftAnimations).map(a => a.name)).not.toContain('theirs')
    window.dispatchEvent(new StorageEvent('storage', { key: 'animation_drafts' }))
    expect(get(draftAnimations).map(a => a.name)).toEqual(['theirs'])
  })
})

describe('the draft a document belongs to', () => {
  const ref = (name: string): DraftRef => ({ name, fingerprint: `${name} as stored` })

  it('is the draft it was opened from or last saved as, and none for a new document', () => {
    const ed = createEditor()
    expect(get(ed).draft).toBeNull()
    expect(ed.open(crouch(), ref('crouch_copy'))).toBeNull()
    expect(get(ed).draft).toEqual(ref('crouch_copy'))
    ed.markSaved()
    expect(get(ed).draft).toEqual(ref('crouch_copy'))
    ed.markSaved(ref('renamed'))
    expect(get(ed).draft).toEqual(ref('renamed'))
    expect(ed.open(crouch())).toBeNull()
    expect(get(ed).draft).toBeNull()
    ed.markSaved(ref('again'))
    ed.newDocument()
    expect(get(ed).draft).toBeNull()
  })
})

describe('autosave', () => {
  const KEY = 'animation_editor_test'

  afterEach(() => {
    vi.restoreAllMocks()
    localStorage.clear()
  })

  it('restores the unsaved document, its dirty flag and its draft after a reload', () => {
    const first = createEditor(undefined, KEY)
    const mine: DraftRef = { name: 'mine', fingerprint: 'as opened' }
    expect(first.open(crouch(), mine)).toBeNull()
    first.setMeta({ description: 'edited before the reload' })
    first.addKeyframe(0)
    const reloaded = createEditor(undefined, KEY)
    const s = get(reloaded)
    expect(Animation.toJSON(s.document)).toEqual(Animation.toJSON(get(first).document))
    expect(s.dirty).toBe(true)
    expect(s.draft).toEqual(mine)
    expect(get(reloaded.autosaved)).toBe(true)
  })

  it('says so when the browser refuses the autosave', () => {
    const ed = createEditor(undefined, KEY)
    refuseStorage()
    ed.setMeta({ description: 'not stored' })
    expect(get(ed.autosaved)).toBe(false)
    vi.restoreAllMocks()
    ed.setMeta({ description: 'stored' })
    expect(get(ed.autosaved)).toBe(true)
  })

  it('starts from the given document when the stored one is unreadable', () => {
    localStorage.setItem(KEY, '{"document": {"keyframes": []}, "dirty": true}')
    expect(get(createEditor(undefined, KEY)).document.keyframes).toHaveLength(1)
    localStorage.setItem(KEY, 'not json')
    expect(get(createEditor(undefined, KEY)).dirty).toBe(false)
  })

  it('does not write while only the scrub head moves', () => {
    const ed = createEditor(crouch(), KEY)
    const setItem = vi.spyOn(Storage.prototype, 'setItem')
    ed.setScrub(0.2)
    ed.selectKeyframe(1)
    expect(setItem).not.toHaveBeenCalled()
  })
})

describe('a refused number entry', () => {
  it('puts the input back on the document value', () => {
    const ed = createEditor()
    ed.addKeyframe(0)
    ed.addKeyframe(1)
    const input = document.createElement('input')
    input.type = 'number'
    input.value = '0.2'
    commitNumber(input, get(ed).document.keyframes[2].time, v => ed.setKeyframeTime(2, v))
    expect(get(ed).error).toMatch(/between/)
    expect(input.value).toBe('1')
    input.value = ''
    commitNumber(input, get(ed).document.keyframes[2].time, v => ed.setKeyframeTime(2, v))
    expect(input.value).toBe('1')
    input.value = '1.5'
    commitNumber(input, get(ed).document.keyframes[2].time, v => ed.setKeyframeTime(2, v))
    expect(get(ed).document.keyframes[2].time).toBe(1.5)
    expect(input.value).toBe('1.5')
  })
})

describe('the preview Loop', () => {
  // Runs the preview player for the given seconds and returns every state it passed through.
  const statesOver = (clip: Animation, seconds: number) => {
    const player = new Player(kinematics, stanceFor(0))
    player.play(clip, undefined, undefined)
    const seen = new Set<State>()
    for (let t = 0; t < seconds; t += 0.02) {
      player.update(0.02)
      seen.add(player.state)
    }
    return { player, seen }
  }

  it('wraps the playing clock without replaying Exit and Entry', () => {
    const clip = crouch()
    const { player, seen } = statesOver(previewClip(clip, true), 4)
    expect(seen).not.toContain(State.EXIT)
    expect(player.state).toBe(State.PLAYING)
    player.stop()
    expect(player.state).toBe(State.EXIT)
  })

  it('loops a hold_end animation instead of holding it', () => {
    const clip = { ...crouch(), holdEnd: true }
    expect(statesOver(clip, 6).player.state).toBe(State.HOLD)
    const { seen } = statesOver(previewClip(clip, true), 6)
    expect(seen).not.toContain(State.HOLD)
  })

  it('plays the document as written when off', () => {
    const clip = crouch()
    expect(previewClip(clip, false)).toBe(clip)
  })
})
