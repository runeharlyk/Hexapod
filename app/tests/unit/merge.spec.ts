import { describe, expect, it } from 'vitest'
import { fieldOf, mergePush, type Together } from '../../src/lib/components/layout/merge'

interface Wifi {
  hostname: string
  priority: number
  wifiNetworks: { ssid: string; password: string }[]
  selectedNetwork: number
}

const WIFI_TOGETHER: Together = [['wifiNetworks', 'selectedNetwork']]

const wifi = (overrides: Partial<Wifi> = {}): Wifi => ({
  hostname: 'hexapod',
  priority: 1,
  wifiNetworks: [
    { ssid: 'home', password: 'a' },
    { ssid: 'work', password: 'b' }
  ],
  selectedNetwork: 0,
  ...overrides
})

const merge = (draft: Wifi, base: Wifi, next: Wifi, focused: string | null = null) =>
  mergePush(draft, base, next, focused, WIFI_TOGETHER)

describe('fieldOf', () => {
  it('is the first key of the group a path starts in, else the path itself', () => {
    expect(fieldOf('selectedNetwork', WIFI_TOGETHER)).toBe('wifiNetworks')
    expect(fieldOf('wifiNetworks.1.password', WIFI_TOGETHER)).toBe('wifiNetworks')
    expect(fieldOf('hostname', WIFI_TOGETHER)).toBe('hostname')
    expect(fieldOf('servos.3.centerPwm', [])).toBe('servos.3.centerPwm')
  })
})

describe('mergePush', () => {
  const base = wifi()

  it('takes the push in an untouched field', () => {
    const { draft, notes } = merge(base, base, wifi({ hostname: 'rover' }))
    expect(draft.hostname).toBe('rover')
    expect(notes).toEqual({})
  })

  it('keeps a focused untouched field on its text', () => {
    const { draft, notes } = merge(base, base, wifi({ hostname: 'rover', priority: 5 }), 'hostname')
    expect(draft.hostname).toBe('hexapod')
    expect(draft.priority).toBe(5)
    expect(notes).toEqual({})
  })

  it('keeps a changed field and says nothing while the robot has not changed it', () => {
    const edited = wifi({ hostname: 'mine' })
    const { draft, notes } = merge(edited, base, wifi({ priority: 7 }))
    expect(draft).toEqual(wifi({ hostname: 'mine', priority: 7 }))
    expect(notes).toEqual({})
  })

  it('keeps a changed field the robot also changed and notes the robot value', () => {
    const edited = wifi({ hostname: 'mine' })
    const { draft, notes } = merge(edited, base, wifi({ hostname: 'theirs' }))
    expect(draft.hostname).toBe('mine')
    expect(notes).toEqual({ hostname: 'theirs' })
  })

  it('keeps a focused changed field the robot also changed, with the note', () => {
    const edited = wifi({ hostname: 'mine' })
    const { draft, notes } = merge(edited, base, wifi({ hostname: 'theirs' }), 'hostname')
    expect(draft.hostname).toBe('mine')
    expect(notes).toEqual({ hostname: 'theirs' })
  })

  it('keeps a list with edits whole, with a note, when the robot copy changed in content at the same length', () => {
    const edited = wifi({
      wifiNetworks: [
        { ssid: 'home', password: 'typed' },
        { ssid: 'work', password: 'b' }
      ]
    })
    const robot = wifi({
      wifiNetworks: [
        { ssid: 'home', password: 'a' },
        { ssid: 'office', password: 'b' }
      ]
    })
    const { draft, notes } = merge(edited, base, robot)
    expect(draft.wifiNetworks).toEqual(edited.wifiNetworks)
    expect(notes).toEqual({ wifiNetworks: robot.wifiNetworks })
  })

  it('does the same for a list whose length changed', () => {
    const edited = wifi({
      wifiNetworks: [
        { ssid: 'home', password: 'typed' },
        { ssid: 'work', password: 'b' }
      ]
    })
    const robot = wifi({ wifiNetworks: [{ ssid: 'home', password: 'a' }] })
    const { draft, notes } = merge(edited, base, robot)
    expect(draft.wifiNetworks).toEqual(edited.wifiNetworks)
    expect(notes).toEqual({ wifiNetworks: robot.wifiNetworks })
  })

  it('moves selectedNetwork with its list', () => {
    const robot = wifi({ wifiNetworks: [{ ssid: 'lab', password: 'c' }], selectedNetwork: 0 })
    const idle = wifi({ selectedNetwork: 1 })
    const untouched = merge(idle, idle, robot)
    expect(untouched.draft.wifiNetworks).toEqual(robot.wifiNetworks)
    expect(untouched.draft.selectedNetwork).toBe(0)

    const kept = merge(wifi({ selectedNetwork: 1 }), base, robot)
    expect(kept.draft.wifiNetworks).toEqual(base.wifiNetworks)
    expect(kept.draft.selectedNetwork).toBe(1)
    expect(kept.notes).toEqual({ wifiNetworks: robot.wifiNetworks })
  })

  it('takes a list without edits whole', () => {
    const robot = wifi({ wifiNetworks: [{ ssid: 'lab', password: 'c' }] })
    const { draft, notes } = merge(wifi({ hostname: 'mine' }), base, robot)
    expect(draft.wifiNetworks).toEqual(robot.wifiNetworks)
    expect(draft.hostname).toBe('mine')
    expect(notes).toEqual({})
  })

  it('treats servo rows as fields of their own', () => {
    interface Servos {
      servos: { centerPwm: number }[]
    }
    const rows = (...pwm: number[]): Servos => ({ servos: pwm.map(centerPwm => ({ centerPwm })) })
    const { draft, notes } = mergePush(
      rows(300, 300, 300, 350, 300),
      rows(300, 300, 300, 300, 300),
      rows(300, 300, 300, 300, 320),
      null,
      []
    )
    expect(draft).toEqual(rows(300, 300, 300, 350, 320))
    expect(notes).toEqual({})
  })

  describe('a push while saving', () => {
    it('leaves the sent fields clean, with no note, when the push is the save itself', () => {
      const sent = wifi({ hostname: 'mine', priority: 9 })
      const { draft, notes } = merge(sent, base, sent)
      expect(draft).toEqual(sent)
      expect(notes).toEqual({})
    })

    it('notes another field the user also edited that the push changed', () => {
      const sent = wifi({ hostname: 'mine', priority: 9 })
      const push = wifi({ hostname: 'mine', priority: 4 })
      const { draft, notes } = merge(sent, base, push)
      expect(draft.priority).toBe(9)
      expect(notes).toEqual({ priority: 4 })
    })
  })

  it('merges the snapshot after a drop by the same rule', () => {
    const edited = wifi({ hostname: 'mine' })
    const snapshot = wifi({ hostname: 'rebooted', priority: 3 })
    const { draft, notes } = merge(edited, base, snapshot, 'priority')
    expect(draft).toEqual(wifi({ hostname: 'mine', priority: 1 }))
    expect(notes).toEqual({ hostname: 'rebooted' })
  })
})
