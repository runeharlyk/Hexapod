import { get, writable } from 'svelte/store'
import { describe, expect, it } from 'vitest'
import {
  Settings,
  type SettingsSource,
  type WatchedSettingsSource
} from '../../src/lib/components/layout/settings.svelte'
import { notifications } from '../../src/lib/components/toasts/notifications'
import { robotData } from '../../src/lib/robot/data'
import type { Watched } from '../../src/lib/transport/topics'

type Bus = { sda: number; scl: number; frequency: number }

const STORED: Bus = { sda: 8, scl: 9, frequency: 400000 }

const source = (overrides: Partial<SettingsSource<Bus>> = {}): SettingsSource<Bus> => ({
  what: 'the I2C settings',
  read: async () => ({ ...STORED }),
  write: async value => value,
  saved: 'Saved.',
  ...overrides
})

const toasts = () => get(notifications).map(({ type, message }) => ({ type, message }))

describe('Settings', () => {
  it('is loading until read, then ready and clean with the read value', async () => {
    const settings = new Settings(source())
    expect(settings.status).toBe('loading')
    expect(settings.dirty).toBe(false)

    await settings.load()

    expect(settings.status).toBe('ready')
    expect(settings.value).toEqual(STORED)
    expect(settings.dirty).toBe(false)
  })

  it('is dirty while the draft differs from the read value, and reset restores it', async () => {
    const settings = new Settings(source())
    await settings.load()

    settings.value.frequency = 100000
    expect(settings.dirty).toBe(true)
    settings.value.frequency = 400000
    expect(settings.dirty).toBe(false)

    settings.value.sda = 21
    settings.reset()
    expect(settings.value).toEqual(STORED)
    expect(settings.dirty).toBe(false)
  })

  it('names a failed read and reads again on load', async () => {
    let fail = true
    const settings = new Settings(
      source({
        read: async () => {
          if (fail) throw new Error('the robot refused the I2C settings')
          return { ...STORED }
        }
      })
    )
    await settings.load()
    expect(settings.status).toBe('failed')
    expect(settings.error).toBe(
      'Could not read the I2C settings: the robot refused the I2C settings'
    )

    fail = false
    await settings.load()
    expect(settings.status).toBe('ready')
  })

  it('writes the draft and adopts what the robot kept as the value reset returns to', async () => {
    const written: Bus[] = []
    const settings = new Settings(
      source({
        write: async value => {
          written.push(value)
          return { ...value, frequency: 100000 }
        }
      })
    )
    await settings.load()
    settings.value.frequency = 150000

    await settings.save()

    expect(written).toEqual([{ ...STORED, frequency: 150000 }])
    expect(settings.value.frequency).toBe(100000)
    expect(settings.dirty).toBe(false)
    expect(settings.pending).toBe(false)
    expect(toasts()).toContainEqual({ type: 'success', message: 'Saved.' })
  })

  it('keeps the draft dirty after a refused write and says why', async () => {
    const settings = new Settings(
      source({
        write: async () => {
          throw new Error('the robot refused the I2C configuration')
        }
      })
    )
    await settings.load()
    settings.value.scl = 22

    await settings.save()

    expect(settings.value.scl).toBe(22)
    expect(settings.dirty).toBe(true)
    expect(settings.pending).toBe(false)
    expect(toasts()).toContainEqual({
      type: 'error',
      message: 'Not saved: the robot refused the I2C configuration'
    })
  })

  it('is pending while its save is in flight', async () => {
    let reply = () => {}
    const settings = new Settings(
      source({ write: value => new Promise(resolve => (reply = () => resolve(value))) })
    )
    await settings.load()
    settings.value.scl = 22

    const saving = settings.save()
    expect(settings.pending).toBe(true)
    reply()
    await saving

    expect(settings.pending).toBe(false)
  })
})

interface Wifi {
  hostname: string
  priority: number
  wifiNetworks: { ssid: string; password: string }[]
  selectedNetwork: number
}

const WIFI: Wifi = {
  hostname: 'hexapod',
  priority: 1,
  wifiNetworks: [
    { ssid: 'home', password: 'a' },
    { ssid: 'work', password: 'b' }
  ],
  selectedNetwork: 0
}

const wifi = (overrides: Partial<Wifi> = {}): Wifi => ({ ...WIFI, ...overrides })

const live = (value: Wifi | undefined, overrides: Partial<Watched<Wifi>> = {}): Watched<Wifi> => ({
  value,
  status: 'live',
  stale: false,
  failure: null,
  pending: false,
  ...overrides
})

const following = (
  initial: Watched<Wifi> = live(wifi()),
  overrides: Partial<WatchedSettingsSource<Wifi>> = {}
) => {
  const watched = writable(initial)
  const written: Wifi[] = []
  const settings = new Settings<Wifi>({
    what: 'the WiFi settings',
    saved: 'Saved.',
    watched,
    write: async value => {
      written.push(value)
    },
    together: [['wifiNetworks', 'selectedNetwork']],
    ...overrides
  })
  const stop = settings.start()
  return { settings, watched, written, stop }
}

describe('Settings following a watched topic', () => {
  it('is loading until the first value, then ready and clean with it', () => {
    const { settings, watched } = following(live(undefined, { status: 'subscribing' }))
    expect(settings.status).toBe('loading')

    watched.set(live(wifi()))

    expect(settings.status).toBe('ready')
    expect(settings.value).toEqual(WIFI)
    expect(settings.dirty).toBe(false)
  })

  it('names a topic that failed before any value', () => {
    const { settings } = following(
      live(undefined, { status: 'error', failure: { kind: 'timeout' } })
    )
    expect(settings.status).toBe('failed')
    expect(settings.error).toMatch(/^Could not read the WiFi settings: /)
  })

  it('follows pending, and stops following when stopped', () => {
    const { settings, watched, stop } = following()
    watched.set(live(wifi(), { pending: true }))
    expect(settings.pending).toBe(true)

    stop()
    watched.set(live(wifi({ priority: 8 })))
    expect(settings.value.priority).toBe(1)
  })

  it('takes a push in an untouched field and stays clean', () => {
    const { settings, watched } = following()
    watched.set(live(wifi({ hostname: 'rover' })))
    expect(settings.value.hostname).toBe('rover')
    expect(settings.dirty).toBe(false)
    expect(settings.notes).toEqual({})
  })

  it('keeps a focused untouched field on its text until blur, then takes the latest value', () => {
    const { settings, watched } = following()
    settings.focus('hostname')

    watched.set(live(wifi({ hostname: 'rover', priority: 5 })))
    watched.set(live(wifi({ hostname: 'scout', priority: 5 })))
    expect(settings.value.hostname).toBe('hexapod')
    expect(settings.value.priority).toBe(5)
    expect(settings.dirty).toBe(false)

    settings.blur('hostname')
    expect(settings.value.hostname).toBe('scout')
    expect(settings.dirty).toBe(false)
  })

  it('notes a robot change made to a field while it was focused and edited after', () => {
    const { settings, watched } = following()
    settings.focus('hostname')
    watched.set(live(wifi({ hostname: 'rover' })))

    settings.value.hostname = 'mine'
    settings.blur('hostname')

    expect(settings.value.hostname).toBe('mine')
    expect(settings.notes).toEqual({ hostname: 'rover' })
  })

  it('compares a field edited after a focused push with the robot value, not the older one', () => {
    const { settings, watched } = following()
    settings.focus('hostname')
    watched.set(live(wifi({ hostname: 'rover' })))
    settings.value.hostname = 'mine'
    settings.blur('hostname')
    expect(settings.dirty).toBe(true)

    settings.value.hostname = 'hexapod'
    expect(settings.dirty).toBe(true)

    settings.value.hostname = 'rover'
    expect(settings.dirty).toBe(false)
  })

  it('drops the note when the edit ends on the robot value', () => {
    const { settings, watched } = following()
    settings.focus('hostname')
    watched.set(live(wifi({ hostname: 'rover' })))
    settings.value.hostname = 'rover'

    settings.blur('hostname')

    expect(settings.notes).toEqual({})
    expect(settings.dirty).toBe(false)
  })

  it('keeps a changed field with a note when the robot changed it too', () => {
    const { settings, watched } = following()
    settings.value.hostname = 'mine'

    watched.set(live(wifi({ hostname: 'theirs', priority: 4 })))

    expect(settings.value.hostname).toBe('mine')
    expect(settings.value.priority).toBe(4)
    expect(settings.notes).toEqual({ hostname: 'theirs' })
    expect(settings.dirty).toBe(true)
  })

  it('drops a note once the draft agrees with the robot', () => {
    const { settings, watched } = following()
    settings.value.hostname = 'mine'
    watched.set(live(wifi({ hostname: 'theirs' })))

    settings.value.hostname = 'theirs'
    watched.set(live(wifi({ hostname: 'theirs', priority: 2 })))

    expect(settings.notes).toEqual({})
    expect(settings.dirty).toBe(false)
  })

  it('merges the snapshot after a drop like a push', () => {
    const { settings, watched } = following()
    settings.value.hostname = 'mine'
    watched.set(live(wifi(), { stale: true, status: 'subscribing' }))
    expect(settings.value.hostname).toBe('mine')

    watched.set(live(wifi({ hostname: 'rebooted', priority: 6 })))

    expect(settings.value).toEqual(wifi({ hostname: 'mine', priority: 6 }))
    expect(settings.notes).toEqual({ hostname: 'rebooted' })
  })

  it('keeps the draft through a value lost after the first, and merges the next one into it', () => {
    const { settings, watched } = following()
    settings.value.hostname = 'mine'
    watched.set(live(wifi({ hostname: 'theirs' })))

    watched.set(live(undefined, { status: 'subscribing' }))
    expect(settings.status).toBe('ready')
    expect(settings.dirty).toBe(true)
    expect(settings.notes).toEqual({ hostname: 'theirs' })

    watched.set(live(wifi({ hostname: 'theirs', priority: 6 })))

    expect(settings.value).toEqual(wifi({ hostname: 'mine', priority: 6 }))
    expect(settings.notes).toEqual({ hostname: 'theirs' })
    expect(settings.dirty).toBe(true)
  })

  it('stays dirty while the topic fails after a lost value, so leaving still asks', () => {
    const { settings, watched } = following()
    settings.value.hostname = 'mine'
    watched.set(live(undefined, { status: 'subscribing' }))

    watched.set(live(undefined, { status: 'error', failure: { kind: 'timeout' } }))

    expect(settings.status).toBe('failed')
    expect(settings.dirty).toBe(true)
  })

  it("adopts the next robot's snapshot whole once the user chose another robot", () => {
    const { settings, watched } = following()
    settings.value.hostname = 'mine'
    watched.set(live(wifi({ hostname: 'theirs' })))

    robotData.reset()
    watched.set(live(undefined, { status: 'subscribing' }))
    expect(settings.status).toBe('loading')
    watched.set(live(wifi({ priority: 4 })))

    expect(settings.value).toEqual(wifi({ priority: 4 }))
    expect(settings.dirty).toBe(false)
    expect(settings.notes).toEqual({})
  })

  it('adopts the next snapshot whatever forgets the old robot first', () => {
    const { settings, watched } = following()
    settings.value.hostname = 'mine'

    watched.set(live(undefined, { status: 'subscribing' }))
    robotData.reset()
    watched.set(live(wifi({ priority: 4 })))

    expect(settings.value).toEqual(wifi({ priority: 4 }))
    expect(settings.dirty).toBe(false)
  })

  it('no longer forgets its robot once stopped', () => {
    const { settings, stop } = following()
    stop()

    robotData.reset()

    expect(settings.status).toBe('ready')
  })

  it('reset returns every field to the current baseline and clears the notes', () => {
    const { settings, watched } = following()
    settings.focus('priority')
    settings.value.hostname = 'mine'
    settings.value.wifiNetworks[0].password = 'typed'
    watched.set(live(wifi({ hostname: 'theirs', priority: 7 })))
    expect(settings.value.priority).toBe(1)
    expect(settings.notes).toEqual({ hostname: 'theirs' })

    settings.reset()

    expect(settings.value).toEqual(wifi({ hostname: 'theirs', priority: 7 }))
    expect(settings.notes).toEqual({})
    expect(settings.dirty).toBe(false)
  })

  it('writes the draft, and a push after the reply brings the value it settles on', async () => {
    const { settings, watched, written } = following()
    settings.value.priority = 9

    await settings.save()
    watched.set(live(wifi({ priority: 9 })))

    expect(written).toEqual([wifi({ priority: 9 })])
    expect(settings.value.priority).toBe(9)
    expect(settings.dirty).toBe(false)
    expect(settings.notes).toEqual({})
    expect(toasts()).toContainEqual({ type: 'success', message: 'Saved.' })
  })

  it('is clean after the save pushed before its reply arrived', async () => {
    let reply = () => {}
    const { settings, watched } = following(live(wifi()), {
      write: () => new Promise<void>(resolve => (reply = resolve))
    })
    settings.value.priority = 9

    const saving = settings.save()
    watched.set(live(wifi({ priority: 9 })))
    reply()
    await saving

    expect(settings.dirty).toBe(false)
    expect(settings.notes).toEqual({})
  })

  it('shows what the robot stored when it normalised a saved value, with no note', async () => {
    let reply = () => {}
    const { settings, watched } = following(live(wifi()), {
      write: () => new Promise<void>(resolve => (reply = resolve))
    })
    settings.value.hostname = 'Mine'

    const saving = settings.save()
    watched.set(live(wifi({ hostname: 'mine' })))
    expect(settings.notes).toEqual({ hostname: 'mine' })
    reply()
    await saving

    expect(settings.value.hostname).toBe('mine')
    expect(settings.dirty).toBe(false)
    expect(settings.notes).toEqual({})
  })

  it('keeps the draft and its notes after a refused write, and says why', async () => {
    const { settings, watched } = following(live(wifi()), {
      write: async () => {
        throw new Error('the robot refused the WiFi settings')
      }
    })
    settings.value.hostname = 'mine'
    watched.set(live(wifi({ hostname: 'theirs' })))

    await settings.save()

    expect(settings.value.hostname).toBe('mine')
    expect(settings.notes).toEqual({ hostname: 'theirs' })
    expect(settings.dirty).toBe(true)
    expect(settings.pending).toBe(false)
    expect(toasts()).toContainEqual({
      type: 'error',
      message: 'Not saved: the robot refused the WiFi settings'
    })
  })

  it('saves a focused field it kept unchanged with the latest value, as leaving it would', async () => {
    const { settings, watched, written } = following()
    settings.value.hostname = 'mine'
    settings.focus('priority')
    watched.set(live(wifi({ priority: 7 })))
    expect(settings.value.priority).toBe(1)

    await settings.save()

    expect(written).toEqual([wifi({ hostname: 'mine', priority: 7 })])
  })

  it('follows the topic for pending, not its own save', async () => {
    let reply = () => {}
    const { settings, watched } = following(live(wifi()), {
      write: () => new Promise<void>(resolve => (reply = resolve))
    })
    settings.value.priority = 9

    const saving = settings.save()
    expect(settings.pending).toBe(false)
    watched.set(live(wifi(), { pending: true }))
    expect(settings.pending).toBe(true)
    reply()
    await saving

    expect(settings.pending).toBe(true)
  })

  it('does not hold a field whose focus a save disabled', async () => {
    const { settings, watched } = following()
    settings.focus('priority')
    settings.value.hostname = 'mine'

    await settings.save()
    watched.set(live(wifi({ hostname: 'mine', priority: 3 })))

    expect(settings.value.priority).toBe(3)
  })
})

describe('Settings following a group with a list the page does not edit', () => {
  interface Bus {
    sda: number
    pins: { pin: number; role: string }[]
  }
  const bus = (sda: number, pins: Bus['pins']): Bus => ({ sda, pins })
  const pinned = (value: Bus): Watched<Bus> => ({
    value,
    status: 'live',
    stale: false,
    failure: null,
    pending: false
  })
  const follow = () => {
    const watched = writable(pinned(bus(8, [])))
    const settings = new Settings<Bus>({
      what: 'the I2C settings',
      saved: 'Saved.',
      watched,
      write: async () => {},
      together: [['pins']]
    })
    settings.start()
    return { settings, watched }
  }

  it('takes a list whose length changed whole, and stays clean', () => {
    const { settings, watched } = follow()
    watched.set(
      pinned(
        bus(8, [
          { pin: 4, role: 'led' },
          { pin: 5, role: 'led' }
        ])
      )
    )
    watched.set(pinned(bus(8, [{ pin: 4, role: 'led' }])))

    expect(settings.value.pins).toEqual([{ pin: 4, role: 'led' }])
    expect(settings.dirty).toBe(false)
  })

  it('keeps an edit to another field when the list changed length under it', () => {
    const { settings, watched } = follow()
    watched.set(
      pinned(
        bus(8, [
          { pin: 4, role: 'led' },
          { pin: 5, role: 'led' }
        ])
      )
    )
    settings.value.sda = 21

    watched.set(pinned(bus(8, [{ pin: 4, role: 'led' }])))

    expect(settings.value).toEqual(bus(21, [{ pin: 4, role: 'led' }]))
    expect(settings.notes).toEqual({})
  })
})
