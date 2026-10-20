import { get } from 'svelte/store'
import { describe, expect, it } from 'vitest'
import { Settings, type SettingsSource } from '../../src/lib/components/layout/settings.svelte'
import { notifications } from '../../src/lib/components/toasts/notifications'

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
    expect(settings.saving).toBe(false)
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
    expect(settings.saving).toBe(false)
    expect(toasts()).toContainEqual({
      type: 'error',
      message: 'Not saved: the robot refused the I2C configuration'
    })
  })
})
