// The home page's check of a saved robot (src/lib/services/saved-robot-refresh.ts): its address first, then the IP it
// last reported, which counts only when the robot there is the saved one.
import { beforeEach, describe, expect, it, vi } from 'vitest'
import { get } from 'svelte/store'
import type { Robot } from '$lib/stores/robots'

const network = vi.hoisted(() => ({
  online: new Set<string>(),
  reports: new Map<string, { robotId: string | null }>(),
  probed: [] as string[]
}))
vi.mock('$lib/services/discovery', () => ({
  probeAddress: async (address: string) => {
    network.probed.push(address)
    return network.online.has(address)
  }
}))
vi.mock('$lib/services/firmware-report', () => ({
  readFirmwareReport: async (address: string) => {
    const report = network.reports.get(address)
    return report ? { version: '0.2.0', build: 'aa', servosCalibrated: true, ...report } : null
  }
}))

const A = 'aaaaaaaaaaaa'
const B = 'bbbbbbbbbbbb'
const IP = '192.168.1.50'

const load = async (saved: object[]) => {
  vi.resetModules()
  const { robots } = await import('$lib/stores/robots')
  robots.set(saved.map(r => ({ lastSeenAt: null, ...r })) as Robot[])
  const { refreshSavedRobot } = await import('$lib/services/saved-robot-refresh')
  return { robots, refreshSavedRobot }
}

const savedRobot = { name: 'Spider', address: 'hexapod.local', robotId: A, lastIp: IP }

describe('refreshSavedRobot', () => {
  beforeEach(() => {
    network.online.clear()
    network.reports.clear()
    network.probed = []
  })

  it('a saved robot unreachable by name is probed at its lastIp, and counts as online only when that IP reports the same robotId', async () => {
    const { robots, refreshSavedRobot } = await load([savedRobot])
    network.online.add(IP)
    network.reports.set(IP, { robotId: A })

    await expect(refreshSavedRobot('hexapod.local')).resolves.toEqual({
      online: true,
      reported: true
    })
    expect(network.probed).toEqual(['hexapod.local', IP])
    expect(get(robots)[0].lastSeenAt).not.toBeNull()
    expect(get(robots)[0].firmware?.version).toBe('0.2.0')

    network.reports.set(IP, { robotId: B })
    await expect(refreshSavedRobot('hexapod.local')).resolves.toEqual({
      online: false,
      reported: false
    })
  })

  it('records nothing of another robot that answers at the IP', async () => {
    const { robots, refreshSavedRobot } = await load([savedRobot])
    network.online.add(IP)
    network.reports.set(IP, { robotId: B })

    await refreshSavedRobot('hexapod.local')

    expect(get(robots)[0].lastSeenAt).toBeNull()
    expect(get(robots)[0].firmware).toBeUndefined()
  })

  it('is offline for an IP that answers without saying which robot it is', async () => {
    const { refreshSavedRobot } = await load([savedRobot])
    network.online.add(IP)

    await expect(refreshSavedRobot('hexapod.local')).resolves.toMatchObject({ online: false })
    network.reports.set(IP, { robotId: null })
    await expect(refreshSavedRobot('hexapod.local')).resolves.toMatchObject({ online: false })
  })

  it('is offline when neither the name nor the IP answers', async () => {
    const { refreshSavedRobot } = await load([savedRobot])

    await expect(refreshSavedRobot('hexapod.local')).resolves.toMatchObject({ online: false })
    expect(network.probed).toEqual(['hexapod.local', IP])
  })

  it('asks no IP of a robot with none, or one with no id to compare it by', async () => {
    const { refreshSavedRobot } = await load([
      { name: 'a', address: 'a.local', robotId: A },
      { name: 'b', address: 'b.local', lastIp: IP }
    ])

    await refreshSavedRobot('a.local')
    await refreshSavedRobot('b.local')

    expect(network.probed).toEqual(['a.local', 'b.local'])
  })

  it('does not look past a name that answers, and learns the id the robot reports', async () => {
    const { robots, refreshSavedRobot } = await load([
      { name: 'a', address: 'a.local', lastIp: IP }
    ])
    network.online.add('a.local')
    network.reports.set('a.local', { robotId: A })

    await expect(refreshSavedRobot('a.local')).resolves.toEqual({ online: true, reported: true })

    expect(network.probed).toEqual(['a.local'])
    expect(get(robots)[0].robotId).toBe(A)
  })

  it('is online without a report from a robot that answers but reports nothing', async () => {
    const { refreshSavedRobot } = await load([savedRobot])
    network.online.add('hexapod.local')

    await expect(refreshSavedRobot('hexapod.local')).resolves.toEqual({
      online: true,
      reported: false
    })
  })
})
