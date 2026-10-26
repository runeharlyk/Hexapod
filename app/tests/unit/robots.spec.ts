// The saved robots' identity (src/lib/stores/robots.ts): every robot ships with the mDNS name hexapod, so a saved
// robot is told by the robot_id it reports, and follows its own rename.
import { beforeEach, describe, expect, it, vi } from 'vitest'
import { get } from 'svelte/store'

const loadRobots = async () => {
  vi.resetModules()
  localStorage.clear()
  return import('../../src/lib/stores/robots')
}

const report = (robotId: string | null) => ({
  version: '0.2.0',
  build: 'aa',
  servosCalibrated: true,
  robotId
})

describe('followRename', () => {
  it("rewrites only the matching robot's .local address", async () => {
    const { robots, addRobot, recordFirmware, followRename } = await loadRobots()
    addRobot('esp32.local')
    addRobot('hexapod.local')
    recordFirmware('esp32.local', report('aaaaaaaaaaaa'))
    recordFirmware('hexapod.local', report('bbbbbbbbbbbb'))

    expect(followRename('aaaaaaaaaaaa', 'hexapod2')).toEqual({
      from: 'esp32.local',
      to: 'hexapod2.local'
    })

    expect(get(robots).map(r => r.address)).toEqual(['hexapod2.local', 'hexapod.local'])
    expect(get(robots)[1].robotId).toBe('bbbbbbbbbbbb')
  })

  it('moves the name with the address only when it was the address', async () => {
    const { robots, addRobot, renameRobot, recordFirmware, followRename } = await loadRobots()
    addRobot('esp32.local')
    addRobot('hexapod.local', 'Spider')
    recordFirmware('esp32.local', report('aaaaaaaaaaaa'))
    recordFirmware('hexapod.local', report('bbbbbbbbbbbb'))

    followRename('aaaaaaaaaaaa', 'kitchen')
    followRename('bbbbbbbbbbbb', 'garage')

    expect(get(robots).map(r => [r.name, r.address])).toEqual([
      ['kitchen.local', 'kitchen.local'],
      ['Spider', 'garage.local']
    ])
    renameRobot('garage.local', 'Garage')
    expect(get(robots)[1].name).toBe('Garage')
  })

  it('moves nothing when the hostname is the name the address already has', async () => {
    const { robots, addRobot, recordFirmware, followRename } = await loadRobots()
    addRobot('Hexapod.local')
    recordFirmware('Hexapod.local', report('aaaaaaaaaaaa'))

    expect(followRename('aaaaaaaaaaaa', 'hexapod')).toBeNull()

    expect(get(robots)[0].address).toBe('Hexapod.local')
  })

  it('never rewrites an IP or a non-.local name', async () => {
    const { robots, addRobot, recordFirmware, followRename } = await loadRobots()
    addRobot('192.168.1.50')
    addRobot('robot.example.com')
    addRobot('hexapod.local:8080')
    recordFirmware('192.168.1.50', report('aaaaaaaaaaaa'))
    recordFirmware('robot.example.com', report('bbbbbbbbbbbb'))
    recordFirmware('hexapod.local:8080', report('cccccccccccc'))

    expect(followRename('aaaaaaaaaaaa', 'kitchen')).toBeNull()
    expect(followRename('bbbbbbbbbbbb', 'kitchen')).toBeNull()
    expect(followRename('cccccccccccc', 'kitchen')).toBeNull()

    expect(get(robots).map(r => r.address)).toEqual([
      '192.168.1.50',
      'robot.example.com',
      'hexapod.local:8080'
    ])
  })

  it('takes no host name the firmware would refuse', async () => {
    const { robots, addRobot, recordFirmware, followRename } = await loadRobots()
    addRobot('esp32.local')
    recordFirmware('esp32.local', report('aaaaaaaaaaaa'))

    for (const hostname of [
      'x@evil.com',
      'a/b',
      'a:80',
      'a.b',
      '-a',
      'a-',
      'A',
      'a'.repeat(33),
      ''
    ])
      expect(followRename('aaaaaaaaaaaa', hostname)).toBeNull()

    expect(get(robots)[0].address).toBe('esp32.local')
    expect(followRename('aaaaaaaaaaaa', 'a'.repeat(32))).not.toBeNull()
  })

  it('rewrites the .local entry of a robot saved by its IP as well', async () => {
    const { robots, addRobot, recordFirmware, followRename } = await loadRobots()
    addRobot('192.168.1.50')
    addRobot('esp32.local')
    recordFirmware('192.168.1.50', report('aaaaaaaaaaaa'))
    recordFirmware('esp32.local', report('aaaaaaaaaaaa'))

    expect(followRename('aaaaaaaaaaaa', 'hexapod')).toEqual({
      from: 'esp32.local',
      to: 'hexapod.local'
    })

    expect(get(robots).map(r => r.address)).toEqual(['192.168.1.50', 'hexapod.local'])
  })

  it('leaves a robot without that id alone, even one saved under the old name', async () => {
    const { robots, addRobot, followRename } = await loadRobots()
    addRobot('esp32.local')

    expect(followRename('aaaaaaaaaaaa', 'hexapod')).toBeNull()

    expect(get(robots)[0].address).toBe('esp32.local')
  })

  it('does not move a robot onto the address another saved robot holds', async () => {
    const { robots, addRobot, recordFirmware, followRename } = await loadRobots()
    addRobot('esp32.local')
    addRobot('hexapod.local')
    recordFirmware('esp32.local', report('aaaaaaaaaaaa'))

    expect(followRename('aaaaaaaaaaaa', 'hexapod')).toBeNull()

    expect(get(robots).map(r => r.address)).toEqual(['esp32.local', 'hexapod.local'])
  })
})

describe('recordFirmware', () => {
  it('keeps an existing robotId', async () => {
    const { robots, addRobot, recordFirmware } = await loadRobots()
    addRobot('hexapod.local')
    recordFirmware('hexapod.local', report('aaaaaaaaaaaa'))

    recordFirmware('hexapod.local', report('bbbbbbbbbbbb'))

    expect(get(robots)[0].robotId).toBe('aaaaaaaaaaaa')
  })

  it('learns the robotId a robot reports, and none from firmware that reports none', async () => {
    const { robots, addRobot, recordFirmware } = await loadRobots()
    addRobot('hexapod.local')

    recordFirmware('hexapod.local', report(null))
    expect(get(robots)[0].robotId).toBeUndefined()

    recordFirmware('hexapod.local', report('aaaaaaaaaaaa'))
    expect(get(robots)[0].robotId).toBe('aaaaaaaaaaaa')
  })
})

describe('recordLastIp', () => {
  let stores: Awaited<ReturnType<typeof loadRobots>>

  beforeEach(async () => {
    stores = await loadRobots()
    stores.addRobot('a.local')
    stores.addRobot('b.local')
    stores.recordFirmware('a.local', report('aaaaaaaaaaaa'))
    stores.recordFirmware('b.local', report('bbbbbbbbbbbb'))
  })

  it('records the IP on the robot with that id only', () => {
    stores.recordLastIp('bbbbbbbbbbbb', '192.168.1.50')

    expect(get(stores.robots).map(r => r.lastIp)).toEqual([undefined, '192.168.1.50'])
  })

  it('writes nothing for an id no saved robot has, or an IP it already holds', () => {
    stores.recordLastIp('bbbbbbbbbbbb', '192.168.1.50')
    const writes = vi.fn()
    const stop = stores.robots.subscribe(writes)
    writes.mockClear()

    stores.recordLastIp('cccccccccccc', '192.168.1.60')
    stores.recordLastIp('bbbbbbbbbbbb', '192.168.1.50')

    expect(writes).not.toHaveBeenCalled()
    stop()
  })
})
