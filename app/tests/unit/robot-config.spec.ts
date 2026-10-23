import { afterEach, beforeAll, beforeEach, describe, expect, it, vi } from 'vitest'
import { dataBroker } from '$lib/transport/databroker'
import { FakeRobot, fakeLink } from '../fake-robot/transport'
import { PeripheralSettings, ServoSettings } from '$lib/platform_shared/api'
import { applyConfig, exportConfig, parseConfig, serializeConfig } from '$lib/robot-config'

const calibration = (count: number, offset: number) =>
  ServoSettings.create({
    servos: Array.from({ length: count }, (_, i) => ({
      centerPwm: 300 + offset + i,
      direction: i % 2 ? -1 : 1,
      centerAngle: i % 3 === 0 ? 0 : 45.5,
      conversion: 2.1 + offset / 8,
      name: `servo ${i}`
    }))
  })

// The robot stores and echoes servo floats at float32 precision.
const onWire = (settings: ServoSettings) =>
  ServoSettings.decode(ServoSettings.encode(settings).finish())

const i2c = (sda: number, scl: number) =>
  PeripheralSettings.create({
    sda,
    scl,
    frequency: 400000,
    pins: [{ pin: 4, mode: 'output', type: 'digital', role: 'led' }]
  })

// Stands in for the firmware's settings handlers: both groups are state topics, which a subscribe answers with the
// stored settings and a stored change pushes; an update answers the stored settings after the update, which keeps the
// old ones when it is refused.
const stored = { servo: calibration(18, 0), peripherals: i2c(1, 2) }
const robot = {
  get servo() {
    return stored.servo
  },
  set servo(settings: ServoSettings) {
    stored.servo = settings
    fake.set(ServoSettings, settings)
  },
  get peripherals() {
    return stored.peripherals
  },
  set peripherals(settings: PeripheralSettings) {
    stored.peripherals = settings
    fake.set(PeripheralSettings, settings)
  },
  refuseServos: false,
  refusePeripherals: false
}
const updates: string[] = []

const fake = new FakeRobot()
fake.respond('servoSettingsUpdate', request => {
  updates.push('servo')
  if (!robot.refuseServos) robot.servo = request.servoSettingsUpdate!
  return { servoSettings: robot.servo }
})
fake.respond('peripheralSettingsUpdate', request => {
  updates.push('peripherals')
  if (robot.refusePeripherals) return { statusCode: 400, peripheralSettings: robot.peripherals }
  robot.peripherals = request.peripheralSettingsUpdate!
  return { peripheralSettings: robot.peripherals }
})
const transport = fakeLink(fake)

beforeAll(async () => {
  dataBroker.addTransport(transport)
  await transport.connect()
})

beforeEach(() => {
  robot.servo = calibration(18, 0)
  robot.peripherals = i2c(1, 2)
  robot.refuseServos = false
  robot.refusePeripherals = false
  updates.length = 0
})

const savedFrom = async (servo: ServoSettings, peripherals: PeripheralSettings) => {
  robot.servo = servo
  robot.peripherals = peripherals
  const text = serializeConfig(await exportConfig(new Date('2026-10-13T12:00:00Z')))
  robot.servo = calibration(18, 0)
  robot.peripherals = i2c(1, 2)
  return text
}

describe('robot configuration file', () => {
  it('restores the saved calibration and peripheral settings onto the robot', async () => {
    const text = await savedFrom(calibration(18, 7), i2c(47, 21))

    const config = parseConfig(text)
    expect(config.exportedAt).toBe('2026-10-13T12:00:00.000Z')
    await applyConfig(config)

    expect(updates).toEqual(['servo', 'peripherals'])
    expect(robot.servo).toEqual(onWire(calibration(18, 7)))
    expect(robot.peripherals).toEqual(i2c(47, 21))
  })

  it('keeps the compass offset the robot holds when the file has none, and restores one it has', async () => {
    const offset = { magOffsetX: 60, magOffsetY: -85, magOffsetZ: 30 }
    const older = parseConfig(await savedFrom(calibration(18, 7), i2c(47, 21)))
    robot.peripherals = PeripheralSettings.create({ ...i2c(1, 2), ...offset })

    await applyConfig(older)

    expect(robot.peripherals).toEqual(PeripheralSettings.create({ ...i2c(47, 21), ...offset }))

    const fitted = { magOffsetX: -3, magOffsetY: 4.5, magOffsetZ: 0 }
    const withOffset = await savedFrom(
      calibration(18, 7),
      PeripheralSettings.create({ ...i2c(47, 21), ...fitted })
    )
    robot.peripherals = PeripheralSettings.create({ ...i2c(1, 2), ...offset })

    await applyConfig(parseConfig(withOffset))

    expect(robot.peripherals).toEqual(PeripheralSettings.create({ ...i2c(47, 21), ...fitted }))
  })

  it('accepts a hand-written calibration that the robot rounds to float32', async () => {
    const text = serializeConfig({
      format: 'hexapod-robot-config',
      version: 1,
      exportedAt: '2026-10-13T12:00:00.000Z',
      servo: calibration(18, 7),
      peripherals: i2c(47, 21)
    })
    expect(text).toContain('"conversion": 2.975')

    await applyConfig(parseConfig(text))

    expect(updates).toEqual(['servo', 'peripherals'])
    expect(robot.servo).toEqual(onWire(calibration(18, 7)))
  })

  it('refuses a file with a section missing or malformed', async () => {
    const file = JSON.parse(await savedFrom(calibration(18, 7), i2c(47, 21)))
    const without = (key: string, value?: unknown) => JSON.stringify({ ...file, [key]: value })

    expect(() => parseConfig(without('peripherals'))).toThrow('not a hexapod configuration')
    expect(() => parseConfig(without('peripherals', 'pins'))).toThrow('not a hexapod configuration')
    expect(() => parseConfig(without('servo'))).toThrow('not a hexapod configuration')
    expect(() => parseConfig(without('servo', { servos: 'none' }))).toThrow(
      'not a hexapod configuration'
    )
  })

  it('refuses I2C settings the robot would reject, before writing the calibration', async () => {
    const sharedPin = await savedFrom(calibration(18, 7), i2c(21, 21))
    await expect(applyConfig(parseConfig(sharedPin))).rejects.toThrow(
      'The I2C settings in this file are invalid'
    )
    const slowBus = await savedFrom(
      calibration(18, 7),
      PeripheralSettings.create({ sda: 47, scl: 21, frequency: 50000 })
    )
    await expect(applyConfig(parseConfig(slowBus))).rejects.toThrow(
      'The I2C settings in this file are invalid'
    )
    expect(updates).toEqual([])
  })

  it('refuses a file that is not a hexapod configuration', () => {
    expect(() => parseConfig('not json')).toThrow('not a hexapod configuration')
    expect(() => parseConfig('null')).toThrow('not a hexapod configuration')
    expect(() => parseConfig(JSON.stringify({ format: 'leika-robot-config', version: 1 }))).toThrow(
      'not a hexapod configuration'
    )
  })

  it('refuses a configuration version this app does not read', async () => {
    const text = await savedFrom(calibration(18, 7), i2c(47, 21))
    expect(() => parseConfig(text.replace('"version": 1', '"version": 2'))).toThrow(
      'configuration version 2; this app reads version 1'
    )
  })

  it('refuses a file whose servo count differs from the robot, writing nothing', async () => {
    const text = await savedFrom(calibration(12, 7), i2c(47, 21))

    await expect(applyConfig(parseConfig(text))).rejects.toThrow(
      'This file holds 12 servos; the robot has 18'
    )
    expect(updates).toEqual([])
    expect(robot.servo).toEqual(calibration(18, 0))
  })

  it('reports a calibration the robot kept out, and stops before the peripherals', async () => {
    const text = await savedFrom(calibration(18, 7), i2c(47, 21))
    robot.refuseServos = true

    await expect(applyConfig(parseConfig(text))).rejects.toThrow(
      'The robot rejected the servo calibration'
    )
    expect(updates).toEqual(['servo'])
  })

  it('reports peripheral settings the robot rejected', async () => {
    const text = await savedFrom(calibration(18, 7), i2c(47, 21))
    robot.refusePeripherals = true

    await expect(applyConfig(parseConfig(text))).rejects.toThrow(
      'The robot rejected the peripheral settings; the servo calibration was already applied'
    )
    expect(robot.peripherals).toEqual(i2c(1, 2))
  })
})

describe('robot configuration file with no link', () => {
  afterEach(() => vi.useRealTimers())

  it('fails at once as not connected, before the feature flags are in', async () => {
    vi.useFakeTimers()
    vi.resetModules()
    const offline = await import('$lib/robot-config')
    let outcome: unknown = 'unsettled'
    offline.exportConfig().then(
      () => (outcome = 'exported'),
      (error: Error) => (outcome = error.message)
    )

    await vi.advanceTimersByTimeAsync(0)

    expect(outcome).toBe('The robot did not send its servo calibration: the robot is not connected')
  })
})
