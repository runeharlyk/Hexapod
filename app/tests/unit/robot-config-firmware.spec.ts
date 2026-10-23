import { beforeAll, describe, expect, it } from 'vitest'
import { dataBroker } from '$lib/transport/databroker'
import { tagOf } from '$lib/transport/message-codec'
import { FeaturesDataResponse } from '$lib/platform_shared/message'
import { PeripheralSettings, ServoSettings } from '$lib/platform_shared/api'
import { applyConfig, exportConfig } from '$lib/robot-config'
import { FakeRobot, fakeLink } from '../fake-robot/transport'

// Firmware from before the settings topics acks a subscribe to one it does not know without a value, so the export
// would wait out the subscribe's retries; the feature flags say at once that it cannot answer.
const fake = new FakeRobot()
fake.respond('featuresDataRequest', () => ({
  featuresDataResponse: FeaturesDataResponse.create({
    firmwareName: 'hexapod',
    settingsTopics: false
  })
}))
const transport = fakeLink(fake)

beforeAll(async () => {
  dataBroker.addTransport(transport)
  await transport.connect()
})

const TOO_OLD =
  "The robot did not send its servo calibration: the robot's firmware does not support the servo calibration; update it"

describe('the configuration file against firmware without settings topics', () => {
  it('names the firmware as too old when saving, without subscribing', async () => {
    await expect(exportConfig()).rejects.toThrow(TOO_OLD)
    expect(fake.subscriptions()).not.toContain(tagOf(ServoSettings))
    expect(fake.subscriptions()).not.toContain(tagOf(PeripheralSettings))
  })

  it('names the firmware as too old when loading, and writes nothing', async () => {
    const file = {
      format: 'hexapod-robot-config' as const,
      version: 1,
      exportedAt: '2026-10-13T12:00:00.000Z',
      servo: ServoSettings.create({ servos: [] }),
      peripherals: PeripheralSettings.create({ sda: 8, scl: 9, frequency: 400000 })
    }
    await expect(applyConfig(file)).rejects.toThrow(TOO_OLD)
    expect(fake.requests('servoSettingsUpdate')).toEqual([])
  })
})
