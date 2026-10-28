// The stock fake robot serves the six settings topics as the firmware does (firmware/test/test_settings_topics): the
// stored group, redacted, as the snapshot; a push to every subscribed client when an update changes what is stored;
// none for an update that leaves it as it was or one the robot refuses.
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import type { MessageFns } from '$lib/platform_shared/message'
import { DataBroker } from '$lib/transport/databroker'
import { RequestFailed } from '$lib/transport/reply'
import { snapshotOnce } from '$lib/robot/snapshot-once'
import { writeSettings } from '$lib/robot/settings-write'
import {
  APSettings,
  CameraSettings,
  MDNSSettings,
  ServoSettings,
  WifiSettings
} from '$lib/platform_shared/api'
import { fakeLink } from '../fake-robot/transport'
import { stockRobot, type StockRobot } from '../fake-robot/stock'

const settle = () => vi.advanceTimersByTimeAsync(0)

describe('the stock robot', () => {
  let robot: StockRobot
  let brokers: DataBroker[]

  const join = async () => {
    const link = fakeLink(robot)
    const broker = new DataBroker()
    broker.addTransport(link)
    await link.connect()
    brokers.push(broker)
    return broker
  }

  // The frames of a topic a client hears after its snapshot.
  const hear = async <T>(broker: DataBroker, topic: MessageFns<T>) => {
    const frames: T[] = []
    broker.on(topic, frame => frames.push(frame))
    await settle()
    frames.length = 0
    return frames
  }

  beforeEach(() => {
    vi.useFakeTimers()
    robot = stockRobot()
    brokers = []
  })

  afterEach(() => vi.useRealTimers())

  it('snapshots the stored settings without the passwords or the applied secret', async () => {
    const broker = await join()
    robot.hold('wifiSettings', { ...robot.held.wifiSettings, appliedSecret: 7 })

    const wifi = snapshotOnce(WifiSettings, broker)
    const ap = snapshotOnce(APSettings, broker)
    await settle()

    expect((await wifi).appliedSecret).toBe(0)
    expect((await wifi).wifiNetworks.map(n => n.password)).toEqual(['', ''])
    expect((await ap).password).toBe('')
    expect(robot.held.apSettings.password).not.toBe('')
  })

  it('pushes a stored change to every subscribed client, redacted, and answers with the same group', async () => {
    const [writer, reader] = [await join(), await join()]
    const written = await hear(writer, WifiSettings)
    const read = await hear(reader, WifiSettings)

    const reply = await writeSettings(
      WifiSettings,
      { wifiSettingsUpdate: { ...robot.shown('wifiSettings'), hostname: 'hexapod-lab' } },
      'the Wi-Fi settings',
      writer
    )

    expect(written).toHaveLength(1)
    expect(read).toHaveLength(1)
    expect(read[0].hostname).toBe('hexapod-lab')
    expect(read[0].wifiNetworks.map(n => n.password)).toEqual(['', ''])
    expect(reply.wifiSettings).toEqual(read[0])
    expect(robot.held.wifiSettings.wifiNetworks[0].password).toBe('kitchen-table-42')
  })

  it('pushes nothing for an update that leaves the stored settings as they were', async () => {
    const broker = await join()
    const frames = await hear(broker, WifiSettings)

    await writeSettings(
      WifiSettings,
      { wifiSettingsUpdate: robot.shown('wifiSettings') },
      'the Wi-Fi settings',
      broker
    )

    expect(frames).toHaveLength(0)
  })

  it('pushes nothing for an update it refuses, and says why', async () => {
    const broker = await join()
    const servos = await hear(broker, ServoSettings)
    const mdns = await hear(broker, MDNSSettings)

    const badServo = robot.shown('servoSettings').servos.map(s => ({ ...s, conversion: 0 }))
    await expect(
      writeSettings(
        ServoSettings,
        { servoSettingsUpdate: { servos: badServo } },
        'the servo settings',
        broker
      )
    ).rejects.toBeInstanceOf(RequestFailed)
    await expect(
      writeSettings(
        MDNSSettings,
        { mdnsSettingsUpdate: { ...robot.shown('mdnsSettings'), hostname: 'Not A Label' } },
        'the mDNS settings',
        broker
      )
    ).rejects.toThrow('the hostname must be one DNS label: a-z, 0-9 and -')

    expect(servos).toHaveLength(0)
    expect(mdns).toHaveLength(0)
    expect(robot.held.mdnsSettings.hostname).toBe('hexapod')
  })

  // firmware/src/main.cpp asks the fitted sensor first and the range second, and gives each refusal its reason.
  it('refuses a camera value the sensor lacks before one out of range, and says why for either', async () => {
    const broker = await join()
    const stored = robot.held.cameraSettings
    const update = (change: Partial<CameraSettings>) =>
      writeSettings(
        CameraSettings,
        { cameraSettingsUpdate: { ...robot.shown('cameraSettings'), ...change } },
        'the camera settings',
        broker
      )

    await expect(update({ quality: 64 })).rejects.toThrow(
      "a camera setting is out of the sensor's range"
    )
    await expect(update({ sharpness: 4 })).rejects.toThrow(
      'this camera has no sharpness setting; leave it at 0'
    )
    expect(robot.held.cameraSettings).toBe(stored)
  })

  it('refuses an mDNS host name longer than the 32 characters the wire carries', async () => {
    const broker = await join()
    const update = (hostname: string) =>
      writeSettings(
        MDNSSettings,
        { mdnsSettingsUpdate: { ...robot.shown('mdnsSettings'), hostname } },
        'the mDNS settings',
        broker
      )

    await expect(update('a'.repeat(33))).rejects.toThrow('the hostname must be one DNS label')
    await expect(update('a'.repeat(32))).resolves.toBeDefined()
    expect(robot.held.mdnsSettings.hostname).toBe('a'.repeat(32))
  })

  it('starts with the mDNS names of the factory and the 18 servos', async () => {
    const broker = await join()
    const mdns = snapshotOnce(MDNSSettings, broker)
    const servos = snapshotOnce(ServoSettings, broker)
    await settle()

    expect(await mdns).toMatchObject({ hostname: 'hexapod', instance: 'Hexapod' })
    expect((await servos).servos).toHaveLength(18)
  })
})
