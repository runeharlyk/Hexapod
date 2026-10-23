// writeSettings counts a settings write into its topic's `pending` from the send to the reply, however the reply ends,
// over the real DataBroker and the stock fake robot, which stores the update and pushes the group as the firmware does.
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import type { Readable } from 'svelte/store'
import { DataBroker } from '$lib/transport/databroker'
import type { Watched } from '$lib/transport/topics'
import { watch } from '$lib/robot/watch'
import { writeSettings } from '$lib/robot/settings-write'
import { WifiSettings } from '$lib/platform_shared/api'
import { fakeLink } from '../fake-robot/transport'
import { stockRobot, type StockRobot } from '../fake-robot/stock'

const settle = () => vi.advanceTimersByTimeAsync(0)

const record = <T>(store: Readable<Watched<T>>) => {
  const seen: Watched<T>[] = []
  const stop = store.subscribe(w => seen.push(w))
  return { seen, stop, now: () => seen[seen.length - 1] }
}

// A reply the test releases when it chooses.
const gate = () => {
  let open!: () => void
  const opened = new Promise<void>(resolve => (open = resolve))
  return { open, opened }
}

describe('writeSettings', () => {
  let robot: StockRobot
  let link: ReturnType<typeof fakeLink>
  let broker: DataBroker

  const write = (hostname: string) =>
    writeSettings(
      WifiSettings,
      { wifiSettingsUpdate: { ...robot.shown('wifiSettings'), hostname } },
      'the Wi-Fi settings',
      broker
    )

  beforeEach(async () => {
    vi.useFakeTimers()
    robot = stockRobot()
    link = fakeLink(robot)
    broker = new DataBroker()
    broker.addTransport(link)
    await link.connect()
  })

  afterEach(() => vi.useRealTimers())

  it('is pending from the send to the reply, and the value comes from the push', async () => {
    const reply = gate()
    robot.respond('wifiSettingsUpdate', async request => {
      await reply.opened
      return robot.store('wifiSettingsUpdate', request)
    })
    const wifi = record(watch(WifiSettings, broker))
    await settle()
    expect(wifi.now()).toMatchObject({ status: 'live', pending: false })

    const written = write('hexapod-lab')
    await settle()

    expect(robot.requests('wifiSettingsUpdate')).toHaveLength(1)
    expect(wifi.now()).toMatchObject({ pending: true, value: { hostname: 'hexapod' } })
    reply.open()
    await written
    await settle()
    expect(wifi.now()).toMatchObject({ pending: false, value: { hostname: 'hexapod-lab' } })
    wifi.stop()
  })

  it('stays pending until every overlapping write has settled, and shows only what the robot pushed', async () => {
    const firstReply = gate()
    robot.respond('wifiSettingsUpdate', async request => {
      if (request.wifiSettingsUpdate?.hostname !== 'first')
        return robot.store('wifiSettingsUpdate', request)
      await firstReply.opened
      return { statusCode: 400, wifiSettings: robot.shown('wifiSettings') }
    })
    const wifi = record(watch(WifiSettings, broker))
    await settle()

    const first = write('first').then(
      () => 'answered',
      (error: Error) => error.name
    )
    await write('second')
    await settle()

    expect(wifi.now()).toMatchObject({ pending: true, value: { hostname: 'second' } })
    firstReply.open()
    await expect(first).resolves.toBe('RequestFailed')
    await settle()

    expect(wifi.now()).toMatchObject({ pending: false, value: { hostname: 'second' } })
    expect(new Set(wifi.seen.map(w => w.value?.hostname))).toEqual(
      new Set([undefined, 'hexapod', 'second'])
    )
    wifi.stop()
  })

  it('ends the write when the link drops', async () => {
    robot.respond('wifiSettingsUpdate', () => new Promise(() => {}))
    const wifi = record(watch(WifiSettings, broker))
    await settle()

    const failure = expect(write('hexapod-lab')).rejects.toMatchObject({
      failure: { kind: 'disconnected' }
    })
    await settle()
    expect(wifi.now().pending).toBe(true)

    await link.disconnect()
    await failure

    expect(wifi.now()).toMatchObject({ pending: false, stale: true })
    wifi.stop()
  })

  it('counts a write begun before anything watches the topic', async () => {
    const reply = gate()
    robot.respond('wifiSettingsUpdate', async request => {
      await reply.opened
      return robot.store('wifiSettingsUpdate', request)
    })

    const written = write('hexapod-lab')
    await settle()
    const wifi = record(watch(WifiSettings, broker))
    await settle()

    expect(wifi.now()).toMatchObject({ pending: true })
    reply.open()
    await written
    await settle()
    expect(wifi.now().pending).toBe(false)
    wifi.stop()
  })
})
