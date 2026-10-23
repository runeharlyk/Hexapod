// snapshotOnce reads a state topic once: it follows until the robot's snapshot is in, then lets go.
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { DataBroker } from '$lib/transport/databroker'
import { tagOf } from '$lib/transport/message-codec'
import { SUBSCRIBE_TRIES } from '$lib/transport/topics'
import { RequestFailed } from '$lib/transport/reply'
import { snapshotOnce } from '$lib/robot/snapshot-once'
import { PeripheralSettings, WifiSettings } from '$lib/platform_shared/api'
import { fakeLink } from '../fake-robot/transport'
import { stockRobot, type StockRobot } from '../fake-robot/stock'

const WIFI = tagOf(WifiSettings)

describe('snapshotOnce', () => {
  let robot: StockRobot
  let broker: DataBroker
  let link: ReturnType<typeof fakeLink>

  beforeEach(async () => {
    vi.useFakeTimers()
    robot = stockRobot()
    link = fakeLink(robot)
    broker = new DataBroker()
    broker.addTransport(link)
    await link.connect()
  })

  afterEach(() => vi.useRealTimers())

  const unsubscribes = () => robot.received.flatMap(m => (m.unsubNotif ? [m.unsubNotif.tag] : []))

  it('resolves with the snapshot, passwords left out, and unsubscribes', async () => {
    const snapshot = snapshotOnce(WifiSettings, broker)
    await vi.advanceTimersByTimeAsync(0)

    const wifi = await snapshot

    expect(wifi.hostname).toBe('hexapod')
    expect(wifi.wifiNetworks.map(n => [n.ssid, n.password])).toEqual([
      ['home-network', ''],
      ['Workshop-Extender-5GHz-Upstairs1', '']
    ])
    expect(robot.subscriptions()).toEqual([WIFI])
    expect(unsubscribes()).toEqual([WIFI])
  })

  it('resolves at once for a topic another watcher follows, and leaves that watcher subscribed', async () => {
    const stop = broker.follow(PeripheralSettings, () => {})
    await vi.advanceTimersByTimeAsync(0)

    await expect(snapshotOnce(PeripheralSettings, broker)).resolves.toMatchObject({
      sda: 8,
      scl: 9
    })

    expect(unsubscribes()).toEqual([])
    stop()
  })

  it('rejects when the robot never sends a value', async () => {
    robot.dropNext(WIFI, SUBSCRIBE_TRIES)
    const outcome = snapshotOnce(WifiSettings, broker).then(
      () => null,
      (error: unknown) => error
    )
    await vi.advanceTimersByTimeAsync(0)

    const error = await outcome

    expect(error).toBeInstanceOf(RequestFailed)
    expect((error as RequestFailed).failure).toEqual({ kind: 'timeout' })
    expect(robot.subscriptions()).toEqual(Array(SUBSCRIBE_TRIES).fill(WIFI))
    expect(unsubscribes()).toEqual([WIFI])
  })

  const failureOf = (read: Promise<unknown>) =>
    read.then(
      () => null,
      (error: unknown) => (error instanceof RequestFailed ? error.failure : error)
    )

  it('rejects at once as disconnected while no link is up', async () => {
    await link.disconnect()

    const failure = failureOf(snapshotOnce(WifiSettings, broker))
    await vi.advanceTimersByTimeAsync(0)

    expect(await failure).toEqual({ kind: 'disconnected' })
    expect(robot.subscriptions()).toEqual([])
  })

  it('does not take the value a dropped link left stale', async () => {
    const stop = broker.follow(PeripheralSettings, () => {})
    await vi.advanceTimersByTimeAsync(0)
    await link.disconnect()

    const failure = failureOf(snapshotOnce(PeripheralSettings, broker))
    await vi.advanceTimersByTimeAsync(0)

    expect(await failure).toEqual({ kind: 'disconnected' })
    stop()
  })

  it('times out as a read does when the link drops before the snapshot and no link comes back', async () => {
    robot.ackless()
    let failure: unknown = 'unsettled'
    void failureOf(snapshotOnce(WifiSettings, broker)).then(f => (failure = f))
    await vi.advanceTimersByTimeAsync(0)
    await link.disconnect()

    await vi.advanceTimersByTimeAsync(14999)
    expect(failure).toBe('unsettled')
    await vi.advanceTimersByTimeAsync(1)

    expect(failure).toEqual({ kind: 'timeout' })
  })
})
