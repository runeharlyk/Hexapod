// What becomes of a request while no link is up, by its kind (src/lib/transport/request-kinds.ts): an action fails at
// once, a settings write waits out a link blip and then fails as not saved, and a read waits until its timeout.
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { DataBroker } from '$lib/transport/databroker'
import { SETTING_GRACE_MS } from '$lib/transport/request-kinds'
import { FakeRobot, fakeLink } from '../fake-robot/transport'

const REQUEST_TIMEOUT_MS = 15000

describe('a request while no link is up', () => {
  let robot: FakeRobot
  let link: ReturnType<typeof fakeLink>
  let broker: DataBroker

  beforeEach(() => {
    vi.useFakeTimers()
    robot = new FakeRobot()
    robot.respondOtherwise(() => ({ statusCode: 200 }))
    link = fakeLink(robot)
    broker = new DataBroker()
    broker.addTransport(link)
  })

  afterEach(() => vi.useRealTimers())

  it('fails at once for an action, which is never sent later', async () => {
    const move = broker.request({ goalCancel: {} })
    await expect(move).rejects.toMatchObject({ failure: { kind: 'disconnected' } })

    await link.connect()
    await vi.advanceTimersByTimeAsync(0)
    expect(robot.requests('goalCancel')).toHaveLength(0)
  })

  it('waits out a link blip for a settings write', async () => {
    const write = broker.request({ servoSettingsUpdate: { servos: [] } })
    await vi.advanceTimersByTimeAsync(SETTING_GRACE_MS - 1)
    await link.connect()

    await expect(write).resolves.toMatchObject({ statusCode: 200 })
    expect(robot.requests('servoSettingsUpdate')).toHaveLength(1)
  })

  it('fails a settings write as not sent once the grace has passed, and never sends it', async () => {
    const write = broker.request({ wifiSettingsUpdate: { hostname: 'hexapod' } as never })
    const failed = expect(write).rejects.toMatchObject({ failure: { kind: 'disconnected' } })
    await vi.advanceTimersByTimeAsync(SETTING_GRACE_MS)
    await failed

    await link.connect()
    await vi.advanceTimersByTimeAsync(0)
    expect(robot.requests('wifiSettingsUpdate')).toHaveLength(0)
  })

  it('holds a read for a link until its timeout, counted from the call', async () => {
    const late = broker.request({ featuresDataRequest: {} })
    const timedOut = expect(late).rejects.toMatchObject({ failure: { kind: 'timeout' } })
    await vi.advanceTimersByTimeAsync(REQUEST_TIMEOUT_MS)
    await timedOut

    const read = broker.request({ taskStatsRequest: {} })
    await vi.advanceTimersByTimeAsync(REQUEST_TIMEOUT_MS - 1)
    await link.connect()
    await expect(read).resolves.toMatchObject({ statusCode: 200 })
    expect(robot.requests('featuresDataRequest')).toHaveLength(0)
  })
})
