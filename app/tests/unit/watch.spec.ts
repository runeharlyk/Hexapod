// watch() over the real DataBroker and the fake robot (tests/fake-robot), which answers a subscribe as the firmware
// does: a state topic's value, then the ack. The watch shows the value and whether it is live, stale or failed; the
// link's drops, fail-overs and the robot's lost frames are what it has to survive.
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import type { Readable } from 'svelte/store'
import { DataBroker } from '$lib/transport/databroker'
import { tagOf } from '$lib/transport/message-codec'
import { ACK_TIMEOUT_MS, type Watched } from '$lib/transport/topics'
import { watch } from '$lib/robot/watch'
import { describeFailure } from '$lib/transport/reply'
import { GoalStatus, ModeData, ModesEnum } from '$lib/platform_shared/message'
import { FakeRobot, fakeLink } from '../fake-robot/transport'

const MODE = tagOf(ModeData)
const STAND = ModesEnum.STAND
const WALK = ModesEnum.WALK

// Lets the fake robot's frames, which arrive on later microtasks, land.
const settle = () => vi.advanceTimersByTimeAsync(0)

const record = <T>(store: Readable<Watched<T>>) => {
  const seen: Watched<T>[] = []
  const stop = store.subscribe(w => seen.push(w))
  return { seen, stop, now: () => seen[seen.length - 1] }
}

describe('watch', () => {
  let robot: FakeRobot
  let link: ReturnType<typeof fakeLink>
  let broker: DataBroker

  beforeEach(async () => {
    vi.useFakeTimers()
    robot = new FakeRobot()
    link = fakeLink(robot)
    broker = new DataBroker()
    broker.addTransport(link)
    await link.connect()
  })

  afterEach(() => vi.useRealTimers())

  it('subscribes once for every watch of a tag, and a second watch starts live with the value', async () => {
    robot.set(ModeData, { mode: STAND, handheld: false })
    const first = record(watch(ModeData, broker))
    await settle()

    const second = record(watch(ModeData, broker))

    expect(robot.subscriptions()).toEqual([MODE])
    expect(second.seen).toHaveLength(1)
    expect(second.now()).toMatchObject({ value: { mode: STAND }, status: 'live', stale: false })
    first.stop()
    second.stop()
  })

  it('shows the snapshot as it arrives and goes live on the ack', async () => {
    robot.set(ModeData, { mode: WALK, handheld: true })
    const mode = record(watch(ModeData, broker))
    await settle()

    expect(mode.seen.map(w => [w.value?.mode, w.status])).toEqual([
      [undefined, 'subscribing'],
      [WALK, 'subscribing'],
      [WALK, 'live']
    ])
    mode.stop()
  })

  it('follows a push after the ack', async () => {
    const mode = record(watch(ModeData, broker))
    await settle()

    robot.set(ModeData, { mode: STAND, handheld: false })
    await settle()

    expect(mode.now()).toMatchObject({ value: { mode: STAND }, status: 'live' })
    mode.stop()
  })

  // An ack with no value before it means the snapshot was lost on the way.
  it('resubscribes when the snapshot was lost, and fails after three acks without one', async () => {
    robot.dropNext(MODE, 1)
    const mode = record(watch(ModeData, broker))
    await settle()
    expect(robot.subscriptions()).toEqual([MODE, MODE])
    expect(mode.now()).toMatchObject({ status: 'live', value: { mode: 0 } })
    mode.stop()

    robot.dropNext(tagOf(GoalStatus), 3)
    const goal = record(watch(GoalStatus, broker))
    await settle()
    expect(robot.subscriptions().filter(tag => tag === tagOf(GoalStatus))).toHaveLength(3)
    expect(goal.now()).toMatchObject({ status: 'error', value: undefined })
    goal.stop()
  })

  it('resubscribes when no ack comes in time and gives up after three tries', async () => {
    robot.stall()
    const mode = record(watch(ModeData, broker))
    await settle()

    await vi.advanceTimersByTimeAsync(ACK_TIMEOUT_MS)
    expect(robot.subscriptions()).toEqual([MODE, MODE])
    await vi.advanceTimersByTimeAsync(2 * ACK_TIMEOUT_MS)

    expect(robot.subscriptions()).toEqual([MODE, MODE, MODE])
    expect(mode.now()).toMatchObject({ status: 'error', failure: { kind: 'timeout' } })
    mode.stop()
  })

  // A firmware from before the snapshots answers pings and requests but never acks a subscribe. Every watch would end
  // in an error; it says the firmware is too old, as a request it does not know does.
  it('names a firmware that never acks as too old, and tries again on the next link', async () => {
    robot.ackless()
    const mode = record(watch(ModeData, broker))
    await settle()

    await vi.advanceTimersByTimeAsync(3 * ACK_TIMEOUT_MS)

    expect(mode.now()).toMatchObject({ status: 'error', failure: { kind: 'too-old' } })
    expect(describeFailure(mode.now().failure!, 'the live mode')).toBe(
      "the robot's firmware does not support the live mode; update it"
    )

    await link.disconnect()
    await link.connect()
    await settle()
    expect(mode.now().status).toBe('subscribing')
    expect(robot.subscriptions()).toHaveLength(4)
    mode.stop()
  })

  it('keeps the value as stale while the link is down, and the next snapshot makes it current', async () => {
    robot.set(ModeData, { mode: STAND, handheld: false })
    const mode = record(watch(ModeData, broker))
    await settle()

    await link.disconnect()
    expect(mode.now()).toMatchObject({ value: { mode: STAND }, status: 'subscribing', stale: true })

    robot.set(ModeData, { mode: WALK, handheld: false })
    await link.connect()
    await settle()

    expect(mode.now()).toMatchObject({ value: { mode: WALK }, status: 'live', stale: false })
    expect(robot.subscriptions()).toEqual([MODE, MODE])
    mode.stop()
  })

  // Serial unplugged while the WebSocket stays up: the watch moves to the WebSocket and its snapshot.
  it('goes stale on a fail-over and is made current by the other link', async () => {
    const other = new FakeRobot()
    other.set(ModeData, { mode: WALK, handheld: false })
    const backup = fakeLink(other)
    broker.addTransport(backup)
    await backup.connect()
    robot.set(ModeData, { mode: STAND, handheld: false })
    const mode = record(watch(ModeData, broker))
    await settle()
    expect(mode.now()).toMatchObject({ value: { mode: STAND }, status: 'live' })

    // The broker moves at once; the other robot's snapshot lands on a later microtask.
    const dropping = link.disconnect()
    expect(mode.now()).toMatchObject({ value: { mode: STAND }, stale: true, status: 'subscribing' })
    await dropping
    await settle()

    expect(mode.now()).toMatchObject({ value: { mode: WALK }, stale: false, status: 'live' })
    mode.stop()
  })

  it('forgets the value on reset and asks the robot again', async () => {
    robot.set(ModeData, { mode: STAND, handheld: false })
    const mode = record(watch(ModeData, broker))
    await settle()
    robot.set(ModeData, { mode: WALK, handheld: false })

    broker.resetStates()
    expect(mode.now()).toMatchObject({ value: undefined, stale: false, status: 'subscribing' })
    await settle()

    expect(mode.now()).toMatchObject({ value: { mode: WALK }, status: 'live' })
    mode.stop()
  })

  it('unsubscribes once the last watch stops, and a new watch starts afresh', async () => {
    const first = record(watch(ModeData, broker))
    const second = record(watch(ModeData, broker))
    await settle()

    first.stop()
    second.stop()

    expect(robot.received.filter(m => m.unsubNotif).map(m => m.unsubNotif!.tag)).toEqual([MODE])
    robot.set(ModeData, { mode: WALK, handheld: false })
    await settle()
    expect(second.seen.some(w => w.value?.mode === WALK)).toBe(false)

    const again = record(watch(ModeData, broker))
    expect(again.now()).toMatchObject({ value: undefined, status: 'subscribing' })
    await settle()
    expect(again.now()).toMatchObject({ value: { mode: WALK }, status: 'live' })
    again.stop()
  })
})
