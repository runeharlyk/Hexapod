// The stores the layout and the pages read the robot's state from (src/lib/robot/state.ts), over the real DataBroker and
// the fake robot, which answers a subscribe with the topic's value and then the ack, as the firmware does.
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { get, type Readable } from 'svelte/store'
import { DataBroker } from '$lib/transport/databroker'
import { ACK_TIMEOUT_MS } from '$lib/transport/topics'
import { reportFailure, robotState } from '$lib/robot/state'
import { GOAL_END_HOLD_MS } from '$lib/stores/goal'
import {
  AnimationState,
  AnimationStatus,
  GaitData,
  GaitEnum,
  GoalKind,
  GoalState,
  GoalStatus,
  ModeData,
  ModesEnum,
  OtaState,
  OtaStatusData
} from '$lib/platform_shared/message'
import { MotionModes } from '$lib/motion'
import { GaitType } from '$lib/gait'
import { FakeRobot, fakeLink } from '../fake-robot/transport'

const settle = () => vi.advanceTimersByTimeAsync(0)

const goal = (over: Partial<GoalStatus>): GoalStatus =>
  GoalStatus.create({
    id: 1,
    state: GoalState.GOAL_RUNNING,
    kind: GoalKind.GOAL_FORWARD,
    target: 1,
    progress: 0.5,
    ...over
  })

describe('the robot state', () => {
  let robot: FakeRobot
  let link: ReturnType<typeof fakeLink>
  let state: ReturnType<typeof robotState>
  const kept: (() => void)[] = []
  // As the layout does, so each watch stays subscribed between reads.
  const follow = (...stores: Readable<unknown>[]) =>
    stores.forEach(store => kept.push(store.subscribe(() => {})))

  beforeEach(async () => {
    vi.useFakeTimers()
    robot = new FakeRobot()
    link = fakeLink(robot)
    const broker = new DataBroker()
    broker.addTransport(link)
    state = robotState(broker)
  })

  afterEach(() => {
    kept.splice(0).forEach(stop => stop())
    vi.useRealTimers()
  })

  it('shows the mode and whether the handheld drives from the snapshot and every push', async () => {
    robot.set(ModeData, { mode: ModesEnum.WALK, handheld: true })
    follow(state.mode, state.handheldDriving)
    await link.connect()
    await settle()
    expect([get(state.mode), get(state.handheldDriving)]).toEqual([MotionModes.WALK, true])

    robot.set(ModeData, { mode: ModesEnum.STAND, handheld: false })
    await settle()
    expect([get(state.mode), get(state.handheldDriving)]).toEqual([MotionModes.STAND, false])
  })

  it("shows a mode this app does not know, as an older firmware's POSE, as unknown and not as DEACTIVATED", async () => {
    robot.set(ModeData, { mode: 2 as ModesEnum, handheld: false })
    follow(state.mode)
    await link.connect()
    await settle()
    expect(get(state.mode)).toBeNull()

    robot.set(ModeData, { mode: ModesEnum.STAND, handheld: false })
    await settle()
    expect(get(state.mode)).toBe(MotionModes.STAND)
  })

  it('reads a mode frame from a firmware without the flag as no handheld', async () => {
    robot.set(ModeData, ModeData.decode(Uint8Array.of(0x08, ModesEnum.STAND)))
    follow(state.mode, state.handheldDriving)
    await link.connect()
    await settle()
    expect([get(state.mode), get(state.handheldDriving)]).toEqual([MotionModes.STAND, false])
  })

  it('keeps the mode but not the handheld while the link is down', async () => {
    robot.set(ModeData, { mode: ModesEnum.WALK, handheld: true })
    follow(state.mode, state.handheldDriving)
    await link.connect()
    await settle()

    await link.disconnect()

    expect([get(state.mode), get(state.handheldDriving)]).toEqual([MotionModes.WALK, false])
  })

  it('shows the gait', async () => {
    robot.set(GaitData, { gait: GaitEnum.WAVE })
    follow(state.gait)
    await link.connect()
    await settle()
    expect(get(state.gait)).toBe(GaitType.WAVE)
  })

  it('shows a running goal, and an ended one for a while after it ended', async () => {
    robot.set(GoalStatus, goal({ progress: 0.3 }))
    follow(state.goalStatus)
    await link.connect()
    await settle()
    expect(get(state.goalStatus)?.progress).toBeCloseTo(0.3, 6)

    robot.set(GoalStatus, goal({ state: GoalState.GOAL_REACHED, progress: 1 }))
    await settle()
    expect(get(state.goalStatus)?.state).toBe(GoalState.GOAL_REACHED)
    await vi.advanceTimersByTimeAsync(GOAL_END_HOLD_MS)
    expect(get(state.goalStatus)).toBeNull()
  })

  // A client that connects after a goal ended gets that end in the snapshot: history, not news to show.
  it('does not show a goal that had ended before the link came up', async () => {
    robot.set(GoalStatus, goal({ state: GoalState.GOAL_REACHED, progress: 1 }))
    follow(state.goalStatus)
    await link.connect()
    await settle()
    expect(get(state.goalStatus)).toBeNull()
  })

  it('shows no goal for an idle robot, and none while the link is down', async () => {
    follow(state.goalStatus)
    await link.connect()
    await settle()
    expect(get(state.goalStatus)).toBeNull()

    robot.set(GoalStatus, goal({ id: 2 }))
    await settle()
    expect(get(state.goalStatus)?.id).toBe(2)
    await link.disconnect()
    expect(get(state.goalStatus)).toBeNull()
  })

  it('shows the animation status, and none while the link is down', async () => {
    robot.set(
      AnimationStatus,
      AnimationStatus.create({ name: 'wave', state: AnimationState.ANIM_PLAYING })
    )
    follow(state.animationStatus)
    await link.connect()
    await settle()
    expect(get(state.animationStatus)?.name).toBe('wave')

    await link.disconnect()
    expect(get(state.animationStatus)).toBeNull()
  })

  // The robot keeps a failed update's error as its OTA status until it restarts. In the snapshot a page's subscribe is
  // answered with, that error is an earlier update's: shown, it would greet the next update as failed.
  it("shows an update's end only when it ends while followed", async () => {
    robot.set(OtaStatusData, {
      state: OtaState.OTA_ERROR,
      progress: 40,
      error: 'checksum mismatch'
    })
    follow(state.otaStatus)
    await link.connect()
    await settle()
    expect(get(state.otaStatus)).toBeNull()

    robot.set(OtaStatusData, { state: OtaState.OTA_PROGRESS, progress: 10, error: '' })
    await settle()
    expect(get(state.otaStatus)).toMatchObject({ state: OtaState.OTA_PROGRESS, progress: 10 })
    robot.set(OtaStatusData, { state: OtaState.OTA_ERROR, progress: 20, error: 'write failed' })
    await settle()
    expect(get(state.otaStatus)).toMatchObject({ state: OtaState.OTA_ERROR, error: 'write failed' })
  })

  // A robot that downloads an update itself ends it whether or not the app is linked. The end of an update the app saw
  // under way arrives as the reconnect's snapshot, and the update dialog, which refuses to close while an update runs,
  // would otherwise stay at the progress last seen.
  it('shows the end of an update seen under way that ended while the link was down', async () => {
    follow(state.otaStatus)
    await link.connect()
    robot.set(OtaStatusData, { state: OtaState.OTA_PROGRESS, progress: 30, error: '' })
    await settle()
    expect(get(state.otaStatus)).toMatchObject({ state: OtaState.OTA_PROGRESS, progress: 30 })

    await link.disconnect()
    robot.set(OtaStatusData, { state: OtaState.OTA_ERROR, progress: 30, error: 'download failed' })
    await link.connect()
    await settle()

    expect(get(state.otaStatus)).toMatchObject({
      state: OtaState.OTA_ERROR,
      error: 'download failed'
    })
  })

  it('shows an update under way from its snapshot', async () => {
    robot.set(OtaStatusData, { state: OtaState.OTA_PROGRESS, progress: 60, error: '' })
    follow(state.otaStatus)
    await link.connect()
    await settle()
    expect(get(state.otaStatus)).toMatchObject({ state: OtaState.OTA_PROGRESS, progress: 60 })
  })

  // Firmware from before the snapshots never acks: the failure is said, once, in the words a request it lacks gets.
  it('reports a firmware that never acks as too old, once', async () => {
    robot.ackless()
    const reported: string[] = []
    kept.push(reportFailure(state.modeState, 'live state', message => reported.push(message)))
    await link.connect()
    await settle()

    await vi.advanceTimersByTimeAsync(4 * ACK_TIMEOUT_MS)

    expect(reported).toEqual(["the robot's firmware does not support live state; update it"])
  })
})
