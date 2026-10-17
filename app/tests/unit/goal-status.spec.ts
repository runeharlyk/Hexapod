import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { get, writable } from 'svelte/store'
import { GoalKind, GoalState, type GoalStatus } from '../../src/lib/platform_shared/message'
import {
  GOAL_END_HOLD_MS,
  clearGoalStatus,
  followGoalStatus,
  goalLine,
  goalStatus,
  isGoalActive,
  setGoalStatus,
  type GoalStatusLink
} from '../../src/lib/stores/goal'

const status = (over: Partial<GoalStatus>): GoalStatus => ({
  id: 1,
  state: GoalState.GOAL_RUNNING,
  kind: GoalKind.GOAL_FORWARD,
  target: 1,
  progress: 0,
  x: 0,
  y: 0,
  yaw: 0,
  passes: 1,
  ...over
})

describe('goalLine', () => {
  it('shows a forward walk as covered over asked metres', () => {
    expect(goalLine(status({ target: 1, progress: 0.62 }))).toBe('walking 0.62 / 1.00 m')
  })

  it('shows a backward walk by its length', () => {
    expect(goalLine(status({ target: -0.5, progress: 0.4 }))).toBe('walking 0.20 / 0.50 m')
  })

  it('names a sideways walk apart from a forward one', () => {
    expect(goalLine(status({ kind: GoalKind.GOAL_SIDEWAYS, target: 0.3, progress: 0.5 }))).toBe(
      'sidestepping 0.15 / 0.30 m'
    )
  })

  it('shows a turn in degrees whichever way it turns', () => {
    const quarter = Math.PI / 2
    expect(goalLine(status({ kind: GoalKind.GOAL_TURN, target: quarter, progress: 0.5 }))).toBe(
      'turning 45 / 90 deg'
    )
    expect(goalLine(status({ kind: GoalKind.GOAL_TURN, target: -quarter, progress: 0.5 }))).toBe(
      'turning 45 / 90 deg'
    )
  })

  it('shows an arc in degrees of its turn', () => {
    const circle = 2 * Math.PI
    expect(goalLine(status({ kind: GoalKind.GOAL_ARC, target: circle, progress: 1 / 3 }))).toBe(
      'arc 120 / 360 deg'
    )
  })

  it('clamps progress into the target', () => {
    expect(goalLine(status({ target: 1, progress: 1.2 }))).toBe('walking 1.00 / 1.00 m')
    expect(goalLine(status({ target: 1, progress: -0.1 }))).toBe('walking 0.00 / 1.00 m')
  })

  it('names each ending state with the move', () => {
    const ended = (state: GoalState, kind: GoalKind) => goalLine(status({ state, kind }))
    expect(ended(GoalState.GOAL_REACHED, GoalKind.GOAL_FORWARD)).toBe('walk reached')
    expect(ended(GoalState.GOAL_CANCELLED, GoalKind.GOAL_TURN)).toBe('turn cancelled')
    expect(ended(GoalState.GOAL_TIMED_OUT, GoalKind.GOAL_ARC)).toBe('arc timed out')
    expect(ended(GoalState.GOAL_REFUSED, GoalKind.GOAL_SIDEWAYS)).toBe('sidestep refused')
  })

  it('says a goal is settling while it takes its posture or comes to rest', () => {
    expect(goalLine(status({ state: GoalState.GOAL_SETTLING, kind: GoalKind.GOAL_TURN }))).toBe(
      'turn settling'
    )
  })

  it('has no line for an idle robot or an unknown state', () => {
    expect(goalLine(status({ state: GoalState.GOAL_IDLE }))).toBeNull()
    expect(goalLine(status({ state: GoalState.UNRECOGNIZED }))).toBeNull()
  })
})

describe('goalStatus store', () => {
  beforeEach(() => vi.useFakeTimers())
  afterEach(() => {
    clearGoalStatus()
    vi.useRealTimers()
  })

  it('holds a running goal for as long as it runs', () => {
    setGoalStatus(status({ progress: 0.3 }))
    vi.advanceTimersByTime(60_000)
    expect(get(goalStatus)?.progress).toBe(0.3)
  })

  it('keeps an ended goal for the hold, then drops it', () => {
    setGoalStatus(status({ state: GoalState.GOAL_REACHED, progress: 1 }))
    vi.advanceTimersByTime(GOAL_END_HOLD_MS - 1)
    expect(get(goalStatus)?.state).toBe(GoalState.GOAL_REACHED)
    vi.advanceTimersByTime(1)
    expect(get(goalStatus)).toBeNull()
  })

  it("lets a new goal cancel the old one's drop", () => {
    setGoalStatus(status({ id: 1, state: GoalState.GOAL_REACHED }))
    vi.advanceTimersByTime(GOAL_END_HOLD_MS - 100)
    setGoalStatus(status({ id: 2, progress: 0.1 }))
    vi.advanceTimersByTime(10_000)
    expect(get(goalStatus)?.id).toBe(2)
  })

  it('treats the idle status as no goal', () => {
    setGoalStatus(status({ state: GoalState.GOAL_RUNNING }))
    setGoalStatus(status({ state: GoalState.GOAL_IDLE }))
    expect(get(goalStatus)).toBeNull()
  })

  it('tells an active goal from an ended one', () => {
    expect(isGoalActive(status({ state: GoalState.GOAL_SETTLING }))).toBe(true)
    expect(isGoalActive(status({ state: GoalState.GOAL_RUNNING }))).toBe(true)
    expect(isGoalActive(status({ state: GoalState.GOAL_REACHED }))).toBe(false)
    expect(isGoalActive(status({ state: GoalState.GOAL_IDLE }))).toBe(false)
  })
})

// The data broker as followGoalStatus uses it: its goal status pushes, and requests answered when the test says.
const fakeBroker = () => {
  const pushes = new Set<(s: GoalStatus) => void>()
  const replies: ((reply: { goalStatus?: GoalStatus }) => void)[] = []
  const broker: GoalStatusLink = {
    on: (_fns, callback) => {
      pushes.add(callback)
      return () => pushes.delete(callback)
    },
    request: () => new Promise(resolve => replies.push(resolve))
  }
  return {
    broker,
    push: (s: GoalStatus) => pushes.forEach(callback => callback(s)),
    replies,
    listening: () => pushes.size
  }
}

describe('followGoalStatus', () => {
  const linked = writable(false)
  let stop: () => void = () => {}

  beforeEach(() => {
    vi.useFakeTimers()
    linked.set(false)
  })
  afterEach(() => {
    stop()
    clearGoalStatus()
    vi.useRealTimers()
  })

  it('asks once when the link comes up for a goal that was running before it listened', async () => {
    const fake = fakeBroker()
    stop = followGoalStatus(fake.broker, linked)
    expect(fake.replies).toHaveLength(0)
    linked.set(true)
    expect(fake.replies).toHaveLength(1)
    fake.replies[0]({ goalStatus: status({ progress: 0.4 }) })
    await vi.runAllTimersAsync()
    expect(get(goalStatus)?.progress).toBe(0.4)
    fake.push(status({ progress: 0.5 }))
    expect(fake.replies).toHaveLength(1)
  })

  it('shows no ended goal from the reply', async () => {
    const fake = fakeBroker()
    stop = followGoalStatus(fake.broker, linked)
    linked.set(true)
    fake.replies[0]({ goalStatus: status({ state: GoalState.GOAL_REACHED }) })
    await vi.runAllTimersAsync()
    expect(get(goalStatus)).toBeNull()
  })

  it('keeps a push that beat the reply, as it is newer', async () => {
    const fake = fakeBroker()
    stop = followGoalStatus(fake.broker, linked)
    linked.set(true)
    fake.push(status({ id: 2, progress: 0.1 }))
    fake.replies[0]({ goalStatus: status({ id: 1, progress: 0.9 }) })
    await vi.runAllTimersAsync()
    expect(get(goalStatus)?.id).toBe(2)
  })

  it('updates on every push', () => {
    const fake = fakeBroker()
    stop = followGoalStatus(fake.broker, linked)
    fake.push(status({ progress: 0.2 }))
    expect(get(goalStatus)?.progress).toBe(0.2)
    fake.push(status({ progress: 0.3 }))
    expect(get(goalStatus)?.progress).toBe(0.3)
  })

  it('clears at once when the link drops, without waiting out an ended goal', () => {
    const fake = fakeBroker()
    stop = followGoalStatus(fake.broker, linked)
    linked.set(true)
    fake.push(status({ state: GoalState.GOAL_REACHED }))
    expect(get(goalStatus)?.state).toBe(GoalState.GOAL_REACHED)
    linked.set(false)
    expect(get(goalStatus)).toBeNull()
  })

  it('stops listening when stopped', () => {
    const fake = fakeBroker()
    followGoalStatus(fake.broker, linked)()
    expect(fake.listening()).toBe(0)
    linked.set(true)
    expect(fake.replies).toHaveLength(0)
  })
})
