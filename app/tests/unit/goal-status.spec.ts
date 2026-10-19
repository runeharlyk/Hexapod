import { describe, expect, it } from 'vitest'
import { GoalKind, GoalState, type GoalStatus } from '../../src/lib/platform_shared/message'
import { goalLine } from '../../src/lib/stores/goal'

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
