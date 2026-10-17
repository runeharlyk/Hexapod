import { writable, type Readable } from 'svelte/store'
import { GoalKind, GoalState, type GoalStatus } from '$lib/platform_shared/message'

export const GOAL_END_HOLD_MS = 3000

const status = writable<GoalStatus | null>(null)
let dropTimer: ReturnType<typeof setTimeout> | undefined

export const goalStatus: Readable<GoalStatus | null> = { subscribe: status.subscribe }

export const isGoalActive = (s: GoalStatus) =>
  s.state === GoalState.GOAL_SETTLING || s.state === GoalState.GOAL_RUNNING

const ENDED: Partial<Record<GoalState, string>> = {
  [GoalState.GOAL_REACHED]: 'reached',
  [GoalState.GOAL_CANCELLED]: 'cancelled',
  [GoalState.GOAL_TIMED_OUT]: 'timed out',
  [GoalState.GOAL_REFUSED]: 'refused'
}

const MOVES: Record<GoalKind, { noun: string; running: string; unit: 'm' | 'deg' }> = {
  [GoalKind.GOAL_FORWARD]: { noun: 'walk', running: 'walking', unit: 'm' },
  [GoalKind.GOAL_SIDEWAYS]: { noun: 'sidestep', running: 'sidestepping', unit: 'm' },
  [GoalKind.GOAL_TURN]: { noun: 'turn', running: 'turning', unit: 'deg' },
  [GoalKind.GOAL_ARC]: { noun: 'arc', running: 'arc', unit: 'deg' },
  [GoalKind.UNRECOGNIZED]: { noun: 'move', running: 'moving', unit: 'm' }
}

const amount = (value: number, unit: 'm' | 'deg') =>
  unit === 'm' ? value.toFixed(2) : ((value * 180) / Math.PI).toFixed(0)

export const goalLine = (s: GoalStatus): string | null => {
  const move = MOVES[s.kind]
  if (s.state === GoalState.GOAL_SETTLING) return `${move.noun} settling`
  if (s.state === GoalState.GOAL_RUNNING) {
    const target = Math.abs(s.target)
    const covered = target * Math.min(Math.max(s.progress, 0), 1)
    return `${move.running} ${amount(covered, move.unit)} / ${amount(target, move.unit)} ${move.unit}`
  }
  const ended = ENDED[s.state]
  return ended ? `${move.noun} ${ended}` : null
}

const stopDrop = () => {
  clearTimeout(dropTimer)
  dropTimer = undefined
}

export const clearGoalStatus = () => {
  stopDrop()
  status.set(null)
}

export const setGoalStatus = (s: GoalStatus) => {
  stopDrop()
  if (s.state === GoalState.GOAL_IDLE) return status.set(null)
  status.set(s)
  if (!isGoalActive(s)) dropTimer = setTimeout(clearGoalStatus, GOAL_END_HOLD_MS)
}
