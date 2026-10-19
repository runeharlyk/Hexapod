import { derived, readable, type Readable } from 'svelte/store'
import {
  AnimationStatus,
  GaitData,
  GoalState,
  GoalStatus,
  ModeData,
  OtaStatusData
} from '$lib/platform_shared/message'
import { GaitType } from '$lib/gait'
import { MotionModes } from '$lib/motion'
import { GOAL_END_HOLD_MS, isGoalActive } from '$lib/stores/goal'
import { dataBroker, type DataBroker } from '$lib/transport/databroker'
import { describeFailure } from '$lib/transport/reply'
import type { Watched } from '$lib/transport/topics'
import { watch } from './watch'

/**
 * The goal to show: a running one, and an ended one for GOAL_END_HOLD_MS after it ended. An ended goal in the snapshot
 * that a subscribe is answered with, before the ack, is history rather than an event, so it is not shown; nor is a
 * goal while the link is down.
 */
const shownGoal = (goal: Readable<Watched<GoalStatus>>): Readable<GoalStatus | null> =>
  readable<GoalStatus | null>(null, set => {
    let seen: GoalStatus | undefined
    let hold: ReturnType<typeof setTimeout> | undefined
    const stop = goal.subscribe(({ value, status, stale }) => {
      if (stale || value === undefined) {
        seen = undefined
        clearTimeout(hold)
        return set(null)
      }
      if (value === seen) return
      seen = value
      clearTimeout(hold)
      if (isGoalActive(value)) return set(value)
      if (value.state === GoalState.GOAL_IDLE || status !== 'live') return set(null)
      set(value)
      hold = setTimeout(() => set(null), GOAL_END_HOLD_MS)
    })
    return () => {
      clearTimeout(hold)
      stop()
    }
  })

/**
 * The robot's state topics the layout follows, as watches, and the stores the pages read from them. The mode keeps its
 * last value while the link is down; whether the handheld drives, the animation and the goal are cleared then.
 */
export const robotState = (broker: DataBroker) => {
  const modeState = watch(ModeData, broker)
  const gaitState = watch(GaitData, broker)
  const goalState = watch(GoalStatus, broker)
  const animationState = watch(AnimationStatus, broker)
  const otaState = watch(OtaStatusData, broker)
  return {
    modeState,
    gaitState,
    goalState,
    animationState,
    otaState,
    mode: derived(modeState, ({ value }): MotionModes =>
      value === undefined ?
        MotionModes.DEACTIVATED
      : (Object.values(MotionModes)[value.mode] ?? MotionModes.DEACTIVATED)
    ),
    /** Whether the ESP-NOW handheld drives the robot, which then drops this app's sticks and refuses its goals. */
    handheldDriving: derived(modeState, ({ value, stale }) => !stale && (value?.handheld ?? false)),
    gait: derived(gaitState, ({ value }): GaitType =>
      value === undefined ?
        GaitType.TRI_GATE
      : (Object.values(GaitType)[value.gait] ?? GaitType.TRI_GATE)
    ),
    goalStatus: shownGoal(goalState),
    animationStatus: derived(animationState, ({ value, stale }) => (stale ? null : (value ?? null)))
  }
}

export const {
  modeState,
  gaitState,
  goalState,
  animationState,
  otaState,
  mode,
  handheldDriving,
  gait,
  goalStatus,
  animationStatus
} = robotState(dataBroker)

/**
 * Says once, each time a watch fails, why: a firmware from before the snapshots never acks a subscribe, and every watch
 * would otherwise end in an error that nothing on the page shows. Returns the function that stops listening.
 */
export const reportFailure = (
  state: Readable<Watched<unknown>>,
  feature: string,
  report: (message: string) => void
) => {
  let failed = false
  return state.subscribe(({ status, failure }) => {
    if (status === 'error' && failure && !failed) report(describeFailure(failure, feature))
    failed = status === 'error'
  })
}
