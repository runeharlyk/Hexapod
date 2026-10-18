import { TaskState, type TaskEntry, type TaskStats } from '$lib/platform_shared/message'

/** What the task table needs of the data broker (transport/databroker.ts): one request, which it can abort. */
export interface TaskStatsLink {
  request(
    payload: { taskStatsRequest: object },
    opts?: { signal?: AbortSignal }
  ): Promise<{ taskStats?: TaskStats }>
}

/**
 * How often the open metrics page asks: the period of the analytics stream, so the table moves with the CPU chart.
 * The robot measures CPU over the time between two snapshots, and each snapshot holds its kernel's lock for a
 * millisecond or two, so a shorter period would cost the control loop more for no steadier figure.
 */
export const TASK_STATS_PERIOD_MS = 2000

/**
 * Asks for the task table now, and again one period after each reply or failure, so a slow link never stacks
 * requests; hands each table to `show`. Returns the function that stops, which aborts a request in flight.
 */
export const pollTaskStats = (
  broker: TaskStatsLink,
  show: (stats: TaskStats) => void,
  periodMs = TASK_STATS_PERIOD_MS
) => {
  const aborter = new AbortController()
  let timer: ReturnType<typeof setTimeout> | undefined
  const ask = async () => {
    const reply = await broker
      .request({ taskStatsRequest: {} }, { signal: aborter.signal })
      .catch(() => null)
    if (aborter.signal.aborted) return
    if (reply?.taskStats) show(reply.taskStats)
    timer = setTimeout(ask, periodMs)
  }
  ask()
  return () => {
    aborter.abort()
    clearTimeout(timer)
  }
}

export interface TaskRow {
  key: number
  name: string
  state: string
  priority: string
  core: string
  stack: string
  cpu: string
}

const STATE_NAMES: Record<number, string> = {
  [TaskState.TASK_RUNNING]: 'running',
  [TaskState.TASK_READY]: 'ready',
  [TaskState.TASK_BLOCKED]: 'blocked',
  [TaskState.TASK_SUSPENDED]: 'suspended',
  [TaskState.TASK_DELETED]: 'deleted'
}

// As the robot orders them (firmware/include/task_table.h): the stack closest to an overflow first.
const tightestFirst = (a: TaskEntry, b: TaskEntry) =>
  a.stackFreeMin - b.stackFreeMin ||
  (a.name < b.name ? -1
  : a.name > b.name ? 1
  : 0) ||
  a.number - b.number

/** The table's rows, the least free stack first; a reply without a window has no CPU figures. */
export const taskRows = (stats: TaskStats): TaskRow[] =>
  [...stats.tasks].sort(tightestFirst).map(task => ({
    key: task.number,
    name: task.name,
    state: STATE_NAMES[task.state] ?? 'unknown',
    priority:
      task.priority === task.basePriority ?
        `${task.priority}`
      : `${task.priority} (base ${task.basePriority})`,
    core: task.core < 0 ? 'either' : `${task.core}`,
    stack: `${task.stackFreeMin} B`,
    cpu: stats.windowUs > 0 ? `${task.cpuPercent.toFixed(1)} %` : '-'
  }))

/** The line under the table: how many tasks it lists, what its CPU figures cover, and what the snapshot cost. */
export const taskSummary = (stats: TaskStats) => {
  const listed =
    stats.tasks.length < stats.taskCount ?
      `${stats.tasks.length} of ${stats.taskCount} tasks`
    : `${stats.taskCount} tasks`
  const window =
    stats.windowUs > 0 ?
      `CPU of one core over ${(stats.windowUs / 1e6).toFixed(1)} s`
    : 'CPU from the next reply'
  return `${listed}, ${window}, snapshot ${(stats.snapshotUs / 1000).toFixed(2)} ms`
}
