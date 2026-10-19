import { TaskState, type TaskStats } from '$lib/platform_shared/message'
import { ask } from '$lib/transport/reply'

type TaskStatsReply = { statusCode: number; errorMessage: string; taskStats?: TaskStats }

/** What the task table needs of the data broker (transport/databroker.ts): one request, which it can abort. */
export interface TaskStatsLink {
  request(
    payload: { taskStatsRequest: object },
    opts?: { signal?: AbortSignal }
  ): Promise<TaskStatsReply>
}

/**
 * How often the table refreshes itself when asked to: the robot's own minimum between two snapshots, since a snapshot
 * masks interrupts on both cores for milliseconds (firmware/include/task_table.h) and the robot hands a request that
 * comes sooner the last reply.
 */
export const TASK_STATS_REFRESH_MS = 10_000

/** What the page shows: the last good table, kept when a later request fails, and why that one did. */
export interface TaskStatsView {
  stats: TaskStats | null
  error: string | null
  loading: boolean
}

export interface TaskStatsFeed {
  /** Asks for the table now; does nothing while a request is outstanding. */
  refresh(): void
  /** Turns the refresh, TASK_STATS_REFRESH_MS after each reply or failure, on or off. */
  setAuto(on: boolean): void
  /** Aborts the request in flight and cancels the refresh; the page calls it on close. */
  stop(): void
}

/** Hands `show` the page's view on every change; nothing is requested until `refresh` is called. */
export const watchTaskStats = (
  broker: TaskStatsLink,
  show: (view: TaskStatsView) => void
): TaskStatsFeed => {
  const aborter = new AbortController()
  let view: TaskStatsView = { stats: null, error: null, loading: false }
  let auto = false
  let timer: ReturnType<typeof setTimeout> | undefined

  const update = (change: Partial<TaskStatsView>) => {
    view = { ...view, ...change }
    show(view)
  }
  const schedule = () => {
    clearTimeout(timer)
    if (auto && !aborter.signal.aborted) timer = setTimeout(refresh, TASK_STATS_REFRESH_MS)
  }
  async function refresh() {
    if (view.loading || aborter.signal.aborted) return
    clearTimeout(timer)
    update({ loading: true })
    try {
      const reply = await ask(broker, { taskStatsRequest: {} }, 'the task table', {
        signal: aborter.signal
      })
      if (aborter.signal.aborted) return
      const error = reply.taskStats ? null : 'the robot answered without a task table'
      update({ loading: false, error, ...(error ? {} : { stats: reply.taskStats }) })
    } catch (e) {
      if (aborter.signal.aborted) return
      update({ loading: false, error: e instanceof Error ? e.message : String(e) })
    }
    schedule()
  }
  return {
    refresh,
    setAuto: on => {
      auto = on
      if (!view.loading) schedule()
    },
    stop: () => {
      aborter.abort()
      clearTimeout(timer)
    }
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

/** What the State column's header says: the one running task is the one that took the snapshot. */
export const STATE_NOTE =
  'The transport task answering this request always shows as running. A task running on the other core shows as ready.'

/** The table's rows in the robot's order, the least free stack first; a reply without a window has no CPU figures. */
export const taskRows = (stats: TaskStats): TaskRow[] =>
  stats.tasks.map(task => ({
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

/** The untouched stack bytes the snapshot scanned, which uxTaskGetSystemState reads one at a time under its lock. */
export const scannedBytes = (stats: TaskStats) =>
  stats.tasks.reduce((sum, task) => sum + task.stackFreeMin, 0)

/**
 * The line under the table: how many tasks it lists, what its CPU figures cover, and what the snapshot cost beside the
 * bytes it scanned, so the first reading on a robot gives the cost of a byte. The robot cuts the roomiest stacks from
 * a long list, and those are the longest scans, so for a cut table the bytes are a lower bound.
 */
export const taskSummary = (stats: TaskStats) => {
  const cut = stats.tasks.length < stats.taskCount
  const listed =
    cut ? `${stats.tasks.length} of ${stats.taskCount} tasks` : `${stats.taskCount} tasks`
  const window =
    stats.windowUs > 0 ?
      `CPU of one core over ${(stats.windowUs / 1e6).toFixed(1)} s`
    : 'CPU from the next reply'
  const snapshot = `snapshot ${(stats.snapshotUs / 1000).toFixed(2)} ms`
  const scanned = `${cut ? 'at least ' : ''}${scannedBytes(stats)} B of free stack scanned`
  return `${listed}, ${window}, ${snapshot}, ${scanned}`
}
