import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { TaskState, type TaskEntry, type TaskStats } from '../../src/lib/platform_shared/message'
import {
  scannedBytes,
  taskRows,
  taskSummary,
  watchTaskStats,
  TASK_STATS_REFRESH_MS,
  type TaskStatsLink,
  type TaskStatsView
} from '../../src/lib/stores/task-stats'

const entry = (fields: Partial<TaskEntry>): TaskEntry => ({
  name: 'task',
  number: 1,
  state: TaskState.TASK_BLOCKED,
  priority: 5,
  basePriority: 5,
  core: -1,
  stackFreeMin: 1000,
  cpuPercent: 0,
  ...fields
})

const stats = (fields: Partial<TaskStats>): TaskStats => ({
  windowUs: 2_000_000,
  snapshotUs: 1200,
  taskCount: 0,
  tasks: [],
  ...fields
})

type Reply = Awaited<ReturnType<TaskStatsLink['request']>>
const ok = (taskStats: TaskStats): Reply => ({ statusCode: 200, errorMessage: '', taskStats })

// The data broker as the feed uses it: requests answered or refused when the test says, and the aborts it was sent.
const fakeBroker = () => {
  const replies: ((reply: Reply) => void)[] = []
  const failures: ((error: Error) => void)[] = []
  const signals: AbortSignal[] = []
  const broker: TaskStatsLink = {
    request: (_payload, opts) =>
      new Promise((resolve, reject) => {
        replies.push(resolve)
        failures.push(reject)
        if (opts?.signal) signals.push(opts.signal)
      })
  }
  return { broker, replies, failures, signals }
}

describe('taskRows', () => {
  // The robot sorts, and cuts the roomiest stacks by that order: the page shows what it was sent.
  it('keeps the order the robot sent', () => {
    const rows = taskRows(
      stats({
        tasks: [
          entry({ name: 'control', number: 3, stackFreeMin: 900 }),
          entry({ name: 'evtbus', number: 14, stackFreeMin: 1800 }),
          entry({ name: 'dns_server', number: 20, stackFreeMin: 1800 }),
          entry({ name: 'serial_rx', number: 9, stackFreeMin: 800 })
        ]
      })
    )
    expect(rows.map(row => row.key)).toEqual([3, 14, 20, 9])
  })

  it('formats a task as the table shows it', () => {
    const [row] = taskRows(
      stats({
        tasks: [
          entry({
            name: 'control',
            state: TaskState.TASK_READY,
            priority: 5,
            basePriority: 5,
            core: 1,
            stackFreeMin: 912,
            cpuPercent: 31.25
          })
        ]
      })
    )
    expect(row).toEqual({
      key: 1,
      name: 'control',
      state: 'ready',
      priority: '5',
      core: '1',
      stack: '912 B',
      cpu: '31.3 %'
    })
  })

  it('shows an inherited priority beside the base one, and an unpinned task on either core', () => {
    const [row] = taskRows(stats({ tasks: [entry({ priority: 6, basePriority: 2, core: -1 })] }))
    expect(row.priority).toBe('6 (base 2)')
    expect(row.core).toBe('either')
  })

  it('has no CPU figure on the first reply, which has no window', () => {
    const [row] = taskRows(stats({ windowUs: 0, tasks: [entry({ cpuPercent: 0 })] }))
    expect(row.cpu).toBe('-')
  })

  it('names every state the robot sends', () => {
    const states = [
      TaskState.TASK_RUNNING,
      TaskState.TASK_READY,
      TaskState.TASK_BLOCKED,
      TaskState.TASK_SUSPENDED,
      TaskState.TASK_DELETED
    ]
    const rows = taskRows(
      stats({ tasks: states.map((state, i) => entry({ state, number: i, stackFreeMin: i })) })
    )
    expect(rows.map(row => row.state)).toEqual([
      'running',
      'ready',
      'blocked',
      'suspended',
      'deleted'
    ])
  })
})

describe('taskSummary', () => {
  it('gives the task count, the window, the snapshot time and the bytes it scanned', () => {
    const tasks = [entry({ stackFreeMin: 900 }), entry({ number: 2, stackFreeMin: 1100 })]
    expect(scannedBytes(stats({ taskCount: 2, tasks }))).toBe(2000)
    expect(taskSummary(stats({ taskCount: 2, tasks }))).toBe(
      '2 tasks, CPU of one core over 2.0 s, snapshot 1.20 ms, 2000 B of free stack scanned'
    )
  })

  it('says how many tasks the robot cut, and that the scanned bytes are then a lower bound', () => {
    expect(taskSummary(stats({ windowUs: 0, taskCount: 45, tasks: [entry({})] }))).toBe(
      '1 of 45 tasks, CPU from the next reply, snapshot 1.20 ms, at least 1000 B of free stack scanned'
    )
  })
})

describe('watchTaskStats', () => {
  let views: TaskStatsView[]
  let feed: ReturnType<typeof watchTaskStats>
  const start = (broker: TaskStatsLink) => {
    views = []
    feed = watchTaskStats(broker, view => views.push(view))
  }
  const last = () => views[views.length - 1]

  beforeEach(() => vi.useFakeTimers())
  afterEach(() => {
    feed.stop()
    vi.useRealTimers()
  })

  it('asks for nothing until it is told to', async () => {
    const fake = fakeBroker()
    start(fake.broker)
    await vi.advanceTimersByTimeAsync(10 * TASK_STATS_REFRESH_MS)
    expect(fake.replies).toHaveLength(0)
    expect(views).toEqual([])
  })

  it('shows the table of a refresh, loading until it arrives', async () => {
    const fake = fakeBroker()
    start(fake.broker)
    feed.refresh()
    expect(last()).toEqual({ stats: null, error: null, loading: true })
    const table = stats({ taskCount: 1, tasks: [entry({})] })
    fake.replies[0](ok(table))
    await vi.advanceTimersByTimeAsync(0)
    expect(last()).toEqual({ stats: table, error: null, loading: false })
  })

  it('never has two requests outstanding', async () => {
    const fake = fakeBroker()
    start(fake.broker)
    feed.refresh()
    feed.refresh()
    expect(fake.replies).toHaveLength(1)
    fake.replies[0](ok(stats({})))
    await vi.advanceTimersByTimeAsync(0)
    feed.refresh()
    expect(fake.replies).toHaveLength(2)
  })

  it('does not refresh by itself while auto-refresh is off', async () => {
    const fake = fakeBroker()
    start(fake.broker)
    feed.refresh()
    fake.replies[0](ok(stats({})))
    await vi.advanceTimersByTimeAsync(10 * TASK_STATS_REFRESH_MS)
    expect(fake.replies).toHaveLength(1)
    expect(vi.getTimerCount()).toBe(0)
  })

  it('refreshes one interval after each reply while auto-refresh is on', async () => {
    const fake = fakeBroker()
    start(fake.broker)
    feed.setAuto(true)
    feed.refresh()
    fake.replies[0](ok(stats({})))
    await vi.advanceTimersByTimeAsync(TASK_STATS_REFRESH_MS - 1)
    expect(fake.replies).toHaveLength(1)
    await vi.advanceTimersByTimeAsync(1)
    expect(fake.replies).toHaveLength(2)
    fake.replies[1](ok(stats({})))
    await vi.advanceTimersByTimeAsync(TASK_STATS_REFRESH_MS)
    expect(fake.replies).toHaveLength(3)
  })

  it('counts the interval from the reply, so a slow link never stacks requests', async () => {
    const fake = fakeBroker()
    start(fake.broker)
    feed.setAuto(true)
    feed.refresh()
    await vi.advanceTimersByTimeAsync(5 * TASK_STATS_REFRESH_MS)
    expect(fake.replies).toHaveLength(1)
  })

  it('starts the interval when auto-refresh is turned on, and stops it when turned off', async () => {
    const fake = fakeBroker()
    start(fake.broker)
    feed.refresh()
    fake.replies[0](ok(stats({})))
    await vi.advanceTimersByTimeAsync(0)
    feed.setAuto(true)
    await vi.advanceTimersByTimeAsync(TASK_STATS_REFRESH_MS)
    expect(fake.replies).toHaveLength(2)
    feed.setAuto(false)
    fake.replies[1](ok(stats({})))
    await vi.advanceTimersByTimeAsync(10 * TASK_STATS_REFRESH_MS)
    expect(fake.replies).toHaveLength(2)
    expect(vi.getTimerCount()).toBe(0)
  })

  it('restarts the interval after a manual refresh instead of asking twice', async () => {
    const fake = fakeBroker()
    start(fake.broker)
    feed.setAuto(true)
    feed.refresh()
    fake.replies[0](ok(stats({})))
    await vi.advanceTimersByTimeAsync(TASK_STATS_REFRESH_MS - 1000)
    feed.refresh()
    fake.replies[1](ok(stats({})))
    await vi.advanceTimersByTimeAsync(TASK_STATS_REFRESH_MS - 1)
    expect(fake.replies).toHaveLength(2)
    await vi.advanceTimersByTimeAsync(1)
    expect(fake.replies).toHaveLength(3)
  })

  it('keeps the last good table and shows the error when the robot refuses', async () => {
    const fake = fakeBroker()
    start(fake.broker)
    const table = stats({ taskCount: 1, tasks: [entry({})] })
    feed.refresh()
    fake.replies[0](ok(table))
    await vi.advanceTimersByTimeAsync(0)
    feed.refresh()
    fake.replies[1]({
      statusCode: 507,
      errorMessage: 'No room for the task snapshot',
      taskStats: stats({ taskCount: 70 })
    })
    await vi.advanceTimersByTimeAsync(0)
    expect(last()).toEqual({ stats: table, error: 'No room for the task snapshot', loading: false })
  })

  it('names the status when the robot refuses without a message', async () => {
    const fake = fakeBroker()
    start(fake.broker)
    feed.refresh()
    fake.replies[0]({ statusCode: 503, errorMessage: '' })
    await vi.advanceTimersByTimeAsync(0)
    expect(last().error).toBe('the robot answered with status 503')
    expect(last().stats).toBeNull()
  })

  // A firmware from before the task table answers a request it does not know with a bare 400.
  it('says the firmware has no task table when the robot answers 400 without a message', async () => {
    const fake = fakeBroker()
    start(fake.broker)
    feed.refresh()
    fake.replies[0]({ statusCode: 400, errorMessage: '' })
    await vi.advanceTimersByTimeAsync(0)
    expect(last().error).toBe("the robot's firmware has no task table; update it to see the tasks")
  })

  it('keeps the last good table when a request fails, and clears the error on the next good one', async () => {
    const fake = fakeBroker()
    start(fake.broker)
    const table = stats({ taskCount: 1, tasks: [entry({})] })
    feed.refresh()
    fake.replies[0](ok(table))
    await vi.advanceTimersByTimeAsync(0)
    feed.refresh()
    fake.failures[1](new Error('request 2 timed out'))
    await vi.advanceTimersByTimeAsync(0)
    expect(last()).toEqual({ stats: table, error: 'request 2 timed out', loading: false })
    const next = stats({ taskCount: 2, tasks: [entry({}), entry({ number: 2 })] })
    feed.refresh()
    fake.replies[2](ok(next))
    await vi.advanceTimersByTimeAsync(0)
    expect(last()).toEqual({ stats: next, error: null, loading: false })
  })

  it('goes on refreshing after a failure while auto-refresh is on', async () => {
    const fake = fakeBroker()
    start(fake.broker)
    feed.setAuto(true)
    feed.refresh()
    fake.failures[0](new Error('request 1 timed out'))
    await vi.advanceTimersByTimeAsync(TASK_STATS_REFRESH_MS)
    expect(fake.replies).toHaveLength(2)
  })

  it('stops when the page closes, aborting the request in flight and showing nothing more', async () => {
    const fake = fakeBroker()
    start(fake.broker)
    feed.setAuto(true)
    feed.refresh()
    const shown = views.length
    feed.stop()
    expect(fake.signals[0].aborted).toBe(true)
    fake.replies[0](ok(stats({})))
    await vi.advanceTimersByTimeAsync(10 * TASK_STATS_REFRESH_MS)
    expect(views).toHaveLength(shown)
    expect(fake.replies).toHaveLength(1)
    expect(vi.getTimerCount()).toBe(0)
    feed.refresh()
    expect(fake.replies).toHaveLength(1)
  })

  it('schedules nothing more when it stops between two requests', async () => {
    const fake = fakeBroker()
    start(fake.broker)
    feed.setAuto(true)
    feed.refresh()
    fake.replies[0](ok(stats({})))
    await vi.advanceTimersByTimeAsync(1)
    expect(vi.getTimerCount()).toBe(1)
    feed.stop()
    expect(vi.getTimerCount()).toBe(0)
    feed.setAuto(true)
    expect(vi.getTimerCount()).toBe(0)
  })
})
