import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { TaskState, type TaskEntry, type TaskStats } from '../../src/lib/platform_shared/message'
import {
  pollTaskStats,
  taskRows,
  taskSummary,
  TASK_STATS_PERIOD_MS,
  type TaskStatsLink
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

// The data broker as the poll uses it: requests answered or refused when the test says, and the aborts it was sent.
const fakeBroker = () => {
  const replies: ((reply: { taskStats?: TaskStats }) => void)[] = []
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
  it('lists the least free stack first, ties by name and then by task number', () => {
    const rows = taskRows(
      stats({
        tasks: [
          entry({ name: 'serial_rx', number: 9, stackFreeMin: 5800 }),
          entry({ name: 'evtbus', number: 14, stackFreeMin: 1800 }),
          entry({ name: 'control', number: 3, stackFreeMin: 900 }),
          entry({ name: 'evtbus', number: 12, stackFreeMin: 1800 }),
          entry({ name: 'dns_server', number: 20, stackFreeMin: 1800 })
        ]
      })
    )
    expect(rows.map(row => [row.name, row.key])).toEqual([
      ['control', 3],
      ['dns_server', 20],
      ['evtbus', 12],
      ['evtbus', 14],
      ['serial_rx', 9]
    ])
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
  it('gives the task count, the window and the snapshot time', () => {
    expect(taskSummary(stats({ taskCount: 2, tasks: [entry({}), entry({ number: 2 })] }))).toBe(
      '2 tasks, CPU of one core over 2.0 s, snapshot 1.20 ms'
    )
  })

  it('says how many tasks the robot cut, and that the first reply has no CPU', () => {
    expect(taskSummary(stats({ windowUs: 0, taskCount: 45, tasks: [entry({})] }))).toBe(
      '1 of 45 tasks, CPU from the next reply, snapshot 1.20 ms'
    )
  })
})

describe('pollTaskStats', () => {
  let stop: () => void = () => {}

  beforeEach(() => vi.useFakeTimers())
  afterEach(() => {
    stop()
    vi.useRealTimers()
  })

  it('asks at once, then one period after each reply, never with a request outstanding', async () => {
    const fake = fakeBroker()
    const shown: TaskStats[] = []
    stop = pollTaskStats(fake.broker, s => shown.push(s))
    expect(fake.replies).toHaveLength(1)

    await vi.advanceTimersByTimeAsync(10 * TASK_STATS_PERIOD_MS)
    expect(fake.replies).toHaveLength(1)

    const first = stats({ taskCount: 1 })
    fake.replies[0]({ taskStats: first })
    await vi.advanceTimersByTimeAsync(TASK_STATS_PERIOD_MS - 1)
    expect(shown).toEqual([first])
    expect(fake.replies).toHaveLength(1)
    await vi.advanceTimersByTimeAsync(1)
    expect(fake.replies).toHaveLength(2)
  })

  it('goes on asking after a request fails', async () => {
    const fake = fakeBroker()
    stop = pollTaskStats(fake.broker, () => {})
    fake.failures[0](new Error('request 1 timed out'))
    await vi.advanceTimersByTimeAsync(TASK_STATS_PERIOD_MS)
    expect(fake.replies).toHaveLength(2)
  })

  it('stops asking when the page closes, and aborts the request in flight', async () => {
    const fake = fakeBroker()
    const shown: TaskStats[] = []
    stop = pollTaskStats(fake.broker, s => shown.push(s))
    stop()
    expect(fake.signals[0].aborted).toBe(true)
    fake.replies[0]({ taskStats: stats({}) })
    await vi.advanceTimersByTimeAsync(10 * TASK_STATS_PERIOD_MS)
    expect(shown).toEqual([])
    expect(fake.replies).toHaveLength(1)
    expect(vi.getTimerCount()).toBe(0)
  })

  it('schedules nothing more when it stops between two requests', async () => {
    const fake = fakeBroker()
    stop = pollTaskStats(fake.broker, () => {})
    fake.replies[0]({ taskStats: stats({}) })
    await vi.advanceTimersByTimeAsync(1)
    expect(vi.getTimerCount()).toBe(1)
    stop()
    expect(vi.getTimerCount()).toBe(0)
  })
})
