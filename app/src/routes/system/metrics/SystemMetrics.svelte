<script lang="ts">
  import { onMount } from 'svelte'
  import { analytics } from '$lib/stores/analytics'
  import { AnalyticsData } from '$lib/platform_shared/message'
  import {
    STATE_NOTE,
    taskRows,
    taskSummary,
    watchTaskStats,
    TASK_STATS_REFRESH_MS,
    type TaskStatsView
  } from '$lib/stores/task-stats'
  import { dataBroker } from '$lib/transport/databroker'
  import { Connection, CPU, Folder, Heap, Metrics, Temperature } from '$lib/components/icons'
  import LineChart from '$lib/components/LineChart.svelte'
  import Card from '$lib/components/layout/Card.svelte'
  import LoadFailure from '$lib/components/layout/LoadFailure.svelte'
  import Loading from '$lib/components/layout/Loading.svelte'

  let taskView: TaskStatsView = $state({ stats: null, error: null, loading: false })
  let autoRefresh = $state(false)
  const taskFeed = watchTaskStats(dataBroker, view => (taskView = view))

  onMount(() => {
    const unsubscribe = dataBroker.on(AnalyticsData, data => analytics.addData(data))
    taskFeed.refresh()
    return () => {
      unsubscribe()
      taskFeed.stop()
    }
  })

  // The latest total, which the robot reports with every sample, bounds the used amount.
  const upTo = (totals: number[]) => ({ min: 0, max: Math.ceil(totals.at(-1) ?? 0) || undefined })
</script>

<Card title="CPU" icon={CPU}>
  <LineChart
    unit="Usage [%]"
    labels={$analytics.uptime}
    series={[
      { label: 'Core 0', colour: 'primary', data: $analytics.cpu0_usage },
      { label: 'Core 1', colour: 'accent', data: $analytics.cpu1_usage },
      { label: 'Total', colour: 'secondary', data: $analytics.cpu_usage }
    ]}
    range={{ min: 0, max: 100 }}
  />
</Card>

<Card title="Heap" icon={Heap}>
  <LineChart
    unit="Used [kB]"
    labels={$analytics.uptime}
    series={[{ label: 'Used heap', colour: 'primary', data: $analytics.used_heap, fill: true }]}
    range={upTo($analytics.total_heap)}
  />
</Card>

<Card title="File system" icon={Folder}>
  <LineChart
    unit="Used [kB]"
    labels={$analytics.uptime}
    series={[
      { label: 'File system used', colour: 'primary', data: $analytics.fs_used, fill: true }
    ]}
    range={upTo($analytics.fs_total)}
  />
</Card>

<Card title="Core temperature" icon={Temperature}>
  <LineChart
    unit="Temperature [°C]"
    labels={$analytics.uptime}
    series={[{ label: 'Core temperature', colour: 'primary', data: $analytics.core_temp }]}
    range={{ suggestedMin: 20, suggestedMax: 100 }}
  />
</Card>

<Card title="I2C bus" icon={Connection}>
  {#if $analytics.i2c_errors.length}
    <div class="bg-base-100 rounded-box overflow-x-auto">
      <table class="table-sm table">
        <thead>
          <tr>
            <th>Device</th>
            <th>NACKs</th>
            <th>Timeouts</th>
          </tr>
        </thead>
        <tbody>
          {#each $analytics.i2c_errors as device (device.address)}
            <tr>
              <td class="font-mono">0x{device.address.toString(16).padStart(2, '0')}</td>
              <td>{device.nacks}</td>
              <td>{device.timeouts}</td>
            </tr>
          {/each}
        </tbody>
      </table>
    </div>
  {:else}
    <p class="text-base-content/70 text-sm">No device errors reported yet.</p>
  {/if}
  <p class="text-sm">Bus recoveries: {$analytics.i2c_bus_recoveries}</p>
</Card>

<Card title="Tasks" icon={Metrics}>
  {#snippet actions()}
    <label class="label cursor-pointer gap-2 text-sm">
      <input
        type="checkbox"
        class="toggle toggle-sm"
        bind:checked={autoRefresh}
        onchange={() => taskFeed.setAuto(autoRefresh)}
      />
      Every {TASK_STATS_REFRESH_MS / 1000} s
    </label>
    <button class="btn btn-sm" disabled={taskView.loading} onclick={() => taskFeed.refresh()}>
      Refresh
    </button>
  {/snippet}
  {#if taskView.error}
    <LoadFailure message="Task table failed: {taskView.error}" retry={() => taskFeed.refresh()} />
  {/if}
  {#if taskView.stats}
    <!-- A phone lists each task as a row of its own; the table's six columns would not fit. -->
    <ul class="list bg-base-100 rounded-box sm:hidden">
      {#each taskRows(taskView.stats) as task (task.key)}
        <li class="list-row block py-2">
          <div class="flex items-baseline justify-between gap-2">
            <span class="truncate font-medium">{task.name}</span>
            <span class="whitespace-nowrap text-sm">{task.cpu}</span>
          </div>
          <div class="text-base-content/70 text-xs">
            Stack {task.stack} free, core {task.core}, priority {task.priority},
            <span title={STATE_NOTE}>{task.state}</span>
          </div>
        </li>
      {/each}
    </ul>
    <div class="bg-base-100 rounded-box hidden overflow-x-auto sm:block">
      <table class="table-sm table">
        <thead>
          <tr>
            <th>Task</th>
            <th>Least free stack</th>
            <th>CPU</th>
            <th>Core</th>
            <th>Priority</th>
            <th title={STATE_NOTE} class="cursor-help underline decoration-dotted">State</th>
          </tr>
        </thead>
        <tbody>
          {#each taskRows(taskView.stats) as task (task.key)}
            <tr>
              <td>{task.name}</td>
              <td>{task.stack}</td>
              <td class="whitespace-nowrap">{task.cpu}</td>
              <td>{task.core}</td>
              <td>{task.priority}</td>
              <td>{task.state}</td>
            </tr>
          {/each}
        </tbody>
      </table>
    </div>
    <p class="text-base-content/70 text-sm">{taskSummary(taskView.stats)}</p>
  {:else if taskView.loading}
    <Loading label="Reading the task table" />
  {/if}
</Card>
