<script lang="ts">
  import { onMount } from 'svelte'
  import SettingsCard from '$lib/components/SettingsCard.svelte'
  import { slide } from 'svelte/transition'
  import { cubicOut } from 'svelte/easing'
  import { Chart, registerables } from 'chart.js'

  import { daisyColor } from '$lib/utilities'
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
  import { Metrics } from '$lib/components/icons'

  Chart.register(...registerables)

  let cpuChartElement: HTMLCanvasElement
  let cpuChart: Chart

  let heapChartElement: HTMLCanvasElement
  let heapChart: Chart

  let filesystemChartElement: HTMLCanvasElement
  let filesystemChart: Chart

  let temperatureChartElement: HTMLCanvasElement
  let temperatureChart: Chart

  let taskView: TaskStatsView = { stats: null, error: null, loading: false }
  let autoRefresh = false
  const taskFeed = watchTaskStats(dataBroker, view => (taskView = view))

  onMount(() => {
    const unsubscribe = dataBroker.on(AnalyticsData, data => analytics.addData(data))
    cpuChart = new Chart(cpuChartElement, {
      type: 'line',
      data: {
        labels: $analytics.uptime,
        datasets: [
          {
            label: 'Cpu usage core 0',
            borderColor: daisyColor('primary'),
            backgroundColor: daisyColor('primary', 50),
            borderWidth: 2,
            data: $analytics.cpu0_usage,
            yAxisID: 'y'
          },
          {
            label: 'Cpu usage core 1',
            borderColor: daisyColor('accent'),
            backgroundColor: daisyColor('accent', 50),
            borderWidth: 2,
            data: $analytics.cpu1_usage,
            yAxisID: 'y'
          },
          {
            label: 'Cpu usage total',
            borderColor: daisyColor('secondary'),
            backgroundColor: daisyColor('secondary', 50),
            borderWidth: 2,
            data: $analytics.cpu_usage,
            yAxisID: 'y'
          }
        ]
      },
      options: {
        maintainAspectRatio: false,
        responsive: true,
        plugins: {
          legend: { display: true },
          tooltip: { mode: 'index', intersect: false }
        },
        elements: { point: { radius: 0 } },
        scales: {
          x: {
            grid: { color: daisyColor('base-content', 10) },
            ticks: { color: daisyColor('base-content') },
            display: false
          },
          y: {
            type: 'linear',
            title: {
              display: true,
              text: 'Cpu usage [%]',
              color: daisyColor('base-content'),
              font: { size: 16, weight: 'bold' }
            },
            position: 'left',
            min: 0,
            max: 100,
            grid: { color: daisyColor('base-content', 10) },
            ticks: { color: daisyColor('base-content') },
            border: { color: daisyColor('base-content', 10) }
          }
        }
      }
    })
    heapChart = new Chart(heapChartElement, {
      type: 'line',
      data: {
        labels: $analytics.uptime,
        datasets: [
          {
            label: 'Used Heap',
            borderColor: daisyColor('primary'),
            backgroundColor: daisyColor('primary', 50),
            borderWidth: 2,
            data: $analytics.used_heap,
            fill: true,
            yAxisID: 'y'
          }
        ]
      },
      options: {
        maintainAspectRatio: false,
        responsive: true,
        plugins: {
          legend: {
            display: true
          },
          tooltip: {
            mode: 'index',
            intersect: false
          }
        },
        elements: {
          point: {
            radius: 0
          }
        },
        scales: {
          x: {
            grid: {
              color: daisyColor('base-content', 10)
            },
            ticks: {
              color: daisyColor('base-content')
            },
            display: false
          },
          y: {
            type: 'linear',
            title: {
              display: true,
              text: 'Heap [kb]',
              color: daisyColor('base-content'),
              font: {
                size: 16,
                weight: 'bold'
              }
            },
            position: 'left',
            min: 0,
            max: Math.round($analytics.total_heap[0]),
            grid: { color: daisyColor('base-content', 10) },
            ticks: {
              color: daisyColor('base-content')
            },
            border: { color: daisyColor('base-content', 10) }
          }
        }
      }
    })
    filesystemChart = new Chart(filesystemChartElement, {
      type: 'line',
      data: {
        labels: $analytics.uptime,
        datasets: [
          {
            label: 'File System Used',
            borderColor: daisyColor('primary'),
            backgroundColor: daisyColor('primary', 50),
            borderWidth: 2,
            data: $analytics.fs_used,
            fill: true,
            yAxisID: 'y'
          }
        ]
      },
      options: {
        maintainAspectRatio: false,
        responsive: true,
        plugins: {
          legend: {
            display: true
          },
          tooltip: {
            mode: 'index',
            intersect: false
          }
        },
        elements: {
          point: {
            radius: 0
          }
        },
        scales: {
          x: {
            grid: {
              color: daisyColor('base-content', 10)
            },
            ticks: {
              color: daisyColor('base-content')
            },
            display: false
          },
          y: {
            type: 'linear',
            title: {
              display: true,
              text: 'File System [kb]',
              color: daisyColor('base-content'),
              font: {
                size: 16,
                weight: 'bold'
              }
            },
            position: 'left',
            min: 0,
            max: Math.round($analytics.fs_total[0]),
            grid: { color: daisyColor('base-content', 10) },
            ticks: {
              color: daisyColor('base-content')
            },
            border: { color: daisyColor('base-content', 10) }
          }
        }
      }
    })
    temperatureChart = new Chart(temperatureChartElement, {
      type: 'line',
      data: {
        labels: $analytics.uptime,
        datasets: [
          {
            label: 'Core Temperature',
            borderColor: daisyColor('primary'),
            backgroundColor: daisyColor('primary', 50),
            borderWidth: 2,
            data: $analytics.core_temp,
            yAxisID: 'y'
          }
        ]
      },
      options: {
        maintainAspectRatio: false,
        responsive: true,
        plugins: {
          legend: {
            display: true
          },
          tooltip: {
            mode: 'index',
            intersect: false
          }
        },
        elements: {
          point: {
            radius: 0
          }
        },
        scales: {
          x: {
            grid: {
              color: daisyColor('base-content', 10)
            },
            ticks: {
              color: daisyColor('base-content')
            },
            display: false
          },
          y: {
            type: 'linear',
            title: {
              display: true,
              text: 'Core Temperature [°C]',
              color: daisyColor('base-content'),
              font: {
                size: 16,
                weight: 'bold'
              }
            },
            position: 'left',
            suggestedMin: 20,
            suggestedMax: 100,
            grid: { color: daisyColor('base-content', 10) },
            ticks: {
              color: daisyColor('base-content')
            },
            border: { color: daisyColor('base-content', 10) }
          }
        }
      }
    })
    const interval = setInterval(updateData, 500)
    taskFeed.refresh()
    return () => {
      unsubscribe()
      clearInterval(interval)
      taskFeed.stop()
    }
  })

  function updateData() {
    cpuChart.data.labels = $analytics.uptime
    cpuChart.data.datasets[0].data = $analytics.cpu0_usage
    cpuChart.data.datasets[1].data = $analytics.cpu1_usage
    cpuChart.data.datasets[2].data = $analytics.cpu_usage
    cpuChart.update('none')

    heapChart.data.labels = $analytics.uptime
    heapChart.data.datasets[0].data = $analytics.used_heap
    heapChart.options.scales!.y!.max = Math.ceil($analytics.total_heap[0])
    heapChart.update('none')

    filesystemChart.data.labels = $analytics.uptime
    filesystemChart.data.datasets[0].data = $analytics.fs_used
    heapChart.options.scales!.y!.max = Math.ceil($analytics.fs_total[0])
    filesystemChart.update('none')

    temperatureChart.data.labels = $analytics.uptime
    temperatureChart.data.datasets[0].data = $analytics.core_temp
    temperatureChart.update('none')
  }
</script>

<SettingsCard collapsible={false}>
  {#snippet icon()}
    <Metrics class="lex-shrink-0 mr-2 h-6 w-6 self-end" />
  {/snippet}
  {#snippet title()}
    <span>System Metrics</span>
  {/snippet}

  <div class="w-full overflow-x-auto">
    <div
      class="flex w-full flex-col space-y-1 h-60"
      transition:slide|local={{ duration: 300, easing: cubicOut }}
    >
      <canvas bind:this={cpuChartElement}></canvas>
    </div>
  </div>

  <div class="w-full overflow-x-auto">
    <div
      class="flex w-full flex-col space-y-1 h-60"
      transition:slide|local={{ duration: 300, easing: cubicOut }}
    >
      <canvas bind:this={heapChartElement}></canvas>
    </div>
  </div>
  <div class="w-full overflow-x-auto">
    <div
      class="flex w-full flex-col space-y-1 h-52"
      transition:slide|local={{ duration: 300, easing: cubicOut }}
    >
      <canvas bind:this={filesystemChartElement}></canvas>
    </div>
  </div>
  <div class="w-full overflow-x-auto">
    <div
      class="flex w-full flex-col space-y-1 h-52"
      transition:slide|local={{ duration: 300, easing: cubicOut }}
    >
      <canvas bind:this={temperatureChartElement}></canvas>
    </div>
  </div>
  <div class="w-full overflow-x-auto">
    <table class="table table-sm">
      <thead>
        <tr>
          <th>I<sup>2</sup>C device</th>
          <th>NACKs</th>
          <th>Timeouts</th>
        </tr>
      </thead>
      <tbody>
        {#each $analytics.i2c_errors as device (device.address)}
          <tr>
            <td>0x{device.address.toString(16).padStart(2, '0')}</td>
            <td>{device.nacks}</td>
            <td>{device.timeouts}</td>
          </tr>
        {/each}
      </tbody>
    </table>
    <div class="px-2 text-sm">Bus recoveries: {$analytics.i2c_bus_recoveries}</div>
  </div>
  <div class="w-full overflow-x-auto">
    <div class="flex items-center justify-between gap-2 px-2">
      <label class="label cursor-pointer gap-2 text-sm">
        <input
          type="checkbox"
          class="toggle toggle-sm"
          bind:checked={autoRefresh}
          onchange={() => taskFeed.setAuto(autoRefresh)}
        />
        Refresh every {TASK_STATS_REFRESH_MS / 1000} s
      </label>
      <button class="btn btn-sm" disabled={taskView.loading} onclick={() => taskFeed.refresh()}>
        Refresh
      </button>
    </div>
    <table class="table table-sm">
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
        {#each taskView.stats ? taskRows(taskView.stats) : [] as task (task.key)}
          <tr>
            <td>{task.name}</td>
            <td>{task.stack}</td>
            <td>{task.cpu}</td>
            <td>{task.core}</td>
            <td>{task.priority}</td>
            <td>{task.state}</td>
          </tr>
        {/each}
      </tbody>
    </table>
    {#if taskView.stats}
      <div class="px-2 text-sm">{taskSummary(taskView.stats)}</div>
    {/if}
    {#if taskView.error}
      <div class="px-2 text-sm text-error">Task table failed: {taskView.error}</div>
    {/if}
  </div>
</SettingsCard>
