<script lang="ts" module>
  export interface Series {
    label: string
    colour: 'primary' | 'secondary' | 'accent'
    data: number[]
    fill?: boolean
  }

  /** The y axis: fixed bounds, bounds the data may push out, or whole units with one to spare around the data. */
  export type Range =
    { min?: number; max?: number; suggestedMin?: number; suggestedMax?: number } | 'fit'
</script>

<script lang="ts">
  // The one line chart of the IMU and metrics pages: a fixed-height container, theme colours, no x axis.
  import { onDestroy } from 'svelte'
  import { on } from 'svelte/events'
  import { Chart, registerables } from 'chart.js'
  import { daisyColor } from '$lib/utilities'

  Chart.register(...registerables)

  interface Props {
    /** The y axis's title, with its unit. */
    unit: string
    series: Series[]
    /** One per sample; the samples' positions when left out. The x axis is not drawn. */
    labels?: number[]
    range?: Range
    /** At most one redraw per this many milliseconds, however often the data changes. */
    every?: number
  }

  const { unit, series, labels, range = {}, every = 500 }: Props = $props()

  let canvas: HTMLCanvasElement
  let chart: Chart<'line'> | undefined
  let pending: ReturnType<typeof setTimeout> | undefined

  const bounds = () => {
    if (range !== 'fit') return range
    const values = series.flatMap(one => one.data)
    return values.length ?
        { min: Math.floor(Math.min(...values)) - 1, max: Math.ceil(Math.max(...values)) + 1 }
      : {}
  }

  const create = () =>
    paint(
      new Chart(canvas, {
        type: 'line',
        data: {
          labels: [],
          datasets: series.map(one => ({
            label: one.label,
            borderWidth: 2,
            fill: one.fill ?? false,
            data: []
          }))
        },
        options: {
          maintainAspectRatio: false,
          responsive: true,
          animation: false,
          plugins: {
            legend: { display: true, labels: {} },
            tooltip: { mode: 'index', intersect: false }
          },
          elements: { point: { radius: 0 } },
          scales: {
            x: { display: false },
            y: {
              type: 'linear',
              position: 'left',
              title: { display: true, text: unit, font: { weight: 'bold' } },
              grid: {},
              ticks: {},
              border: {}
            }
          }
        }
      })
    )

  // The theme's colours, read again whenever the theme changes.
  const paint = (chart: Chart<'line'>) => {
    const text = daisyColor('base-content')
    const grid = daisyColor('base-content', 10)
    chart.data.datasets.forEach((dataset, index) => {
      dataset.borderColor = daisyColor(series[index].colour)
      dataset.backgroundColor = daisyColor(series[index].colour, 50)
    })
    chart.options.plugins!.legend!.labels!.color = text
    const y = chart.options.scales!.y!
    y.title!.color = text
    y.grid!.color = grid
    y.ticks!.color = text
    y.border!.color = grid
    return chart
  }

  const repaint = () => {
    if (chart) paint(chart).update('none')
  }

  // The menu's theme switch is a daisyUI theme-controller checkbox; without it the theme follows the system.
  $effect(() => {
    const scheme = window.matchMedia('(prefers-color-scheme: dark)')
    const stopScheme = on(scheme, 'change', repaint)
    const stopSwitch = on(document, 'change', event => {
      if ((event.target as Element).matches('.theme-controller')) repaint()
    })
    return () => {
      stopScheme()
      stopSwitch()
    }
  })

  const draw = () => {
    pending = undefined
    chart ??= create()
    chart.data.labels = labels ?? series[0]?.data.map((_, index) => index) ?? []
    series.forEach((one, index) => (chart!.data.datasets[index].data = one.data))
    chart.options.scales!.y = { ...chart.options.scales!.y, ...bounds() }
    chart.update('none')
  }

  $effect(() => {
    // Reading the props here redraws on their change, at most once per `every`.
    void [series.map(one => one.data), labels, range]
    pending ??= setTimeout(draw, chart ? every : 0)
  })

  onDestroy(() => {
    clearTimeout(pending)
    chart?.destroy()
  })
</script>

<div class="relative h-56 w-full">
  <canvas bind:this={canvas}></canvas>
</div>
