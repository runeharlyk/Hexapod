<script lang="ts" module>
  import { createTelemetryRecorder } from '$lib/stores/telemetry'
  import { dataBroker } from '$lib/transport/databroker'
  import { isLinked } from '$lib/stores/link'

  // Module-level, so a recording keeps going while the user visits other pages.
  const recorder = createTelemetryRecorder({ broker: dataBroker, linked: isLinked })
</script>

<script lang="ts">
  import Card from '$lib/components/layout/Card.svelte'
  import { DownloadIcon, Record } from '$lib/components/icons'
  import { CONTROL_PERIOD_US } from '$lib/recording'
  import { humanFileSize } from '$lib/utilities/string-utilities'

  const timing = $derived($recorder.finished?.timing)
  const ms = (us: number) => (Number.isNaN(us) ? '-' : (us / 1000).toFixed(2))
</script>

<Card title="Telemetry recording" icon={Record}>
  <div class="flex flex-wrap items-center gap-2">
    {#if $recorder.recording}
      <button class="btn btn-error btn-sm" onclick={recorder.stop}>Stop</button>
      <span class="text-sm tabular-nums">
        {humanFileSize($recorder.bytes)}, {$recorder.frames} batches{$recorder.gaps ?
          `, ${$recorder.gaps} link gaps`
        : ''}
      </span>
    {:else}
      <button
        class="btn btn-primary btn-sm"
        disabled={$recorder.starting || !$isLinked}
        onclick={recorder.start}
      >
        Record
      </button>
    {/if}
    {#if $recorder.finished && !$recorder.recording}
      <button class="btn btn-sm" onclick={recorder.download} aria-label="Download recording">
        <DownloadIcon class="size-4" />
        {$recorder.finished.filename} ({humanFileSize($recorder.finished.blob.size)})
      </button>
    {/if}
  </div>

  {#if $recorder.error}
    <div role="alert" class="alert alert-error alert-soft">{$recorder.error}</div>
  {/if}

  {#if timing && !$recorder.recording}
    <table class="table-sm bg-base-100 rounded-box table w-full tabular-nums">
      <tbody>
        <tr><td>Ticks</td><td class="text-right">{timing.ticks}</td></tr>
        <tr>
          <td>Control period p50 / p99 (nominal {ms(CONTROL_PERIOD_US)})</td>
          <td class="text-right">{ms(timing.periodP50Us)} / {ms(timing.periodP99Us)} ms</td>
        </tr>
        <tr>
          <td>Control period mean</td>
          <td class="text-right">{ms(timing.periodMeanUs)} ms</td>
        </tr>
        <tr><td>Overruns</td><td class="text-right">{timing.overruns}</td></tr>
        <tr><td>Compute p99</td><td class="text-right">{ms(timing.computeP99Us)} ms</td></tr>
      </tbody>
    </table>
  {/if}
</Card>
