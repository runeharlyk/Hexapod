<script lang="ts">
  import RequiresRobot from '$lib/components/RequiresRobot.svelte'
  import RequiresFeatures from '$lib/components/RequiresFeatures.svelte'
  import SystemMetrics from './SystemMetrics.svelte'
  import TelemetryRecording from './TelemetryRecording.svelte'
</script>

<div class="mx-0 my-1 flex flex-col space-y-4 sm:mx-8 sm:my-8">
  <RequiresRobot>
    <RequiresFeatures
      has={features => features.analytics}
      absent="This robot's firmware reports no analytics."
    >
      <SystemMetrics />
      <TelemetryRecording />
    </RequiresFeatures>
  </RequiresRobot>
</div>
