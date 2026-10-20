<script lang="ts">
  import RequiresRobot from '$lib/components/RequiresRobot.svelte'
  import RequiresFeatures from '$lib/components/RequiresFeatures.svelte'
  import PageShell from '$lib/components/layout/PageShell.svelte'
  import SystemMetrics from './SystemMetrics.svelte'
  import TelemetryRecording from './TelemetryRecording.svelte'
</script>

<PageShell width="wide" grouping="cards">
  <RequiresRobot>
    <RequiresFeatures
      has={features => features.analytics}
      absent="This robot's firmware reports no analytics."
    >
      <SystemMetrics />
      <TelemetryRecording />
    </RequiresFeatures>
  </RequiresRobot>
</PageShell>
