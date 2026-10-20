<script lang="ts">
  import RequiresRobot from '$lib/components/RequiresRobot.svelte'
  import PageShell from '$lib/components/layout/PageShell.svelte'
  import SystemStatus from './SystemStatus.svelte'
  import LastCrash from './LastCrash.svelte'
</script>

<PageShell width="wide" grouping="cards">
  <RequiresRobot>
    <SystemStatus />
    <LastCrash />
  </RequiresRobot>
</PageShell>
