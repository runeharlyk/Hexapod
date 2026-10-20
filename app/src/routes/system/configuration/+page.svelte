<script lang="ts">
  import RequiresRobot from '$lib/components/RequiresRobot.svelte'
  import PageShell from '$lib/components/layout/PageShell.svelte'
  import Configuration from './Configuration.svelte'
</script>

<PageShell>
  <RequiresRobot>
    <Configuration />
  </RequiresRobot>
</PageShell>
