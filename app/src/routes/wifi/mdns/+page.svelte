<script lang="ts">
  import RequiresRobot from '$lib/components/RequiresRobot.svelte'
  import PageShell from '$lib/components/layout/PageShell.svelte'
  import MDNS from './MDNS.svelte'
</script>

<PageShell>
  <RequiresRobot>
    <MDNS />
  </RequiresRobot>
</PageShell>
