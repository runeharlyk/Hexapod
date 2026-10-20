<script lang="ts">
  import RequiresRobot from '$lib/components/RequiresRobot.svelte'
  import PageShell from '$lib/components/layout/PageShell.svelte'
  import Accesspoint from './Accesspoint.svelte'
</script>

<PageShell>
  <RequiresRobot>
    <Accesspoint />
  </RequiresRobot>
</PageShell>
