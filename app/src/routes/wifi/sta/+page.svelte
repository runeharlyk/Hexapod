<script lang="ts">
  import RequiresRobot from '$lib/components/RequiresRobot.svelte'
  import PageShell from '$lib/components/layout/PageShell.svelte'
  import Wifi from './Wifi.svelte'
</script>

<PageShell>
  <RequiresRobot>
    <Wifi />
  </RequiresRobot>
</PageShell>
