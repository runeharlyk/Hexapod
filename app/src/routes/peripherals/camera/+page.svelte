<script lang="ts">
  import RequiresRobot from '$lib/components/RequiresRobot.svelte'
  import PageShell from '$lib/components/layout/PageShell.svelte'
  import Camera from './Camera.svelte'
</script>

<PageShell>
  <RequiresRobot>
    <Camera />
  </RequiresRobot>
</PageShell>
