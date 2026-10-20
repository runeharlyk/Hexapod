<script lang="ts">
  import RequiresRobot from '$lib/components/RequiresRobot.svelte'
  import PageShell from '$lib/components/layout/PageShell.svelte'
  import FileSystem from './FileSystem.svelte'
</script>

<PageShell width="wide">
  <RequiresRobot>
    <FileSystem />
  </RequiresRobot>
</PageShell>
