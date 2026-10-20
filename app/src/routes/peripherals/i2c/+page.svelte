<script lang="ts">
  import RequiresRobot from '$lib/components/RequiresRobot.svelte'
  import PageShell from '$lib/components/layout/PageShell.svelte'
  import I2C from './i2c.svelte'
  import I2CSetting from './i2cSetting.svelte'
</script>

<PageShell>
  <RequiresRobot>
    <I2C />
    <I2CSetting />
  </RequiresRobot>
</PageShell>
