<script lang="ts">
  import RequiresRobot from '$lib/components/RequiresRobot.svelte'
  import PageShell from '$lib/components/layout/PageShell.svelte'
  import Servos from './servos.svelte'
  import ServoTable from './ServoTable.svelte'

  let servoId = $state(0)
  let pwm = $state(306)
</script>

<PageShell>
  <RequiresRobot>
    <Servos bind:servoId bind:pwm />
    <ServoTable bind:servoId bind:pwm />
  </RequiresRobot>
</PageShell>
