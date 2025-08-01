<script lang="ts">
  import SettingsCard from '$lib/components/SettingsCard.svelte'
  import Spinner from '$lib/components/Spinner.svelte'
  import { socket } from '$lib/stores'
  import { throttler as Throttler } from '$lib/utilities'
  import { MotorOutline } from '$lib/components/icons'
  import { dataBroker } from '$lib/transport/databroker'
  import { MessageTopic } from '$lib/interfaces/transport.interface'

  let isLoading = false

  let active = $state(false)

  let { pwm = $bindable(), servoId = $bindable() } = $props()

  const throttler = new Throttler()

  const activateServo = () => {
    socket.sendEvent('servoState', { active: 1 })
  }

  const deactivateServo = () => {
    socket.sendEvent('servoState', { active: 0 })
  }

  const updatePWM = () => {
    throttler.throttle(() => {
      dataBroker.emit(MessageTopic.SERVO, { id: servoId, pwm })
    }, 10)
  }
</script>

<SettingsCard collapsible={false}>
  {#snippet icon()}
    <MotorOutline class="lex-shrink-0 mr-2 h-6 w-6 self-end" />
  {/snippet}
  {#snippet title()}
    <span>Servo</span>
  {/snippet}
  {pwm}
  <input
    type="range"
    min="80"
    max="600"
    bind:value={pwm}
    oninput={updatePWM}
    class="w-full h-2 bg-gray-200 rounded-lg appearance-none cursor-pointer dark:bg-gray-700" />

  {#if isLoading}
    <Spinner />
  {:else}
    <div class="flex flex-col">
      <h2 class="text-lg">General servo configuration</h2>
      <span class="flex items-center gap-2">
        <label for="servoId">Servo active {servoId}</label>
        <input type="range" min="0" max="17" step="1" bind:value={servoId} />
        <input
          type="checkbox"
          class="toggle"
          bind:checked={active}
          onchange={active ? activateServo : deactivateServo} />
      </span>
    </div>
  {/if}
</SettingsCard>
