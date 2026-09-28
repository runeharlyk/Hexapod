<script lang="ts">
  import SettingsCard from '$lib/components/SettingsCard.svelte'
  import { MotorOutline } from '$lib/components/icons'
  import { throttler as Throttler } from '$lib/utilities'
  import { dataBroker } from '$lib/transport/databroker'
  import { ServoPWMData, ServoStateData } from '$lib/platform_shared/message'

  let active = $state(false)

  let { pwm = $bindable(), servoId = $bindable() } = $props()

  const throttler = new Throttler()

  const activateServo = () => dataBroker.emit(ServoStateData, { active: true })
  const deactivateServo = () => dataBroker.emit(ServoStateData, { active: false })

  const updatePWM = () =>
    throttler.throttle(() => dataBroker.emit(ServoPWMData, { servoId, servoPwm: pwm }), 10)
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
    class="w-full h-2 bg-gray-200 rounded-lg appearance-none cursor-pointer dark:bg-gray-700"
  />

  <div class="flex flex-col">
    <h2 class="text-lg">General servo configuration</h2>
    <span class="flex items-center gap-2">
      <label for="servoId">Servo active {servoId}</label>
      <input type="range" min="0" max="17" step="1" bind:value={servoId} />
      <input
        type="checkbox"
        class="toggle"
        bind:checked={active}
        onchange={e => (e.currentTarget.checked ? activateServo() : deactivateServo())}
      />
    </span>
  </div>
</SettingsCard>
