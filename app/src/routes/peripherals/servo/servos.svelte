<script lang="ts">
  import SettingsCard from '$lib/components/SettingsCard.svelte'
  import { MotorOutline } from '$lib/components/icons'
  import { throttler as Throttler } from '$lib/utilities'
  import { dataBroker } from '$lib/transport/databroker'
  import { ServoPWMData, ServoStateData } from '$lib/platform_shared/message'
  import { JOINT_NAMES, LEG_NAMES, servoJoint } from '$lib/calibration/leg-pose'
  import { centreServos } from '$lib/servo-centre'
  import { notifications } from '$lib/components/toasts/notifications'

  let active = $state(false)

  let { pwm = $bindable(), servoId = $bindable() } = $props()

  const throttler = new Throttler()

  const activateServo = () => dataBroker.emit(ServoStateData, { active: true })
  const deactivateServo = () => dataBroker.emit(ServoStateData, { active: false })

  const selected = $derived(servoJoint(servoId))

  // Every servo to its calibrated centre, the pose to line the joints up in; the robot does it only while it idles.
  let centring = $state(false)
  const centreAll = async () => {
    centring = true
    const result = await centreServos()
    centring = false
    if (result.ok) notifications.success('Every servo is at its centre pulse', 3000)
    else notifications.error(`Servos not centred: ${result.error}`, 5000)
  }

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
      <label for="servoId"
        >Servo {servoId}: {LEG_NAMES[selected.leg]}
        {JOINT_NAMES[selected.joint].toLowerCase()}</label
      >
      <input id="servoId" type="range" min="0" max="17" step="1" bind:value={servoId} />
      <input
        type="checkbox"
        class="toggle"
        bind:checked={active}
        onchange={e => (e.currentTarget.checked ? activateServo() : deactivateServo())}
      />
    </span>
  </div>

  <div class="flex flex-wrap items-center gap-2">
    <button class="btn btn-sm" disabled={centring} onclick={centreAll}>Centre all servos</button>
    <span class="text-xs opacity-70">In IDLE: every servo to its calibrated centre pulse</span>
  </div>
</SettingsCard>
