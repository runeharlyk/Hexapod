<script lang="ts">
  import { MotorOutline } from '$lib/components/icons'
  import Field from '$lib/components/layout/Field.svelte'
  import Section from '$lib/components/layout/Section.svelte'
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

<Section
  title="Drive a servo"
  description="Moves one servo to a pulse while it is powered, to find its centre."
  icon={MotorOutline}
>
  {#snippet footer()}
    <span class="text-base-content/70 self-center text-xs">
      In IDLE: every servo to its calibrated centre pulse
    </span>
    <button class="btn" disabled={centring} onclick={centreAll}>Centre all servos</button>
  {/snippet}
  <div class="grid gap-4 text-sm">
    <Field label="Servo" for="servo-id">
      <input
        id="servo-id"
        class="range range-sm w-full"
        type="range"
        min="0"
        max="17"
        step="1"
        bind:value={servoId}
      />
      <p class="mt-1 text-xs">
        {servoId}: {LEG_NAMES[selected.leg]}
        {JOINT_NAMES[selected.joint].toLowerCase()}
      </p>
    </Field>
    <Field label="Pulse" for="servo-pwm">
      <input
        id="servo-pwm"
        class="range range-sm w-full"
        type="range"
        min="80"
        max="600"
        bind:value={pwm}
        oninput={updatePWM}
      />
      <p class="mt-1 font-mono text-xs">{pwm}</p>
    </Field>
    <Field label="Powered" for="servo-active">
      <input
        id="servo-active"
        type="checkbox"
        class="toggle"
        bind:checked={active}
        onchange={e => (e.currentTarget.checked ? activateServo() : deactivateServo())}
      />
    </Field>
  </div>
</Section>
