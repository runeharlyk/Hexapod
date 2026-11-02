<script lang="ts">
  import { RotateCcw, RotateCw, Settings as SettingsIcon } from '$lib/components/icons'
  import LegDiagram from '$lib/components/LegDiagram.svelte'
  import { COXA_SAFE_PULSE, coxaHoldsZero, coxaPulseAtZero } from '$lib/calibration/leg-pose'
  import Section from '$lib/components/layout/Section.svelte'
  import SettingsForm from '$lib/components/layout/SettingsForm.svelte'
  import ChangedNote from '$lib/components/layout/ChangedNote.svelte'
  import { Settings } from '$lib/components/layout/settings.svelte'
  import { topicSource } from '$lib/robot/settings-sources'
  import { ServoSettings, type Servo } from '$lib/platform_shared/api'
  import { shortestFloat32 } from '$lib/utilities'
  import { derived } from 'svelte/store'

  let { pwm = $bindable(306), servoId = $bindable(0) } = $props()

  // The calibration crosses the link as float32, which reads back as 2.200000047683716 for a stored 2.2; the fields show
  // the shortest decimal of the same float32, which the robot stores unchanged.
  const shown = (calibration: ServoSettings): ServoSettings => ({
    servos: calibration.servos.map(servo => ({
      ...servo,
      centerPwm: shortestFloat32(servo.centerPwm),
      direction: shortestFloat32(servo.direction),
      centerAngle: shortestFloat32(servo.centerAngle),
      conversion: shortestFloat32(servo.conversion)
    }))
  })

  // The robot refuses the whole calibration with a 400 when one servo is out of range, and keeps the one it had.
  const source = topicSource(ServoSettings, {
    what: 'the servo calibration',
    saved: 'Servo calibration saved.',
    update: value => ({ servoSettingsUpdate: value }),
    together: []
  })
  const settings = new Settings<ServoSettings>({
    ...source,
    watched: derived(source.watched, state => ({
      ...state,
      value: state.value && shown(state.value)
    }))
  })

  // The robot takes a conversion above 0, which no number input's min can say, so the field says it itself.
  const aboveZero = (input: HTMLInputElement, conversion: number) => {
    const check = (value: number) => input.setCustomValidity(value > 0 ? '' : 'Above 0')
    check(conversion)
    return { update: check }
  }

  // The robot refuses a coxa whose travel misses angle 0 (Servo_isValid), so the coxa's centre angle field says so
  // itself, whichever of the servo's values put angle 0 outside the safe pulses.
  const holdsZero = (input: HTMLInputElement, servo: Servo | null) => {
    const check = (servo: Servo | null) =>
      input.setCustomValidity(
        !servo || coxaHoldsZero(servo) ?
          ''
        : `Angle 0 at ${Math.round(coxaPulseAtZero(servo))}, outside ${COXA_SAFE_PULSE.join(' to ')}`
      )
    check(servo)
    return { update: check }
  }

  const setServoCenter = () => {
    const servo = settings.value.servos[servoId]
    if (servo) servo.centerPwm = pwm
  }
</script>

{#if settings.status === 'ready' && settings.value.servos.length === 18}
  <Section title="Reference pose">
    <LegDiagram servos={settings.value.servos} {servoId} {pwm} />
  </Section>
{/if}

<SettingsForm
  title="Calibration"
  description="Each servo's centre pulse, the way it turns, its angle at the centre and its pulses per degree."
  icon={SettingsIcon}
  {settings}
  rows={6}
>
  {#snippet actions()}
    <button type="button" class="btn btn-sm" disabled={settings.pending} onclick={setServoCenter}>
      Set servo {servoId} centre to {pwm}
    </button>
  {/snippet}
  {#snippet children(form, notes)}
    <div class="overflow-x-auto">
      <table class="table-sm table">
        <thead>
          <tr>
            <th>Servo</th>
            <th>Centre PWM</th>
            <th>Direction</th>
            <th>Centre angle</th>
            <th>Conversion</th>
          </tr>
        </thead>
        <tbody>
          {#each form.servos as servo, index (index)}
            <tr class={index === servoId ? 'bg-base-200' : ''}>
              <td class="whitespace-nowrap font-medium">{servo.name || `Servo ${index}`}</td>
              <td>
                <input
                  type="number"
                  class="input input-sm validator w-20"
                  name="servos.{index}.centerPwm"
                  aria-label="Servo {index} centre PWM"
                  aria-describedby="servo-limits"
                  title={index % 3 === 0 ? COXA_SAFE_PULSE.join(' to ') : '80 to 600'}
                  required
                  min={index % 3 === 0 ? COXA_SAFE_PULSE[0] : 80}
                  max={index % 3 === 0 ? COXA_SAFE_PULSE[1] : 600}
                  step="any"
                  bind:value={servo.centerPwm}
                />
                <ChangedNote {notes} field="servos.{index}.centerPwm" />
              </td>
              <td>
                <button
                  type="button"
                  class="btn btn-sm btn-ghost btn-square"
                  name="servos.{index}.direction"
                  title="Toggle direction ({servo.direction})"
                  aria-label="Servo {index} direction {servo.direction}"
                  onclick={() => (servo.direction = servo.direction === 1 ? -1 : 1)}
                >
                  {#if servo.direction === 1}
                    <RotateCw class="text-success size-4" />
                  {:else}
                    <RotateCcw class="size-4" />
                  {/if}
                </button>
                <ChangedNote {notes} field="servos.{index}.direction" />
              </td>
              <td>
                <input
                  type="number"
                  class="input input-sm validator w-20"
                  name="servos.{index}.centerAngle"
                  aria-label="Servo {index} centre angle"
                  aria-describedby="servo-limits"
                  title="-180 to 180"
                  required
                  min="-180"
                  max="180"
                  step="any"
                  bind:value={servo.centerAngle}
                  use:holdsZero={index % 3 === 0 ? { ...servo } : null}
                />
                <ChangedNote {notes} field="servos.{index}.centerAngle" />
              </td>
              <td>
                <input
                  type="number"
                  class="input input-sm validator w-20"
                  name="servos.{index}.conversion"
                  aria-label="Servo {index} conversion"
                  aria-describedby="servo-limits"
                  title="Above 0, up to 100"
                  required
                  max="100"
                  step="any"
                  bind:value={servo.conversion}
                  use:aboveZero={servo.conversion}
                />
                <ChangedNote {notes} field="servos.{index}.conversion" />
              </td>
            </tr>
          {/each}
        </tbody>
      </table>
    </div>
    <p id="servo-limits" class="text-base-content/70 text-xs">
      Centre PWM 80 to 600: the app's own guard, narrower than the 0 to 4095 the robot accepts; a
      coxa's 103 to 512, the servo's safe 500 to 2500 us, which the robot holds it to, at its centre
      and at angle 0. Centre angle -180 to 180. Conversion above 0, up to 100 pulses per degree.
    </p>
  {/snippet}
</SettingsForm>
