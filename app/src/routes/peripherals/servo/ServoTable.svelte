<script lang="ts">
  import { onMount } from 'svelte'
  import { RotateCcw, RotateCw } from '$lib/components/icons'
  import { dataBroker } from '$lib/transport/databroker'
  import { notifications } from '$lib/components/toasts/notifications'
  import type { Servo } from '$lib/platform_shared/api'

  let { pwm = $bindable(306), servoId = $bindable(0) } = $props()

  let servos: Servo[] = $state([])

  const load = async () => {
    const res = await dataBroker.request({ servoSettingsGet: {} }).catch(() => null)
    servos = res?.servoSettings?.servos ?? []
  }
  onMount(load)

  const save = async () => {
    try {
      await dataBroker.request({ servoSettingsUpdate: { servos } })
      notifications.success('Servo calibration saved.', 3000)
    } catch {
      notifications.error('Failed to save servo calibration.', 3000)
    }
  }

  const setServoCenter = () => {
    if (servos[servoId]) servos[servoId].centerPwm = pwm
  }

  const toggleDirection = (index: number) =>
    (servos[index].direction = servos[index].direction === 1 ? -1 : 1)

  type NumericServoField = 'centerPwm' | 'centerAngle' | 'conversion'

  const updateValue = (event: Event, index: number, key: NumericServoField) =>
    (servos[index][key] = Number((event.target as HTMLInputElement).value))
</script>

<div class="overflow-x-auto space-y-2">
  <div class="flex flex-wrap gap-2">
    <button class="btn btn-sm" onclick={setServoCenter}>Set servo {servoId} center to {pwm}</button>
    <button class="btn btn-sm btn-primary" onclick={save}>Save calibration</button>
  </div>
  <table class="table table-xs">
    <thead>
      <tr>
        <th>Servo</th>
        <th>Center PWM</th>
        <th>Direction</th>
        <th>Center angle</th>
        <th>Conversion</th>
      </tr>
    </thead>
    <tbody>
      {#each servos as servo, index}
        <tr class="hover:bg-base-200">
          <td class="font-medium">{servo.name || `Servo ${index}`}</td>
          <td>
            <input
              type="number"
              class="input input-sm input-bordered w-20"
              value={servo.centerPwm}
              oninput={event => updateValue(event, index, 'centerPwm')}
              min="80"
              max="600"
            />
          </td>
          <td>
            <button
              class="btn btn-sm btn-ghost"
              title="Toggle direction ({servo.direction})"
              onclick={() => toggleDirection(index)}
            >
              {#if servo.direction === 1}
                <RotateCw class="w-4 h-4 text-green-500" />
              {:else}
                <RotateCcw class="w-4 h-4" />
              {/if}
            </button>
          </td>
          <td>
            <input
              type="number"
              step="1"
              class="input input-sm input-bordered w-20"
              value={servo.centerAngle}
              oninput={event => updateValue(event, index, 'centerAngle')}
            />
          </td>
          <td>
            <input
              type="number"
              step="0.01"
              class="input input-sm input-bordered w-20"
              value={servo.conversion}
              oninput={event => updateValue(event, index, 'conversion')}
              min="0"
              max="10"
            />
          </td>
        </tr>
      {/each}
    </tbody>
  </table>
</div>
