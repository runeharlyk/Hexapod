<script lang="ts">
  import { RotateCcw, RotateCw } from '$lib/components/icons'
  import { MessageTopic } from '$lib/interfaces/transport.interface'
  import { dataBroker } from '$lib/transport/databroker'
  import { onMount } from 'svelte'

  interface Props {
    data?: any
    pwm: number
    servoId: number
  }

  let {
    data = $bindable({
      servos: []
    }),
    pwm = $bindable(306),
    servoId = $bindable(0)
  }: Props = $props()

  onMount(async () =>
    dataBroker.on(MessageTopic.SERVO_SETTINGS, d => {
      console.log(d)
      data.servos = d
    })
  )

  const updateValue = (event: Event, index: number, key: string) =>
    (data.servos[index][key] = Number((event.target as HTMLInputElement).value))

  const syncConfig = () => dataBroker.emit(MessageTopic.SERVO_SETTINGS, data.servos)

  const setServoCenter = () => {
    data.servos[servoId]['center'] = pwm
    syncConfig()
  }

  const toggleDirection = (index: number) => {
    data.servos[index].dir = data.servos[index].dir === 1 ? -1 : 1
    syncConfig()
  }
</script>

<div class="overflow-x-auto">
  <button class="btn" onclick={setServoCenter}>Save servo center pwm</button>
  <table class="table table-xs">
    <thead>
      <tr>
        <th>Servo</th>
        <th>Center PWM</th>
        <th>Pin</th>
        <th>Direction</th>
        <th>Conversion</th>
      </tr>
    </thead>
    <tbody>
      {#each data.servos as servo, index}
        <tr class="hover:bg-base-200">
          <td class="font-medium">Servo {index}</td>
          <td>
            <input
              type="number"
              class="input input-sm input-bordered w-20"
              value={servo.center}
              onblur={syncConfig}
              oninput={event => updateValue(event, index, 'center')}
              min="80"
              max="600"
            />
          </td>
          <td>
            <input
              type="number"
              step="1"
              class="input input-sm input-bordered w-20"
              value={servo.pin}
              onblur={syncConfig}
              oninput={event => updateValue(event, index, 'pin')}
              min="0"
              max="32"
            />
          </td>
          <td>
            <button
              class="btn btn-sm btn-ghost"
              title="Toggle direction {servo.dir}"
              onclick={() => toggleDirection(index)}
            >
              {#if servo.dir === 1}
                <RotateCw class="w-4 h-4 text-green-500" />
              {:else}
                <RotateCcw class="w-4 h-4" />
              {/if}
            </button>
          </td>
          <td>
            <input
              type="number"
              step="0.01"
              class="input input-sm input-bordered w-20"
              value={servo.conv}
              onblur={syncConfig}
              oninput={event => updateValue(event, index, 'conv')}
              min="0"
              max="10"
            />
          </td>
        </tr>
      {/each}
    </tbody>
  </table>
</div>
