<script lang="ts">
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

  onMount(async () => dataBroker.on(MessageTopic.SERVO_SETTINGS, d => (data.servos = d)))

  const updateValue = (event: Event, index: number, key: string) =>
    (data.servos[index][key] = event.target?.innerText)

  const syncConfig = async () => dataBroker.emit(MessageTopic.SERVO_SETTINGS, data.servos)

  const setServoCenter = () => (data.servos[servoId]['center'] = pwm)
</script>

<div class="overflow-x-auto">
  <button class="btn" onclick={setServoCenter}>Save servo center pwm</button>
  <table class="table table-xs">
    <thead>
      <tr>
        <th>Name</th>
        <th>Center PWM</th>
        <th>Pin</th>
        <th>Direction</th>
        <th>Conversion</th>
      </tr>
    </thead>
    <tbody>
      {#each data.servos as servo, index}
        <tr>
          <td
            contenteditable="true"
            onblur={syncConfig}
            oninput={event => updateValue(event, index, 'name')}>
            {servo.name}
          </td>
          <td
            contenteditable="true"
            onblur={syncConfig}
            oninput={event => updateValue(event, index, 'center')}>
            {servo.center}
          </td>
          <td
            contenteditable="true"
            onblur={syncConfig}
            oninput={event => updateValue(event, index, 'pin')}>
            {servo.pin}
          </td>
          <td
            contenteditable="true"
            onblur={syncConfig}
            oninput={event => updateValue(event, index, 'dir')}>
            {servo.dir}
          </td>
          <td
            contenteditable="true"
            onblur={syncConfig}
            oninput={event => updateValue(event, index, 'conv')}>
            {servo.conv}
          </td>
        </tr>
      {/each}
    </tbody>
  </table>
</div>
