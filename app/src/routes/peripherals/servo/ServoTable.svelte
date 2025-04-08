<script lang="ts">
    import { api } from '$lib/api';
    import { onMount } from 'svelte';
    interface Props {
        data?: any;
        pwm: number;
        servoId: number;
    }

    let {
        data = $bindable({
            servos: []
        }),
        pwm = $bindable(306),
        servoId = $bindable(0)
    }: Props = $props();

    const updateValue = (event, index, key) => {
        data.servos[index][key] = event.target.innerText;
    };

    const syncConfig = async () => {
        await api.post('/api/servo/config', data);
        console.log(data);
    };

    onMount(async () => {
        const result = await api.get('/api/servo/config');
        if (result.isOk()) {
            data = result.inner;
        }
    });
    const setServoCenter = () => {
        data.servos[servoId]['center_pwm'] = pwm;
    };
</script>

<div class="overflow-x-auto">
    <button class="btn" onclick={setServoCenter}>Save servo center pwm</button>
    <table class="table table-xs">
        <thead>
            <tr>
                <!-- <th>Name</th> -->
                <th>Center PWM</th>
                <th>Pin</th>
                <th>Direction</th>
                <th>Conversion</th>
            </tr>
        </thead>
        <tbody>
            {#each data.servos as servo, index}
                <tr>
                    <!-- <td
                        contenteditable="true"
                        onblur={syncConfig}
                        oninput={event => updateValue(event, index, 'name')}
                    >
                        {servo.name}
                    </td> -->
                    <td
                        contenteditable="true"
                        onblur={syncConfig}
                        oninput={event => updateValue(event, index, 'center_pwm')}
                    >
                        {servo.center_pwm}
                    </td>
                    <td
                        contenteditable="true"
                        onblur={syncConfig}
                        oninput={event => updateValue(event, index, 'pin')}
                    >
                        {servo.pin}
                    </td>
                    <td
                        contenteditable="true"
                        onblur={syncConfig}
                        oninput={event => updateValue(event, index, 'direction')}
                    >
                        {servo.direction}
                    </td>
                    <td
                        contenteditable="true"
                        onblur={syncConfig}
                        oninput={event => updateValue(event, index, 'conversion')}
                    >
                        {servo.conversion}
                    </td>
                </tr>
            {/each}
        </tbody>
    </table>
</div>
