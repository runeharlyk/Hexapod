<script lang="ts">
    import { ble } from '$lib/transport/ble-adapter';
    import SettingsCard from '$lib/components/SettingsCard.svelte';
    import BluetoothIconButton from '$lib/components/input/BluetoothIconButton.svelte';
    import { onMount } from 'svelte';
    import { dataBroker } from '$lib/transport/databroker';
    import { MessageTopic, type Temp } from '$lib/interfaces/transport.interface';

    let value = $state(0);
    let bleConnected = ble.connected;
    let log: string[] = $state([]);
    let subscriptionId = '';

    const handleTemp = (data: Temp) => {
        console.log('temp', data);
        log.push('rx: ' + JSON.stringify(data));
    };

    const publish = () => {
        const data = { value: 110 };
        log.push('Publish: ' + JSON.stringify(data));
        dataBroker.emit(MessageTopic.TEMP, data, subscriptionId);
    };

    const subscribe = () => {
        log.push('Subscribe (temp)');
        subscriptionId = dataBroker.on<Temp>(MessageTopic.TEMP, handleTemp);
    };

    const unsubscribe = () => {
        log.push('Unsubscribe (temp)');
        dataBroker.off(subscriptionId);
        subscriptionId = '';
    };
</script>

<SettingsCard collapsible={false}>
    {#snippet icon()}
        <BluetoothIconButton />
    {/snippet}
    {#snippet title()}
        <span>Bluetooth</span>
    {/snippet}

    <h2>Bluetooth Settings</h2>

    <!-- <div class="flex">
        <label class="label w-32" for="server">Service UUID: </label>
        <div>{SERVICE_UUID}</div>
    </div> -->

    <div class="my-2 flex gap-4">
        <button class="btn btn-primary" onclick={() => subscribe()}>Subscribe</button>
        <button class="btn btn-primary" onclick={() => unsubscribe()}>Unsubscribe</button>
        <button class="btn btn-primary" onclick={() => publish()}>Publish</button>
    </div>

    <hr />

    <div class="my-2 flex gap-4">
        <input class="input" type="number" bind:value />
        <button class="btn btn-primary" onclick={() => ble.send([2, 1, { value }])}>Send</button>
    </div>
</SettingsCard>

<div class="w-full h-96">
    <textarea class="w-full h-full rounded-md bg-gray-100 p-2 text-xs text-gray-500"
        >{log.join('\n')}</textarea
    >
</div>
