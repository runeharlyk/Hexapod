<script lang="ts">
    import { api } from '$lib/api';
    import { bluetooth, SERVICE_UUID } from '$lib/stores/bluetooth';
    import SettingsCard from '$lib/components/SettingsCard.svelte';
    import { BluetoothConnected, BluetoothDisconnected, WiFi } from '$lib/components/icons';
    import BluetoothIconButton from '$lib/components/input/BluetoothIconButton.svelte';
    import { onMount } from 'svelte';

    type BluetoothSettings = { bluetooth: { device_name: string } };

    let bluetoothSettings = $state<BluetoothSettings>({
        bluetooth: { device_name: '' }
    });

    const getBluetoothSettings = async () => {
        const result = await api.get<BluetoothSettings>('/api/bluetooth/settings');
        if (result.isOk()) {
            bluetoothSettings = result.inner;
            console.log(bluetoothSettings);
        }
    };

    const updateBluetoothSettings = async () => {
        const result = await api.post<BluetoothSettings>(
            '/api/bluetooth/settings',
            bluetoothSettings
        );
        if (result.isOk()) {
            bluetoothSettings = result.inner;
        }
    };

    onMount(() => {
        getBluetoothSettings();

        bluetooth.on('telemetry', data => {
            console.log('Telemetry:', data);
        });
    });

    const testBluetooth = async () => {
        bluetooth.sendEvent('test', { message: 'hello' });
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

    <div class="flex">
        <label class="label w-32" for="server">Device Name:</label>
        <input class="input" bind:value={bluetoothSettings.bluetooth.device_name} />
    </div>

    <div class="flex">
        <label class="label w-32" for="server">Service UUID: </label>
        <div>{SERVICE_UUID}</div>
    </div>

    <div class="flex gap-2">
        <button class="btn btn-primary" onclick={updateBluetoothSettings}>Update</button>
        <button class="btn btn-accent" onclick={testBluetooth}>Test connection</button>
    </div>
</SettingsCard>
