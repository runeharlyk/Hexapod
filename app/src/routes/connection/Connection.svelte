<script lang="ts">
    import { connect, disconnect, isConnected, send, SERVICE_UUID } from '$lib/bluetooth';
    import SettingsCard from '$lib/components/SettingsCard.svelte';
    import { BluetoothConnected, BluetoothDisconnected, WiFi } from '$lib/components/icons';
    import { location, socket } from '$lib/stores';

    const update = () => {
        const ws = $location ? $location : window.location.host;
        socket.init(`ws://${ws}/api/ws/events`);
    };
</script>

<SettingsCard collapsible={false}>
    {#snippet icon()}
        <WiFi class="lex-shrink-0 mr-2 h-6 w-6 self-end" />
    {/snippet}
    {#snippet title()}
        <span>Connection</span>
    {/snippet}

    <h2>WebSocket</h2>

    <div class="flex">
        <label class="label w-32" for="server">Address:</label>
        <input class="input" bind:value={$location} />
    </div>

    <button class="btn btn-primary" onclick={update}>Update</button>

    <h2>Bluetooth</h2>

    <div class="flex">
        <label class="label w-32" for="server">Service UUID: </label>
        <div>{SERVICE_UUID}</div>
        {#if $isConnected}
            <button
                class="btn btn-ghost btn-circle btn-sm"
                onclick={() => disconnect()}
                title="Disconnect Bluetooth"
            >
                <BluetoothConnected class="h-6 w-auto text-success" />
            </button>
        {:else}
            <button
                class="btn btn-ghost btn-circle btn-sm"
                onclick={() => connect()}
                title="Connect Bluetooth"
            >
                <BluetoothDisconnected class="h-6 w-auto text-error" />
            </button>
        {/if}
    </div>

    <button class="btn btn-primary" onclick={() => send('hello')}>Send</button>
</SettingsCard>
