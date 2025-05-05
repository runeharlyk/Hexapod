<script lang="ts">
    import { isConnected, connect, disconnect } from '$lib/bluetooth';
    import { BluetoothConnected, BluetoothDisconnected } from '../icons';
</script>

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
