<script lang="ts">
    import { bluetooth } from '$lib/stores/bluetooth';
    import { BluetoothConnected, BluetoothDisconnected } from '../icons';
</script>

{#if $bluetooth}
    <button
        class="btn btn-ghost btn-circle btn-sm"
        onclick={() => bluetooth.disconnect()}
        title="Disconnect Bluetooth"
    >
        <BluetoothConnected class="h-6 w-auto text-success" />
    </button>
{:else}
    <button
        class="btn btn-ghost btn-circle btn-sm"
        onclick={() => bluetooth.init()}
        title="Connect Bluetooth"
    >
        <BluetoothDisconnected class="h-6 w-auto text-error" />
    </button>
{/if}
