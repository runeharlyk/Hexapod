<script lang="ts">
  import { connectBluetooth } from '$lib/stores/link'
  import { ble } from '$lib/transport/ble-adapter'
  import { BluetoothConnected, BluetoothDisconnected } from './icons'

  let isConnected = ble.connected
</script>

{#if navigator.bluetooth}
  {#if $isConnected}
    <button
      class="btn btn-ghost btn-circle btn-sm"
      onclick={() => ble.disconnect()}
      aria-label="Disconnect Bluetooth"
      title="Disconnect Bluetooth"
    >
      <BluetoothConnected class="h-6 w-auto text-success" />
    </button>
  {:else}
    <button
      class="btn btn-ghost btn-circle btn-sm"
      onclick={connectBluetooth}
      aria-label="Connect Bluetooth"
      title="Connect Bluetooth"
    >
      <BluetoothDisconnected class="h-6 w-auto text-error" />
    </button>
  {/if}
{/if}
