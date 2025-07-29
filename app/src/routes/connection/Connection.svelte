<script lang="ts">
  import SettingsCard from '$lib/components/SettingsCard.svelte'
  import { BluetoothConnected, BluetoothDisconnected, WiFi } from '$lib/components/icons'
  import { location } from '$lib/stores'
  import { ble, SERVICE_UUID } from '$lib/transport/ble-adapter'
  import { websocket } from '$lib/transport/websocket-adapter'

  const update = () => {
    websocket.connect()
  }

  let isConnected = ble.connected
  let isWebSocketConnected = websocket.connected
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
    <label class="label w-32" for="server">Status:</label>
    <div>{$isWebSocketConnected ? 'Connected' : 'Disconnected'}</div>
    {#if $isWebSocketConnected}
      <button
        class="btn btn-ghost btn-circle btn-sm"
        onclick={() => websocket.disconnect()}
        title="Disconnect WebSocket">
        <WiFi class="h-6 w-auto text-success" />
      </button>
    {:else}
      <button
        class="btn btn-ghost btn-circle btn-sm"
        onclick={() => websocket.connect()}
        title="Connect WebSocket">
        <WiFi class="h-6 w-auto text-error" />
      </button>
    {/if}
  </div>

  <button class="btn btn-primary" onclick={update}>Connect</button>

  <h2>Bluetooth</h2>

  <div class="flex">
    <label class="label w-32" for="server">Service UUID: </label>
    <div>{SERVICE_UUID}</div>
    {#if $isConnected}
      <button
        class="btn btn-ghost btn-circle btn-sm"
        onclick={() => ble.disconnect()}
        title="Disconnect Bluetooth">
        <BluetoothConnected class="h-6 w-auto text-success" />
      </button>
    {:else}
      <button
        class="btn btn-ghost btn-circle btn-sm"
        onclick={() => ble.connect()}
        title="Connect Bluetooth">
        <BluetoothDisconnected class="h-6 w-auto text-error" />
      </button>
    {/if}
  </div>

  <button class="btn btn-primary" onclick={() => ble.send('hello')}>Send</button>
</SettingsCard>
