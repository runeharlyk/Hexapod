<script lang="ts">
  import { ble } from '$lib/transport/ble-adapter'
  import SettingsCard from '$lib/components/SettingsCard.svelte'
  import BluetoothIconButton from '$lib/components/BluetoothIconButton.svelte'
  let bleConnected = ble.connected
  let log: string[] = $state([])

  // TODO(proto): this debug subscribe/unsubscribe used the old MsgPack topic API.
  // Re-wire to a real protobuf Message once a debug/telemetry type is defined.
  const subscribe = () => log.push('Subscribe (not wired to protobuf yet)')
  const unsubscribe = () => log.push('Unsubscribe (not wired to protobuf yet)')
</script>

<SettingsCard collapsible={false}>
  {#snippet icon()}
    <BluetoothIconButton />
  {/snippet}
  {#snippet title()}
    <span>Bluetooth</span>
  {/snippet}

  <h2>Bluetooth Settings</h2>

  <div class="my-2 flex gap-4">
    <button class="btn btn-primary" onclick={() => subscribe()}>Subscribe</button>
    <button class="btn btn-primary" onclick={() => unsubscribe()}>Unsubscribe</button>
  </div>
</SettingsCard>

<div class="w-full h-96">
  <textarea class="w-full h-full rounded-md bg-gray-100 p-2 text-xs text-gray-500"
    >{log.join('\n')}</textarea
  >
</div>
