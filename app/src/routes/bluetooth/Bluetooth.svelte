<script lang="ts">
  import BluetoothIconButton from '$lib/components/BluetoothIconButton.svelte'
  import StatusItem from '$lib/components/StatusItem.svelte'
  import Section from '$lib/components/layout/Section.svelte'
  import { Bluetooth, BluetoothConnected, Health, Remote, Router } from '$lib/components/icons'
  import { ble, SERVICE_UUID } from '$lib/transport/ble-adapter'
  import { dataBroker } from '$lib/transport/databroker'

  const status = ble.status
  const deviceName = ble.deviceName
  const latency = dataBroker.latencyMs

  // Web Bluetooth exposes no MTU, so the frame size the adapter chunks to is the honest number here.
  const CHUNK_BYTES = 180
</script>

<Section title="Bluetooth link" icon={Bluetooth}>
  {#snippet actions()}
    <BluetoothIconButton />
  {/snippet}
  {#if !navigator.bluetooth}
    <div role="alert" class="alert alert-warning alert-soft">
      This browser exposes no Web Bluetooth API. It needs a Chromium-based browser on a secure
      origin (https or localhost).
    </div>
  {:else}
    <ul class="list">
      <StatusItem icon={BluetoothConnected} title="Link" description={$status} />
      <StatusItem icon={Remote} title="Device" description={$deviceName ?? 'not connected'} />
      <StatusItem
        icon={Health}
        title="Round trip"
        description={$latency === null ? 'no ping yet' : `${$latency} ms`}
      />
      <StatusItem icon={Router} title="Nordic UART service" description={SERVICE_UUID} />
      <StatusItem icon={Router} title="Frame size" description={`${CHUNK_BYTES} bytes`} />
    </ul>
  {/if}
</Section>
