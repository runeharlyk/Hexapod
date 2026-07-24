<script lang="ts">
  import { location } from '$lib/stores'
  import { connectBluetooth, connectWebsocket, link, transportLabels } from '$lib/stores/link'
  import { ble } from '$lib/transport/ble-adapter'
  import { websocket } from '$lib/transport/websocket-adapter'
  import { Bluetooth, Connection } from '../icons'

  const wsStatus = websocket.status
  const wsLatency = websocket.latencyMs
  const bleStatus = ble.status
  const bleLatency = ble.latencyMs

  const dotClass = $derived(
    $link.status === 'connected' ?
      $link.responsive ?
        'bg-success'
      : 'bg-warning'
    : $link.status === 'connecting' ? 'bg-warning animate-pulse'
    : 'bg-error'
  )

  const label = $derived(
    $link.status === 'connecting' ? 'Connecting'
    : $link.transport === null ? 'Offline'
    : transportLabels[$link.transport]
  )

  const detail = $derived(
    $link.status !== 'connected' ? ''
    : $link.latencyMs === null ? 'no reply'
    : `${$link.latencyMs} ms`
  )

  const statusText = (status: string, latency: number | null) =>
    status === 'connected' ?
      latency === null ?
        'Connected, no reply'
      : `Connected, ${latency} ms`
    : status === 'connecting' ? 'Connecting…'
    : 'Disconnected'
</script>

<div class="dropdown dropdown-end">
  <div
    tabindex="0"
    role="button"
    class="btn btn-ghost btn-sm gap-2 px-2"
    aria-label="Robot link: {label} {detail}"
  >
    <span class="size-2 shrink-0 rounded-full {dotClass}"></span>
    <span class="text-xs font-medium">{label}</span>
    {#if detail}
      <span class="hidden text-xs opacity-60 sm:inline">{detail}</span>
    {/if}
  </div>

  <div class="dropdown-content bg-base-200 rounded-box z-50 mt-2 w-72 p-3 shadow-lg">
    <div class="flex items-center gap-2">
      <Connection class="h-5 w-5 shrink-0" />
      <span class="flex-1 text-sm font-semibold">WiFi</span>
      <span class="text-xs opacity-70">{statusText($wsStatus, $wsLatency)}</span>
    </div>

    <div class="mt-2 flex gap-2">
      <input
        class="input input-sm min-w-0 flex-1"
        aria-label="Robot address"
        placeholder={typeof window === 'undefined' ? '' : window.location.host}
        bind:value={$location}
      />
      {#if $wsStatus === 'disconnected'}
        <button class="btn btn-sm btn-primary" onclick={connectWebsocket}>Connect</button>
      {:else}
        <button class="btn btn-sm" onclick={() => websocket.disconnect()}>Disconnect</button>
      {/if}
    </div>

    {#if navigator.bluetooth}
      <div class="divider my-2"></div>

      <div class="flex items-center gap-2">
        <Bluetooth class="h-5 w-5 shrink-0" />
        <span class="flex-1 text-sm font-semibold">Bluetooth</span>
        <span class="text-xs opacity-70">{statusText($bleStatus, $bleLatency)}</span>
      </div>

      <div class="mt-2 flex justify-end">
        {#if $bleStatus === 'disconnected'}
          <button class="btn btn-sm btn-primary" onclick={connectBluetooth}>Pair</button>
        {:else}
          <button class="btn btn-sm" onclick={() => ble.disconnect()}>Disconnect</button>
        {/if}
      </div>
    {/if}
  </div>
</div>
