<script lang="ts">
  import { resolve } from '$app/paths'
  import Card from '$lib/components/layout/Card.svelte'
  import { Bluetooth, Check, Connection as Usb, WiFi } from '$lib/components/icons'
  import { location } from '$lib/stores'
  import {
    chooseSerialPort,
    connectBluetooth,
    connectSerial,
    connectToAddress,
    link,
    serialAvailable,
    transportLabels,
    type TransportKind
  } from '$lib/stores/link'
  import { linkStatusLabels, wifiAction } from '$lib/connection'
  import type { LinkStatus } from '$lib/interfaces/transport.interface'
  import { ble } from '$lib/transport/ble-adapter'
  import { serial } from '$lib/transport/serial-adapter'
  import { socketUrl, websocket } from '$lib/transport/websocket-adapter'

  const wsStatus = websocket.status
  const bleStatus = ble.status
  const bleDevice = ble.deviceName
  const serialStatus = serial.status

  let address = $state($location)

  const action = $derived(wifiAction($wsStatus, $location, address))
  const pageHost = window.location.host

  const connectWifi = () => connectToAddress(address)

  const onWifiAction = () => {
    if (action === 'connect') connectWifi()
    else websocket.disconnect()
  }

  const connectedTo = $derived(
    $link.transport === 'websocket' ? $location || pageHost
    : $link.transport === 'bluetooth' ? ($bleDevice ?? 'the robot')
    : 'the robot'
  )

  const badgeClass: Record<LinkStatus, string> = {
    connected: 'badge-success',
    connecting: 'badge-warning',
    disconnected: 'badge-ghost'
  }
</script>

{#snippet badges(kind: TransportKind, status: LinkStatus)}
  {#if $link.transport === kind}
    <span class="badge badge-primary badge-sm">In use</span>
  {/if}
  <span class="badge badge-sm {badgeClass[status]}">
    {#if status === 'connecting'}
      <span class="loading loading-spinner loading-xs"></span>
    {/if}
    {linkStatusLabels[status]}
  </span>
{/snippet}

{#if $link.status === 'connected' && $link.transport}
  <div role="status" class="alert alert-success alert-soft">
    <Check class="size-5 shrink-0" />
    <div class="min-w-0">
      <div class="font-medium">Connected over {transportLabels[$link.transport]}</div>
      <div class="truncate text-xs">{connectedTo}</div>
    </div>
    <a class="btn btn-primary btn-sm" href={resolve('/controller')}>Open controller</a>
  </div>
{:else}
  <p class="text-base-content/70 text-sm">
    Not connected. Connect over WiFi, Bluetooth or USB; with several up, USB is used first, then
    WiFi.
  </p>
{/if}

<Card title={transportLabels.websocket} icon={WiFi}>
  {#snippet actions()}
    {@render badges('websocket', $wsStatus)}
  {/snippet}
  <div class="flex flex-col gap-2">
    <label class="text-sm font-medium" for="robot-address">Robot address</label>
    <div class="flex gap-2">
      <input
        id="robot-address"
        class="input min-w-0 flex-1"
        placeholder={pageHost}
        bind:value={address}
        onkeydown={e => e.key === 'Enter' && action === 'connect' && connectWifi()}
      />
      <button
        class="btn {action === 'connect' ? 'btn-primary' : 'btn-outline'}"
        onclick={onWifiAction}
      >
        {action === 'connect' ? 'Connect'
        : action === 'stop' ? 'Stop'
        : 'Disconnect'}
      </button>
    </div>
    <p class="text-base-content/70 font-mono text-xs break-all">
      {socketUrl(address.trim() || pageHost)}
    </p>
  </div>
</Card>

<Card title={transportLabels.bluetooth} icon={Bluetooth}>
  {#snippet actions()}
    {@render badges('bluetooth', $bleStatus)}
  {/snippet}
  {#if !navigator.bluetooth}
    <p class="text-base-content/70 text-sm">
      Bluetooth needs Chrome or Edge on an https or localhost page, so it is unavailable here.
    </p>
  {:else}
    <div class="flex flex-wrap items-center justify-end gap-2">
      <span class="text-base-content/70 min-w-0 flex-1 basis-48 text-sm break-words">
        {$bleDevice ?? "The browser's chooser lists only hexapods."}
      </span>
      {#if $bleStatus === 'connected'}
        <button class="btn btn-outline" onclick={() => ble.disconnect()}>Disconnect</button>
      {:else}
        <button
          class="btn btn-primary"
          onclick={connectBluetooth}
          disabled={$bleStatus === 'connecting'}
        >
          Connect
        </button>
      {/if}
    </div>
  {/if}
</Card>

<Card title={transportLabels.serial} icon={Usb}>
  {#snippet actions()}
    {@render badges('serial', $serialStatus)}
  {/snippet}
  {#if !serialAvailable()}
    <p class="text-base-content/70 text-sm">This browser has no Web Serial: use Chrome or Edge.</p>
  {:else}
    <div class="flex flex-wrap items-center justify-end gap-2">
      <span class="text-base-content/70 min-w-0 flex-1 basis-48 text-sm"
        >The robot's native USB port.</span
      >
      {#if $serialStatus === 'connected'}
        <button class="btn btn-outline" onclick={() => serial.disconnect()}>Disconnect</button>
      {:else}
        <div class="flex gap-2">
          <button class="btn" onclick={chooseSerialPort}>Choose port</button>
          <button
            class="btn btn-primary"
            onclick={connectSerial}
            disabled={$serialStatus === 'connecting'}
          >
            Connect
          </button>
        </div>
      {/if}
    </div>
  {/if}
</Card>
