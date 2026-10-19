<script lang="ts">
  import { resolve } from '$app/paths'
  import SettingsCard from '$lib/components/SettingsCard.svelte'
  import {
    Bluetooth,
    Check,
    Connection as Usb,
    WiFi,
    type IconComponent
  } from '$lib/components/icons'
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

{#snippet header(Icon: IconComponent, kind: TransportKind, status: LinkStatus)}
  <div class="flex items-center gap-2">
    <Icon class="h-5 w-5 shrink-0" />
    <h3 class="font-medium">{transportLabels[kind]}</h3>
    {#if $link.transport === kind}
      <span class="badge badge-primary badge-sm">In use</span>
    {/if}
    <span class="badge badge-sm ml-auto {badgeClass[status]}">
      {#if status === 'connecting'}
        <span class="loading loading-spinner loading-xs"></span>
      {/if}
      {linkStatusLabels[status]}
    </span>
  </div>
{/snippet}

<SettingsCard collapsible={false}>
  {#snippet icon()}
    <WiFi class="lex-shrink-0 mr-2 h-6 w-6 self-end" />
  {/snippet}
  {#snippet title()}
    <span>Connection</span>
  {/snippet}

  {#if $link.status === 'connected' && $link.transport}
    <div class="bg-base-100 rounded-box flex flex-wrap items-center gap-3 p-3">
      <span class="bg-success flex size-6 shrink-0 items-center justify-center rounded-full">
        <Check class="text-base-100 h-4 w-4" />
      </span>
      <div class="min-w-0 flex-1">
        <div class="font-medium">Connected over {transportLabels[$link.transport]}</div>
        <div class="truncate text-xs opacity-60">{connectedTo}</div>
      </div>
      <a class="btn btn-primary btn-sm" href={resolve('/controller')}>Open controller</a>
    </div>
  {:else}
    <p class="text-sm opacity-70">
      Not connected. Connect over WiFi, Bluetooth or USB; with several up, USB is used first, then
      WiFi.
    </p>
  {/if}

  <div class="divider my-0"></div>

  <section class="flex flex-col gap-2">
    {@render header(WiFi, 'websocket', $wsStatus)}
    <label class="text-sm opacity-70" for="robot-address">Robot address</label>
    <div class="flex gap-2">
      <input
        id="robot-address"
        class="input input-sm min-w-0 flex-1"
        placeholder={pageHost}
        bind:value={address}
        onkeydown={e => e.key === 'Enter' && action === 'connect' && connectWifi()}
      />
      <button
        class="btn btn-sm {action === 'connect' ? 'btn-primary' : 'btn-outline'}"
        onclick={onWifiAction}
      >
        {action === 'connect' ? 'Connect'
        : action === 'stop' ? 'Stop'
        : 'Disconnect'}
      </button>
    </div>
    <p class="font-mono text-xs opacity-60">{socketUrl(address.trim() || pageHost)}</p>
  </section>

  <div class="divider my-0"></div>

  <section class="flex flex-col gap-2">
    {@render header(Bluetooth, 'bluetooth', $bleStatus)}
    {#if !navigator.bluetooth}
      <p class="text-sm opacity-70">
        Bluetooth needs Chrome or Edge on an https or localhost page, so it is unavailable here.
      </p>
    {:else}
      <div class="flex items-center gap-2">
        <span class="min-w-0 flex-1 truncate text-sm opacity-70">
          {$bleDevice ?? "The browser's chooser lists only hexapods."}
        </span>
        {#if $bleStatus === 'connected'}
          <button class="btn btn-sm btn-outline" onclick={() => ble.disconnect()}>Disconnect</button
          >
        {:else}
          <button
            class="btn btn-sm btn-primary"
            onclick={connectBluetooth}
            disabled={$bleStatus === 'connecting'}
          >
            Connect
          </button>
        {/if}
      </div>
    {/if}
  </section>

  <div class="divider my-0"></div>

  <section class="flex flex-col gap-2">
    {@render header(Usb, 'serial', $serialStatus)}
    {#if !serialAvailable()}
      <p class="text-sm opacity-70">This browser has no Web Serial: use Chrome or Edge.</p>
    {:else}
      <div class="flex items-center gap-2">
        <span class="min-w-0 flex-1 text-sm opacity-70">The robot's native USB port.</span>
        {#if $serialStatus === 'connected'}
          <button class="btn btn-sm btn-outline" onclick={() => serial.disconnect()}>
            Disconnect
          </button>
        {:else}
          <button class="btn btn-sm btn-ghost" onclick={chooseSerialPort}>Choose port</button>
          <button
            class="btn btn-sm btn-primary"
            onclick={connectSerial}
            disabled={$serialStatus === 'connecting'}
          >
            Connect
          </button>
        {/if}
      </div>
    {/if}
  </section>
</SettingsCard>
