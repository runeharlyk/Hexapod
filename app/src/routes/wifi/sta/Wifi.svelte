<script lang="ts">
  import { modals } from 'svelte-modals'
  import DragDropList, { VerticalDropZone, reorder, type DropEvent } from 'svelte-dnd-list'
  import ConfirmDialog from '$lib/components/ConfirmDialog.svelte'
  import InfoDialog from '$lib/components/InfoDialog.svelte'
  import StatusItem from '$lib/components/StatusItem.svelte'
  import { notifications } from '$lib/components/toasts/notifications'
  import Advanced from '$lib/components/layout/Advanced.svelte'
  import Field from '$lib/components/layout/Field.svelte'
  import Section from '$lib/components/layout/Section.svelte'
  import SettingsForm from '$lib/components/layout/SettingsForm.svelte'
  import WatchedStatus from '$lib/components/layout/WatchedStatus.svelte'
  import { Settings } from '$lib/components/layout/settings.svelte'
  import {
    Add,
    AP,
    Cancel,
    Channel,
    Check,
    Delete,
    DNS,
    Edit,
    Gateway,
    Home,
    MAC,
    Router,
    Scan,
    SSID,
    Subnet,
    WiFi
  } from '$lib/components/icons'
  import ScanNetworks from './Scan.svelte'
  import ConnectDialog from './ConnectDialog.svelte'
  import NetworkDialog from './NetworkDialog.svelte'
  import { afterDelete, afterMove } from './selection'
  import type { KnownNetworkItem, NetworkItem } from '$lib/types/models'
  import { ipToString, ipToU32 } from '$lib/proto-api'
  import { dataBroker } from '$lib/transport/databroker'
  import { ask } from '$lib/transport/reply'
  import { WifiSettings, WifiStatus, type WifiNetwork } from '$lib/platform_shared/api'
  import { watch } from '$lib/robot/watch'

  // The page keeps the UI's snake_case shape; the proto carries IPs as uint32.
  type WifiForm = {
    hostname: string
    priority_rssi: boolean
    selected_network: number
    wifi_networks: KnownNetworkItem[]
  }

  const MAX_NETWORKS = 5

  const netToForm = (n: WifiNetwork): KnownNetworkItem => ({
    ssid: n.ssid,
    password: n.password,
    static_ip_config: n.staticIpConfig,
    local_ip: ipToString(n.localIp),
    subnet_mask: ipToString(n.subnetMask),
    gateway_ip: ipToString(n.gatewayIp),
    dns_ip_1: ipToString(n.dnsIp1),
    dns_ip_2: ipToString(n.dnsIp2)
  })
  const formToNet = (n: KnownNetworkItem) => ({
    ssid: n.ssid,
    password: n.password,
    staticIpConfig: n.static_ip_config,
    localIp: ipToU32(n.local_ip ?? '0.0.0.0'),
    subnetMask: ipToU32(n.subnet_mask ?? '0.0.0.0'),
    gatewayIp: ipToU32(n.gateway_ip ?? '0.0.0.0'),
    dnsIp1: ipToU32(n.dns_ip_1 ?? '0.0.0.0'),
    dnsIp2: ipToU32(n.dns_ip_2 ?? '0.0.0.0')
  })
  const toForm = (w: WifiSettings): WifiForm => ({
    hostname: w.hostname,
    priority_rssi: w.priorityRssi,
    selected_network: w.selectedNetwork,
    wifi_networks: (w.wifiNetworks ?? []).map(netToForm)
  })

  // The robot answers the subscribe with its current status and pushes every change.
  const wifiState = watch(WifiStatus)

  const settings = new Settings<WifiForm>({
    what: 'the Wi-Fi settings',
    read: async () => {
      const res = await ask(dataBroker, { wifiSettingsGet: {} }, 'the Wi-Fi settings')
      if (!res.wifiSettings) throw new Error('the reply carried no settings')
      return toForm(res.wifiSettings)
    },
    // The robot answers a refused update with a 400 and the settings it kept, passwords left out; a network sent back
    // with an empty password keeps the one it has.
    write: async value => {
      const res = await ask(
        dataBroker,
        {
          wifiSettingsUpdate: WifiSettings.create({
            hostname: value.hostname,
            priorityRssi: value.priority_rssi ?? true,
            // With none selected the robot starts from the first network.
            selectedNetwork: Math.max(value.selected_network ?? 0, 0),
            wifiNetworks: (value.wifi_networks ?? []).map(formToNet)
          })
        },
        'the Wi-Fi settings'
      )
      return res.wifiSettings && toForm(res.wifiSettings)
    },
    saved: 'Wi-Fi settings updated.'
  })

  const networks = $derived(settings.value?.wifi_networks ?? [])

  const blankNetwork = (ssid = ''): KnownNetworkItem => ({
    ssid,
    password: '',
    static_ip_config: false,
    local_ip: undefined,
    subnet_mask: undefined,
    gateway_ip: undefined,
    dns_ip_1: undefined,
    dns_ip_2: undefined
  })

  // The robot keeps at most five networks; adding a sixth says so instead of opening the editor.
  const roomForAnother = () => {
    if (networks.length < MAX_NETWORKS) return true
    modals.open(InfoDialog, {
      title: 'Reached Maximum Networks',
      message: 'You have reached the maximum number of networks. Please delete one to add another.',
      dismiss: { label: 'OK', icon: Check },
      onDismiss: () => modals.close()
    })
    return false
  }

  let form: { requestSave: () => void }
  // Whether the network editor holds a change it has not applied to the list.
  let editorChanged = $state(false)

  // The editor changes the draft; Save sends it.
  const editNetwork = (index: number | null, ssid = '') =>
    modals
      .open(NetworkDialog, {
        network: index === null ? blankNetwork(ssid) : networks[index],
        isNew: index === null,
        onConfirm: (network: KnownNetworkItem) => {
          if (index === null) settings.value.wifi_networks.push(network)
          else settings.value.wifi_networks[index] = network
        },
        onEdit: (changed: boolean) => (editorChanged = changed)
      })
      .then(() => (editorChanged = false))

  const addNetwork = () => {
    if (roomForAnother()) editNetwork(null)
  }

  const scanForNetworks = () => {
    if (!roomForAnother()) return
    modals.open(ScanNetworks, {
      storeNetwork: (ssid: string) => {
        modals.close()
        editNetwork(null, ssid)
      },
      connect: connectToScanned
    })
  }

  function connectToScanned(network: NetworkItem) {
    modals.close()
    if (network.encryption_type === 0) {
      connectWithPassword(network.ssid, '')
    } else {
      modals.open(ConnectDialog, {
        ssid: network.ssid,
        onConfirm: (password: string) => connectWithPassword(network.ssid, password)
      })
    }
  }

  // Connecting saves at once, through the form's checks: the network, selected, with whatever else the draft holds.
  const connectToSaved = (index: number) => {
    settings.value.selected_network = index
    form.requestSave()
  }

  function connectWithPassword(ssid: string, password: string) {
    const list = settings.value.wifi_networks
    let index = list.findIndex(n => n.ssid === ssid)
    if (index === -1) {
      if (list.length >= MAX_NETWORKS) {
        notifications.error('At most 5 networks can be saved. Remove one first.', 3000)
        return
      }
      list.push({ ...blankNetwork(ssid), password })
      index = list.length - 1
    } else {
      list[index] = { ...list[index], password }
    }
    connectToSaved(index)
  }

  const confirmDelete = (index: number) =>
    modals.open(ConfirmDialog, {
      title: 'Delete Network',
      message: `Delete ${networks[index].ssid} from the saved networks?`,
      labels: {
        cancel: { label: 'Cancel', icon: Cancel },
        confirm: { label: 'Delete', icon: Delete }
      },
      onConfirm: () => {
        settings.value.wifi_networks.splice(index, 1)
        settings.value.selected_network = afterDelete(settings.value.selected_network, index)
        modals.close()
      }
    })

  function onDrop({ detail: { from, to } }: CustomEvent<DropEvent>) {
    if (!to || from === to) return
    settings.value.wifi_networks = reorder(settings.value.wifi_networks, from.index, to.index)
    settings.value.selected_network = afterMove(
      settings.value.selected_network,
      from.index,
      to.index
    )
  }
</script>

<Section title="Status" icon={Router}>
  <WatchedStatus watched={$wifiState} what="the Wi-Fi status">
    {#snippet children(status)}
      <ul class="list">
        <StatusItem
          icon={AP}
          title="Status"
          variant={status.status === 3 ? 'success' : 'error'}
          description={status.status === 3 ? 'Connected' : 'Inactive'}
        />
        {#if status.status === 3}
          <StatusItem icon={SSID} title="SSID" description={status.ssid} />
          <StatusItem icon={Home} title="IP address" description={ipToString(status.localIp)} />
          <StatusItem icon={WiFi} title="RSSI" description="{status.rssi} dBm" />
        {/if}
      </ul>
      {#if status.status === 3}
        <Advanced title="Details">
          <ul class="list">
            <StatusItem icon={MAC} title="MAC address" description={status.macAddress} />
            <StatusItem icon={Channel} title="Channel" description={status.channel} />
            <StatusItem
              icon={Gateway}
              title="Gateway IP"
              description={ipToString(status.gatewayIp)}
            />
            <StatusItem
              icon={Subnet}
              title="Subnet mask"
              description={ipToString(status.subnetMask)}
            />
            <StatusItem icon={DNS} title="DNS" description={ipToString(status.dnsIp1)} />
          </ul>
        </Advanced>
      {/if}
    {/snippet}
  </WatchedStatus>
</Section>

<SettingsForm
  bind:this={form}
  title="Saved networks"
  description="The robot tries them in this order; drag a network to move it."
  icon={WiFi}
  {settings}
  pending={editorChanged}
  rows={4}
>
  {#snippet actions()}
    <button type="button" class="btn btn-sm" onclick={addNetwork}>
      <Add class="size-4" />Add
    </button>
    <button
      type="button"
      class="btn btn-sm"
      aria-label="Scan for networks"
      onclick={scanForNetworks}
    >
      <Scan class="size-4" />Scan
    </button>
  {/snippet}

  {#snippet children(form)}
    {#if form.wifi_networks.length === 0}
      <p class="text-base-content/70">No saved networks. Add one, or scan for networks nearby.</p>
    {:else}
      <DragDropList
        id="networks"
        type={VerticalDropZone}
        itemSize={60}
        itemCount={form.wifi_networks.length}
        zoneClass="list"
        on:drop={onDrop}
      >
        {#snippet children({ index }: { index: number })}
          {@const network = form.wifi_networks[index]}
          {@const selected = form.selected_network === index}
          <div class="list-row h-full cursor-grab items-center py-2">
            <Router class="size-5 opacity-70" />
            <div class="min-w-0">
              <div class="truncate font-medium" title={network.ssid}>{network.ssid}</div>
              <div class="text-base-content/70 text-xs">
                {[selected && 'Selected', network.static_ip_config && 'static IP']
                  .filter(Boolean)
                  .join(', ')}
              </div>
            </div>
            <div class="flex items-center">
              <button
                type="button"
                class="btn btn-sm btn-square {selected ? 'btn-primary' : 'btn-ghost'}"
                title={selected ? 'Selected network' : 'Connect'}
                aria-label="Connect to {network.ssid}"
                onclick={() => connectToSaved(index)}
              >
                <WiFi class="size-5" />
              </button>
              <button
                type="button"
                class="btn btn-sm btn-square btn-ghost"
                aria-label="Edit {network.ssid}"
                onclick={() => editNetwork(index)}
              >
                <Edit class="size-5" />
              </button>
              <button
                type="button"
                class="btn btn-sm btn-square btn-ghost"
                aria-label="Delete {network.ssid}"
                onclick={() => confirmDelete(index)}
              >
                <Delete class="text-error size-5" />
              </button>
            </div>
          </div>
        {/snippet}
      </DragDropList>
    {/if}
    <Field label="Host name" for="hostname" hint="3 to 32 characters">
      <input
        id="hostname"
        class="input validator w-full"
        type="text"
        required
        pattern={'.{3,32}'}
        maxlength="32"
        bind:value={form.hostname}
      />
    </Field>
    <Field label="Connect to the strongest network" for="priority-rssi">
      <input id="priority-rssi" type="checkbox" class="toggle" bind:checked={form.priority_rssi} />
    </Field>
  {/snippet}
</SettingsForm>
