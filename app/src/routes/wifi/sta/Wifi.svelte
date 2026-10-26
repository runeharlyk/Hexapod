<script lang="ts">
  import { modals } from 'svelte-modals'
  import DragDropList, { VerticalDropZone, reorder, type DropEvent } from 'svelte-dnd-list'
  import ConfirmDialog from '$lib/components/ConfirmDialog.svelte'
  import InfoDialog from '$lib/components/InfoDialog.svelte'
  import StatusItem from '$lib/components/StatusItem.svelte'
  import { notifications } from '$lib/components/toasts/notifications'
  import Advanced from '$lib/components/layout/Advanced.svelte'
  import Field from '$lib/components/layout/Field.svelte'
  import ChangedNote from '$lib/components/layout/ChangedNote.svelte'
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
  import { WifiSettings, WifiStatus, type WifiNetwork } from '$lib/platform_shared/api'
  import { topicSource } from '$lib/robot/settings-sources'
  import { watch } from '$lib/robot/watch'

  const MAX_NETWORKS = 5

  // The network dialog edits a network in the UI's snake_case shape; the proto carries IPs as uint32.
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
  const formToNet = (n: KnownNetworkItem): WifiNetwork => ({
    ssid: n.ssid,
    password: n.password,
    staticIpConfig: n.static_ip_config,
    localIp: ipToU32(n.local_ip ?? '0.0.0.0'),
    subnetMask: ipToU32(n.subnet_mask ?? '0.0.0.0'),
    gatewayIp: ipToU32(n.gateway_ip ?? '0.0.0.0'),
    dnsIp1: ipToU32(n.dns_ip_1 ?? '0.0.0.0'),
    dnsIp2: ipToU32(n.dns_ip_2 ?? '0.0.0.0')
  })
  // The robot answers the subscribe with its current status and pushes every change.
  const wifiState = watch(WifiStatus)

  const settings = new Settings<WifiSettings>(
    topicSource(WifiSettings, {
      what: 'the Wi-Fi settings',
      saved: 'Wi-Fi settings updated.',
      // The robot answers a refused update with a 400, and a network sent back with an empty password keeps the one it
      // has. With none selected it starts from the first network.
      update: value => ({
        wifiSettingsUpdate: { ...value, selectedNetwork: Math.max(value.selectedNetwork, 0) }
      }),
      together: [['wifiNetworks', 'selectedNetwork']]
    })
  )

  const networks = $derived(settings.value?.wifiNetworks ?? [])

  const summary = (list: unknown) =>
    (list as WifiNetwork[]).map(network => network.ssid).join(', ') || 'no saved networks'

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

  // While a dialog edits the list it is held as a field in focus is (settings.svelte.ts): a push leaves the list as the
  // dialog saw it, so the edit lands on the network the dialog opened. Closing the dialog takes the robot's list, or
  // after an edit keeps the user's with a note when the robot's changed meanwhile.
  const holdingList = (dialog: () => Promise<unknown>) => {
    settings.focus('wifiNetworks')
    return dialog().finally(() => settings.blur('wifiNetworks'))
  }

  // The editor changes the draft; Save sends it.
  const editNetwork = (index: number | null, ssid = '') =>
    holdingList(() =>
      modals.open(NetworkDialog, {
        network: index === null ? blankNetwork(ssid) : netToForm(networks[index]),
        isNew: index === null,
        onConfirm: (network: KnownNetworkItem) => {
          if (index === null) settings.value.wifiNetworks.push(formToNet(network))
          else settings.value.wifiNetworks[index] = formToNet(network)
        },
        onEdit: (changed: boolean) => (editorChanged = changed)
      })
    ).then(() => (editorChanged = false))

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
    settings.value.selectedNetwork = index
    form.requestSave()
  }

  function connectWithPassword(ssid: string, password: string) {
    const list = settings.value.wifiNetworks
    let index = list.findIndex(n => n.ssid === ssid)
    if (index === -1) {
      if (list.length >= MAX_NETWORKS) {
        notifications.error('At most 5 networks can be saved. Remove one first.', 3000)
        return
      }
      list.push({ ...formToNet(blankNetwork(ssid)), password })
      index = list.length - 1
    } else {
      list[index] = { ...list[index], password }
    }
    connectToSaved(index)
  }

  const confirmDelete = (index: number) =>
    holdingList(() =>
      modals.open(ConfirmDialog, {
        title: 'Delete Network',
        message: `Delete ${networks[index].ssid} from the saved networks?`,
        labels: {
          cancel: { label: 'Cancel', icon: Cancel },
          confirm: { label: 'Delete', icon: Delete }
        },
        onConfirm: () => {
          settings.value.wifiNetworks.splice(index, 1)
          settings.value.selectedNetwork = afterDelete(settings.value.selectedNetwork, index)
          modals.close()
        }
      })
    )

  function onDrop({ detail: { from, to } }: CustomEvent<DropEvent>) {
    if (!to || from === to) return
    settings.value.wifiNetworks = reorder(settings.value.wifiNetworks, from.index, to.index)
    settings.value.selectedNetwork = afterMove(settings.value.selectedNetwork, from.index, to.index)
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

  {#snippet children(form, notes)}
    {#if form.wifiNetworks.length === 0}
      <p class="text-base-content/70">No saved networks. Add one, or scan for networks nearby.</p>
    {:else}
      <DragDropList
        id="networks"
        type={VerticalDropZone}
        itemSize={60}
        itemCount={form.wifiNetworks.length}
        zoneClass="list"
        on:drop={onDrop}
      >
        {#snippet children({ index }: { index: number })}
          {@const network = form.wifiNetworks[index]}
          {@const selected = form.selectedNetwork === index}
          <div class="list-row h-full cursor-grab items-center py-2">
            <Router class="size-5 opacity-70" />
            <div class="min-w-0">
              <div class="truncate font-medium" title={network.ssid}>{network.ssid}</div>
              <div class="text-base-content/70 text-xs">
                {[selected && 'Selected', network.staticIpConfig && 'static IP']
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
    <ChangedNote {notes} field="wifiNetworks" format={summary} />
    <Field label="Host name" for="hostname" hint="3 to 32 characters">
      <input
        id="hostname"
        name="hostname"
        class="input validator w-full"
        type="text"
        required
        pattern={'.{3,32}'}
        maxlength="32"
        bind:value={form.hostname}
      />
      <ChangedNote {notes} field="hostname" />
    </Field>
    <Field label="Connect to the strongest network" for="priority-rssi">
      <input
        id="priority-rssi"
        name="priorityRssi"
        type="checkbox"
        class="toggle"
        bind:checked={form.priorityRssi}
      />
      <ChangedNote {notes} field="priorityRssi" format={value => (value ? 'on' : 'off')} />
    </Field>
  {/snippet}
</SettingsForm>
