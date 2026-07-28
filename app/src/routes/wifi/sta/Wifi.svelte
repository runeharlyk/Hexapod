<script lang="ts">
  import { onMount, onDestroy } from 'svelte'
  import { modals } from 'svelte-modals'
  import { slide } from 'svelte/transition'
  import { cubicOut } from 'svelte/easing'
  import { notifications } from '$lib/components/toasts/notifications'
  import DragDropList, { VerticalDropZone, reorder, type DropEvent } from 'svelte-dnd-list'
  import SettingsCard from '$lib/components/SettingsCard.svelte'
  import { PasswordInput } from '$lib/components/input'
  import ConfirmDialog from '$lib/components/ConfirmDialog.svelte'
  import ScanNetworks from './Scan.svelte'
  import ConnectDialog from './ConnectDialog.svelte'
  import type { NetworkItem } from '$lib/types/models'
  import Spinner from '$lib/components/Spinner.svelte'
  import InfoDialog from '$lib/components/InfoDialog.svelte'
  import type { KnownNetworkItem } from '$lib/types/models'
  import { ipToString, ipToU32 } from '$lib/proto-api'
  import { dataBroker } from '$lib/transport/databroker'
  import { WifiStatus } from '$lib/platform_shared/api'

  const netToForm = (n: any): KnownNetworkItem => ({
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
  import {
    Cancel,
    Delete,
    Check,
    Router,
    AP,
    SSID,
    Home,
    WiFi,
    Down,
    MAC,
    Channel,
    Gateway,
    Subnet,
    DNS,
    Add,
    Scan,
    Edit
  } from '$lib/components/icons'
  import StatusItem from '$lib/components/StatusItem.svelte'

  let networkEditable: KnownNetworkItem = $state({
    ssid: '',
    password: '',
    static_ip_config: false,
    local_ip: undefined,
    subnet_mask: undefined,
    gateway_ip: undefined,
    dns_ip_1: undefined,
    dns_ip_2: undefined
  })

  let static_ip_config = $state(false)

  let newNetwork: boolean = $state(true)
  let showNetworkEditor: boolean = $state(false)

  let wifiStatus: any = $state()
  let wifiSettings: any = $state()

  let dndNetworkList: KnownNetworkItem[] = $state([])

  let showWifiDetails = $state(false)

  let formField: any = $state()

  let formErrors = $state({
    ssid: false,
    local_ip: false,
    gateway_ip: false,
    subnet_mask: false,
    dns_1: false,
    dns_2: false
  })

  let formErrorhostname = $state(false)

  const pageAbort = new AbortController()
  onDestroy(() => pageAbort.abort())

  const applyWifiStatus = (s: any) => {
    wifiStatus = {
      status: s.status,
      local_ip: ipToString(s.localIp),
      mac_address: s.macAddress,
      rssi: s.rssi,
      ssid: s.ssid,
      bssid: s.bssid,
      channel: s.channel,
      subnet_mask: ipToString(s.subnetMask),
      gateway_ip: ipToString(s.gatewayIp),
      dns_ip_1: ipToString(s.dnsIp1),
      dns_ip_2: ipToString(s.dnsIp2)
    }
  }

  async function getWifiStatus() {
    const res = await dataBroker
      .request({ wifiStatusGet: {} }, { signal: pageAbort.signal })
      .catch(() => null)
    if (res?.wifiStatus) applyWifiStatus(res.wifiStatus)
    return wifiStatus
  }

  onMount(() => dataBroker.on(WifiStatus, applyWifiStatus))

  async function getWifiSettings() {
    try {
      const res = await dataBroker.request({ wifiSettingsGet: {} }, { signal: pageAbort.signal })
      const w = res.wifiSettings
      if (!w) return
      wifiSettings = {
        hostname: w.hostname,
        priority_rssi: w.priorityRssi,
        selected_network: w.selectedNetwork,
        wifi_networks: (w.wifiNetworks ?? []).map(netToForm)
      }
      dndNetworkList = wifiSettings.wifi_networks
      return wifiSettings
    } catch (e) {
      if ((e as Error)?.name === 'AbortError') return
      notifications.error('Could not read Wi-Fi settings — is the robot connected?', 3000)
    }
  }

  async function postWiFiSettings(data: any) {
    try {
      const res = await dataBroker.request({
        wifiSettingsUpdate: {
          hostname: data.hostname,
          priorityRssi: data.priority_rssi ?? true,
          selectedNetwork: data.selected_network ?? 0,
          wifiNetworks: (data.wifi_networks ?? []).map(formToNet)
        }
      })
      if (!res.wifiSettings) {
        notifications.error('Failed to update Wi-Fi settings.', 3000)
        return
      }
      notifications.success('Wi-Fi settings updated.', 3000)
    } catch {
      notifications.error('Failed to update Wi-Fi settings — is the robot connected?', 3000)
    }
  }

  function validateHostName() {
    if (wifiSettings.hostname.length < 3 || wifiSettings.hostname.length > 32) {
      formErrorhostname = true
    } else {
      formErrorhostname = false
      wifiSettings.wifi_networks = dndNetworkList
      postWiFiSettings(wifiSettings)
      console.log(wifiSettings)
    }
  }

  function validateWiFiForm(event: SubmitEvent) {
    event.preventDefault()
    let valid = true

    if (networkEditable.ssid.length < 3 || networkEditable.ssid.length > 32) {
      valid = false
      formErrors.ssid = true
    } else {
      formErrors.ssid = false
    }

    networkEditable.static_ip_config = static_ip_config

    if (networkEditable.static_ip_config) {
      const regexExp =
        /\b(?:(?:2(?:[0-4][0-9]|5[0-5])|[0-1]?[0-9]?[0-9])\.){3}(?:(?:2([0-4][0-9]|5[0-5])|[0-1]?[0-9]?[0-9]))\b/

      if (!regexExp.test(networkEditable.gateway_ip!)) {
        valid = false
        formErrors.gateway_ip = true
      } else {
        formErrors.gateway_ip = false
      }

      if (!regexExp.test(networkEditable.subnet_mask!)) {
        valid = false
        formErrors.subnet_mask = true
      } else {
        formErrors.subnet_mask = false
      }

      if (!regexExp.test(networkEditable.local_ip!)) {
        valid = false
        formErrors.local_ip = true
      } else {
        formErrors.local_ip = false
      }

      if (!regexExp.test(networkEditable.dns_ip_1!)) {
        valid = false
        formErrors.dns_1 = true
      } else {
        formErrors.dns_1 = false
      }

      if (!regexExp.test(networkEditable.dns_ip_2!)) {
        valid = false
        formErrors.dns_2 = true
      } else {
        formErrors.dns_2 = false
      }
    } else {
      formErrors.local_ip = false
      formErrors.subnet_mask = false
      formErrors.gateway_ip = false
      formErrors.dns_1 = false
      formErrors.dns_2 = false
    }
    if (valid) {
      if (newNetwork) {
        dndNetworkList.push(networkEditable)
      } else {
        dndNetworkList.splice(dndNetworkList.indexOf(networkEditable), 1, networkEditable)
      }
      addNetwork()
      dndNetworkList = [...dndNetworkList] //Trigger reactivity
      showNetworkEditor = false
    }
  }

  function scanForNetworks() {
    modals.open(ScanNetworks, {
      storeNetwork: (network: string) => {
        addNetwork()
        networkEditable.ssid = network
        showNetworkEditor = true
        modals.close()
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

  async function connectToSaved(index: number) {
    wifiSettings = {
      hostname: wifiSettings?.hostname ?? 'hexapod',
      priority_rssi: wifiSettings?.priority_rssi ?? true,
      selected_network: index,
      wifi_networks: dndNetworkList
    }
    await postWiFiSettings(wifiSettings)
  }

  async function connectWithPassword(ssid: string, password: string) {
    let index = dndNetworkList.findIndex(n => n.ssid === ssid)
    if (index === -1) {
      if (dndNetworkList.length >= 5) {
        notifications.error('At most 5 networks can be saved. Remove one first.', 3000)
        return
      }
      dndNetworkList = [
        ...dndNetworkList,
        {
          ssid,
          password,
          static_ip_config: false,
          local_ip: undefined,
          subnet_mask: undefined,
          gateway_ip: undefined,
          dns_ip_1: undefined,
          dns_ip_2: undefined
        }
      ]
      index = dndNetworkList.length - 1
    } else {
      dndNetworkList[index] = { ...dndNetworkList[index], password }
      dndNetworkList = [...dndNetworkList]
    }
    wifiSettings = {
      hostname: wifiSettings?.hostname ?? 'hexapod',
      priority_rssi: wifiSettings?.priority_rssi ?? true,
      selected_network: index,
      wifi_networks: dndNetworkList
    }
    await postWiFiSettings(wifiSettings)
  }

  function addNetwork() {
    newNetwork = true
    networkEditable = {
      ssid: '',
      password: '',
      static_ip_config: false,
      local_ip: undefined,
      subnet_mask: undefined,
      gateway_ip: undefined,
      dns_ip_1: undefined,
      dns_ip_2: undefined
    }
  }

  function handleEdit(index: number) {
    newNetwork = false
    showNetworkEditor = true
    networkEditable = dndNetworkList[index]
  }

  function confirmDelete(index: number) {
    modals.open(ConfirmDialog, {
      title: 'Delete Network',
      message: 'Are you sure you want to delete this network?',
      labels: {
        cancel: { label: 'Cancel', icon: Cancel },
        confirm: { label: 'Delete', icon: Delete }
      },
      onConfirm: () => {
        if (dndNetworkList[index].ssid === networkEditable.ssid) {
          addNetwork()
        }
        dndNetworkList.splice(index, 1)
        dndNetworkList = [...dndNetworkList] //Trigger reactivity
        showNetworkEditor = false
        modals.close()
      }
    })
  }

  function checkNetworkList() {
    if (dndNetworkList.length >= 5) {
      modals.open(InfoDialog, {
        title: 'Reached Maximum Networks',
        message:
          'You have reached the maximum number of networks. Please delete one to add another.',
        dismiss: { label: 'OK', icon: Check },
        onDismiss: () => modals.close()
      })
      return false
    } else {
      return true
    }
  }

  function onDrop({ detail: { from, to } }: CustomEvent<DropEvent>) {
    if (!to || from === to) {
      return
    }

    dndNetworkList = reorder(dndNetworkList, from.index, to.index)
    console.log(dndNetworkList)
  }
</script>

<SettingsCard collapsible={false}>
  {#snippet icon()}
    <Router class="lex-shrink-0 mr-2 h-6 w-6 self-end" />
  {/snippet}
  {#snippet title()}
    <span>WiFi Connection</span>
  {/snippet}
  <div class="w-full overflow-x-auto">
    {#await getWifiStatus()}
      <Spinner />
    {:then}
      <div
        class="flex w-full flex-col space-y-1"
        transition:slide|local={{ duration: 300, easing: cubicOut }}
      >
        <StatusItem
          icon={AP}
          title="Status"
          variant={wifiStatus.status === 3 ? 'success' : 'error'}
          description={wifiStatus.status === 3 ? 'Connected' : 'Inactive'}
        />

        {#if wifiStatus.status === 3}
          <StatusItem icon={SSID} title="SSID" description={wifiStatus.ssid} />

          <StatusItem icon={Home} title="IP Address" description={wifiStatus.local_ip} />

          <StatusItem icon={WiFi} title="RSSI" description={`${wifiStatus.rssi} dBm`}>
            <button
              class="btn btn-circle btn-ghost btn-sm modal-button"
              onclick={() => {
                showWifiDetails = !showWifiDetails
              }}
            >
              <Down
                class="text-base-content h-auto w-6 transition-transform duration-300 ease-in-out {(
                  showWifiDetails
                ) ?
                  'rotate-180'
                : ''}"
              />
            </button>
          </StatusItem>
        {/if}
      </div>

      <!-- Folds open -->
      {#if showWifiDetails}
        <div
          class="flex w-full flex-col space-y-1 pt-1"
          transition:slide|local={{ duration: 300, easing: cubicOut }}
        >
          <StatusItem icon={MAC} title="MAC Address" description={wifiStatus.mac_address} />

          <StatusItem icon={Channel} title="Channel" description={wifiStatus.channel} />

          <StatusItem icon={Gateway} title="Gateway IP" description={wifiStatus.gateway_ip} />

          <StatusItem icon={Subnet} title="Subnet Mask" description={wifiStatus.subnet_mask} />

          <StatusItem icon={DNS} title="DNS" description={wifiStatus.dns_ip_1} />
        </div>
      {/if}
    {/await}
  </div>

  <div class="bg-base-200 relative grid w-full max-w-2xl self-center overflow-hidden">
    <div
      class="min-h-16 flex w-full items-center justify-between space-x-3 p-0 text-xl font-medium"
    >
      Saved Networks
    </div>
    {#await getWifiSettings()}
      <Spinner />
    {:then}
      <div class="relative w-full overflow-visible">
        <button
          class="btn btn-primary text-primary-content btn-md absolute -top-14 right-16"
          onclick={() => {
            if (checkNetworkList()) {
              addNetwork()
              showNetworkEditor = true
            }
          }}
        >
          <Add class="h-6 w-6" /></button
        >
        <button
          class="btn btn-primary text-primary-content btn-md absolute -top-14 right-0"
          onclick={() => {
            if (checkNetworkList()) {
              scanForNetworks()
              showNetworkEditor = true
            }
          }}
        >
          <Scan class="h-6 w-6" /></button
        >

        <div
          class="overflow-x-auto space-y-1"
          transition:slide|local={{ duration: 300, easing: cubicOut }}
        >
          <DragDropList
            id="networks"
            type={VerticalDropZone}
            itemSize={60}
            itemCount={dndNetworkList.length}
            on:drop={onDrop}
          >
            {#snippet children({ index })}
              <StatusItem icon={Router} title={dndNetworkList[index].ssid}>
                <div class="space-x-0 px-0 mx-0">
                  <button
                    class="btn btn-sm {wifiSettings?.selected_network === index ?
                      'btn-primary'
                    : 'btn-ghost'}"
                    title={wifiSettings?.selected_network === index ?
                      'Selected network'
                    : 'Connect'}
                    onclick={() => connectToSaved(index)}
                  >
                    <WiFi class="h-6 w-6" /></button
                  >
                  <button
                    class="btn btn-ghost btn-sm"
                    onclick={() => {
                      handleEdit(index)
                    }}
                  >
                    <Edit class="h-6 w-6" /></button
                  >
                  <button
                    class="btn btn-ghost btn-sm"
                    onclick={() => {
                      confirmDelete(index)
                    }}
                  >
                    <Delete class="text-error h-6 w-6" />
                  </button>
                </div>
              </StatusItem>
            {/snippet}
          </DragDropList>
        </div>
      </div>

      <div class="divider mb-0"></div>
      <div
        class="flex flex-col gap-2 p-0"
        transition:slide|local={{ duration: 300, easing: cubicOut }}
      >
        <form class="" onsubmit={validateWiFiForm} novalidate bind:this={formField}>
          <div class="grid w-full grid-cols-1 content-center gap-x-4 px-4 sm:grid-cols-2">
            <div>
              <label class="label" for="channel">
                <span class="label-text text-md">Host Name</span>
              </label>
              <input
                type="text"
                min="1"
                max="32"
                class="input input-bordered invalid:border-error w-full invalid:border-2 {(
                  formErrorhostname
                ) ?
                  'border-error border-2'
                : ''}"
                bind:value={wifiSettings.hostname}
                id="channel"
                required
              />
              <label class="label" for="channel">
                <span class="label-text-alt text-error {formErrorhostname ? '' : 'hidden'}"
                  >Host name must be between 2 and 32 characters long</span
                >
              </label>
            </div>
            <label class="label inline-flex cursor-pointer content-end justify-start gap-4">
              <input
                type="checkbox"
                bind:checked={wifiSettings.priority_RSSI}
                class="checkbox checkbox-primary sm:-mb-5"
              />
              <span class="sm:-mb-5">Connect to strongest WiFi</span>
            </label>
          </div>

          {#if showNetworkEditor}
            <div class="divider my-0"></div>
            <div
              class="grid w-full grid-cols-1 content-center gap-x-4 px-4 sm:grid-cols-2"
              transition:slide|local={{ duration: 300, easing: cubicOut }}
            >
              <div>
                <label class="label" for="ssid">
                  <span class="label-text text-md">SSID</span>
                </label>
                <input
                  type="text"
                  class="input input-bordered invalid:border-error w-full invalid:border-2 {(
                    formErrors.ssid
                  ) ?
                    'border-error border-2'
                  : ''}"
                  bind:value={networkEditable.ssid}
                  id="ssid"
                  min="2"
                  max="32"
                  required
                />
                <label class="label" for="ssid">
                  <span class="label-text-alt text-error {formErrors.ssid ? '' : 'hidden'}"
                    >SSID must be between 3 and 32 characters long</span
                  >
                </label>
              </div>
              <div>
                <label class="label" for="pwd">
                  <span class="label-text text-md">Password</span>
                </label>
                <PasswordInput bind:value={networkEditable.password} id="pwd" />
              </div>
              <label
                class="label inline-flex cursor-pointer content-end justify-start gap-4 mt-2 sm:mb-4"
              >
                <input
                  type="checkbox"
                  bind:checked={static_ip_config}
                  class="checkbox checkbox-primary sm:-mb-5"
                />
                <span class="sm:-mb-5">Static IP Config?</span>
              </label>
            </div>
            {#if static_ip_config}
              <div
                class="grid w-full grid-cols-1 content-center gap-x-4 px-4 sm:grid-cols-2"
                transition:slide|local={{ duration: 300, easing: cubicOut }}
              >
                <div>
                  <label class="label" for="localIP">
                    <span class="label-text text-md">Local IP</span>
                  </label>
                  <input
                    type="text"
                    class="input input-bordered w-full {formErrors.local_ip ?
                      'border-error border-2'
                    : ''}"
                    minlength="7"
                    maxlength="15"
                    size="15"
                    bind:value={networkEditable.local_ip}
                    id="localIP"
                    required
                  />
                  <label class="label" for="localIP">
                    <span class="label-text-alt text-error {formErrors.local_ip ? '' : 'hidden'}"
                      >Must be a valid IPv4 address</span
                    >
                  </label>
                </div>

                <div>
                  <label class="label" for="gateway">
                    <span class="label-text text-md">Gateway IP</span>
                  </label>
                  <input
                    type="text"
                    class="input input-bordered w-full {formErrors.gateway_ip ?
                      'border-error border-2'
                    : ''}"
                    minlength="7"
                    maxlength="15"
                    size="15"
                    bind:value={networkEditable.gateway_ip}
                    required
                  />
                  <label class="label" for="gateway">
                    <span class="label-text-alt text-error {formErrors.gateway_ip ? '' : 'hidden'}"
                      >Must be a valid IPv4 address</span
                    >
                  </label>
                </div>
                <div>
                  <label class="label" for="subnet">
                    <span class="label-text text-md">Subnet Mask</span>
                  </label>
                  <input
                    type="text"
                    class="input input-bordered w-full {formErrors.subnet_mask ?
                      'border-error border-2'
                    : ''}"
                    minlength="7"
                    maxlength="15"
                    size="15"
                    bind:value={networkEditable.subnet_mask}
                    required
                  />
                  <label class="label" for="subnet">
                    <span
                      class="label-text-alt text-error {formErrors.subnet_mask ? '' : 'hidden'}"
                    >
                      Must be a valid IPv4 address
                    </span>
                  </label>
                </div>
                <div>
                  <label class="label" for="gateway">
                    <span class="label-text text-md">DNS 1</span>
                  </label>
                  <input
                    type="text"
                    class="input input-bordered w-full {formErrors.dns_1 ? 'border-error border-2'
                    : ''}"
                    minlength="7"
                    maxlength="15"
                    size="15"
                    bind:value={networkEditable.dns_ip_1}
                    required
                  />
                  <label class="label" for="gateway">
                    <span class="label-text-alt text-error {formErrors.dns_1 ? '' : 'hidden'}">
                      Must be a valid IPv4 address
                    </span>
                  </label>
                </div>
                <div>
                  <label class="label" for="subnet">
                    <span class="label-text text-md">DNS 2</span>
                  </label>
                  <input
                    type="text"
                    class="input input-bordered w-full {formErrors.dns_2 ? 'border-error border-2'
                    : ''}"
                    minlength="7"
                    maxlength="15"
                    size="15"
                    bind:value={networkEditable.dns_ip_2}
                    required
                  />
                  <label class="label" for="subnet">
                    <span class="label-text-alt text-error {formErrors.dns_2 ? '' : 'hidden'}">
                      Must be a valid IPv4 address
                    </span>
                  </label>
                </div>
              </div>
            {/if}
          {/if}

          <div class="divider mb-2 mt-0"></div>
          <div class="mx-4 flex flex-wrap justify-end gap-2">
            <button class="btn btn-primary" type="submit" disabled={!showNetworkEditor}>
              {newNetwork ? 'Add Network' : 'Update Network'}
            </button>
            <button class="btn btn-primary" type="button" onclick={validateHostName}>
              Apply Settings
            </button>
          </div>
        </form>
      </div>
    {/await}
  </div>
</SettingsCard>
