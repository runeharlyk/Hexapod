<script lang="ts">
  import { preventDefault } from 'svelte/legacy'

  import { onMount } from 'svelte'
  import { slide } from 'svelte/transition'
  import { cubicOut } from 'svelte/easing'
  import { PasswordInput } from '$lib/components/input'
  import SettingsCard from '$lib/components/SettingsCard.svelte'
  import { notifications } from '$lib/components/toasts/notifications'
  import Spinner from '$lib/components/Spinner.svelte'
  import { useFeatureFlags } from '$lib/stores'
  import { AP, Devices, Home, MAC } from '$lib/components/icons'
  import StatusItem from '$lib/components/StatusItem.svelte'
  import { ipToString, ipToU32 } from '$lib/proto-api'
  import { dataBroker } from '$lib/transport/databroker'
  import { APStatus } from '$lib/platform_shared/api'

  const features = useFeatureFlags()

  let apSettings: any = $state()
  let apStatus: any = $state()

  let formField: any = $state()

  const applyApStatus = (s: any) => {
    apStatus = {
      status: s.status,
      ip_address: ipToString(s.ipAddress),
      mac_address: s.macAddress,
      station_num: s.stationNum
    }
  }

  async function getAPStatus() {
    const res = await dataBroker.request({ apStatusGet: {} }).catch(() => null)
    if (res?.apStatus) applyApStatus(res.apStatus)
    return apStatus
  }

  async function getAPSettings() {
    const res = await dataBroker.request({ apSettingsGet: {} }).catch(() => null)
    if (!res?.apSettings) return
    const a = res.apSettings
    apSettings = {
      provision_mode: a.provisionMode,
      ssid: a.ssid,
      password: a.password,
      channel: a.channel,
      max_clients: a.maxClients,
      ssid_hidden: a.ssidHidden,
      local_ip: ipToString(a.localIp),
      gateway_ip: ipToString(a.gatewayIp),
      subnet_mask: ipToString(a.subnetMask)
    }
    return apSettings
  }

  onMount(() => {
    getAPSettings()
    getAPStatus() // current value now; live changes via the subscription below
    return dataBroker.on(APStatus, applyApStatus)
  })

  let provisionMode = [
    {
      id: 0,
      text: `Always`
    },
    {
      id: 1,
      text: `When WiFi Disconnected`
    },
    {
      id: 2,
      text: `Never`
    }
  ]

  type Variant = 'success' | 'error' | 'primary' | 'info' | 'warning'

  let apStatusVariant: Variant[] = ['success', 'error', 'warning']

  let apStatusDescription = ['Active', 'Inactive', 'Lingering']

  let formErrors = $state({
    ssid: false,
    channel: false,
    max_clients: false,
    local_ip: false,
    gateway_ip: false,
    subnet_mask: false
  })

  async function postAPSettings() {
    try {
      const res = await dataBroker.request({
        apSettingsUpdate: {
          provisionMode: Number(apSettings.provision_mode),
          ssid: apSettings.ssid,
          password: apSettings.password,
          channel: Number(apSettings.channel),
          ssidHidden: apSettings.ssid_hidden,
          maxClients: Number(apSettings.max_clients),
          localIp: ipToU32(apSettings.local_ip),
          gatewayIp: ipToU32(apSettings.gateway_ip),
          subnetMask: ipToU32(apSettings.subnet_mask)
        }
      })
      if (!res.apSettings) {
        notifications.error('Failed to update Access Point settings.', 3000)
        return
      }
      notifications.success('Access Point settings updated.', 3000)
    } catch {
      notifications.error('Failed to update Access Point settings — is the robot connected?', 3000)
    }
  }

  function handleSubmitAP() {
    let valid = true

    if (apSettings.ssid.length < 3 || apSettings.ssid.length > 32) {
      valid = false
      formErrors.ssid = true
    } else {
      formErrors.ssid = false
    }

    let channel = Number(apSettings.channel)
    if (1 > channel || channel > 13) {
      valid = false
      formErrors.channel = true
    } else {
      formErrors.channel = false
    }

    let maxClients = Number(apSettings.max_clients)
    if (1 > maxClients || maxClients > 8) {
      valid = false
      formErrors.max_clients = true
    } else {
      formErrors.max_clients = false
    }

    const regexExp =
      /\b(?:(?:2(?:[0-4][0-9]|5[0-5])|[0-1]?[0-9]?[0-9])\.){3}(?:(?:2([0-4][0-9]|5[0-5])|[0-1]?[0-9]?[0-9]))\b/

    if (!regexExp.test(apSettings.gateway_ip)) {
      valid = false
      formErrors.gateway_ip = true
    } else {
      formErrors.gateway_ip = false
    }

    if (!regexExp.test(apSettings.subnet_mask)) {
      valid = false
      formErrors.subnet_mask = true
    } else {
      formErrors.subnet_mask = false
    }

    if (!regexExp.test(apSettings.local_ip)) {
      valid = false
      formErrors.local_ip = true
    } else {
      formErrors.local_ip = false
    }

    if (valid) {
      postAPSettings()
    }
  }
</script>

<SettingsCard collapsible={false}>
  {#snippet icon()}
    <AP class="lex-shrink-0 mr-2 h-6 w-6 self-end" />
  {/snippet}
  {#snippet title()}
    <span>Access Point</span>
  {/snippet}
  <div class="w-full overflow-x-auto">
    {#await getAPStatus()}
      <Spinner />
    {:then nothing}
      <div
        class="flex w-full flex-col space-y-1"
        transition:slide|local={{ duration: 300, easing: cubicOut }}
      >
        <StatusItem
          icon={AP}
          title="Status"
          variant={apStatusVariant[apStatus.status]}
          description={apStatusDescription[apStatus.status]}
        />

        <StatusItem icon={Home} title="IP Address" description={apStatus.ip_address} />

        <StatusItem icon={MAC} title="MAC Address" description={apStatus.mac_address} />

        <StatusItem icon={Devices} title="AP Clients" description={apStatus.station_num} />
      </div>
    {/await}
  </div>

  <div class="bg-base-200 relative grid w-full max-w-2xl self-center overflow-hidden">
    <div
      class="min-h-16 flex w-full items-center justify-between space-x-3 p-0 text-xl font-medium"
    >
      Change AP Settings
    </div>
    {#await getAPSettings()}
      <Spinner />
    {:then nothing}
      <div
        class="flex flex-col gap-2 p-0"
        transition:slide|local={{ duration: 300, easing: cubicOut }}
      >
        <form
          class="grid w-full grid-cols-1 content-center gap-x-4 p-0s sm:grid-cols-2"
          onsubmit={preventDefault(handleSubmitAP)}
          novalidate
          bind:this={formField}
        >
          <div>
            <label class="label" for="apmode">
              <span class="label-text">Provide Access Point ...</span>
            </label>
            <select
              class="select select-bordered w-full"
              id="apmode"
              bind:value={apSettings.provision_mode}
            >
              {#each provisionMode as mode}
                <option value={mode.id}>
                  {mode.text}
                </option>
              {/each}
            </select>
          </div>
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
              bind:value={apSettings.ssid}
              id="ssid"
              min="2"
              max="32"
              required
            />
            <label class="label" for="ssid">
              <span class="label-text-alt text-error {formErrors.ssid ? '' : 'hidden'}"
                >SSID must be between 2 and 32 characters long</span
              >
            </label>
          </div>

          <div>
            <label class="label" for="pwd">
              <span class="label-text text-md">Password</span>
            </label>
            <PasswordInput bind:value={apSettings.password} id="pwd" />
          </div>
          <div>
            <label class="label" for="channel">
              <span class="label-text text-md">Preferred Channel</span>
            </label>
            <input
              type="number"
              min="1"
              max="13"
              class="input input-bordered invalid:border-error w-full invalid:border-2 {(
                formErrors.channel
              ) ?
                'border-error border-2'
              : ''}"
              bind:value={apSettings.channel}
              id="channel"
              required
            />
            <label class="label" for="channel">
              <span class="label-text-alt text-error {formErrors.channel ? '' : 'hidden'}"
                >Must be channel 1 to 13</span
              >
            </label>
          </div>

          <div>
            <label class="label" for="clients">
              <span class="label-text text-md">Max Clients</span>
            </label>
            <input
              type="number"
              min="1"
              max="8"
              class="input input-bordered invalid:border-error w-full invalid:border-2 {(
                formErrors.max_clients
              ) ?
                'border-error border-2'
              : ''}"
              bind:value={apSettings.max_clients}
              id="clients"
              required
            />
            <label class="label" for="clients">
              <span class="label-text-alt text-error {formErrors.max_clients ? '' : 'hidden'}"
                >Maximum 8 clients allowed</span
              >
            </label>
          </div>

          <div>
            <label class="label" for="localIP">
              <span class="label-text text-md">Local IP</span>
            </label>
            <input
              type="text"
              class="input input-bordered w-full {formErrors.local_ip ? 'border-error border-2' : (
                ''
              )}"
              minlength="7"
              maxlength="15"
              size="15"
              bind:value={apSettings.local_ip}
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
              class="input input-bordered w-full {formErrors.gateway_ip ? 'border-error border-2'
              : ''}"
              minlength="7"
              maxlength="15"
              size="15"
              bind:value={apSettings.gateway_ip}
              id="gateway"
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
              class="input input-bordered w-full {formErrors.subnet_mask ? 'border-error border-2'
              : ''}"
              minlength="7"
              maxlength="15"
              size="15"
              bind:value={apSettings.subnet_mask}
              id="subnet"
              required
            />
            <label class="label" for="subnet">
              <span class="label-text-alt text-error {formErrors.subnet_mask ? '' : 'hidden'}"
                >Must be a valid IPv4 address</span
              >
            </label>
          </div>

          <label class="label my-auto cursor-pointer justify-start gap-4">
            <input
              type="checkbox"
              bind:checked={apSettings.ssid_hidden}
              class="checkbox checkbox-primary"
            />
            <span class="">Hide SSID</span>
          </label>

          <div class="place-self-end">
            <button class="btn btn-primary" type="submit">Apply Settings</button>
          </div>
        </form>
      </div>
    {/await}
  </div>
</SettingsCard>
