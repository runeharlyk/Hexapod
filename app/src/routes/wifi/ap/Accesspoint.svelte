<script lang="ts">
  import { PasswordInput } from '$lib/components/input'
  import { AP, Devices, Home, MAC, Settings as SettingsIcon } from '$lib/components/icons'
  import StatusItem from '$lib/components/StatusItem.svelte'
  import Advanced from '$lib/components/layout/Advanced.svelte'
  import Field from '$lib/components/layout/Field.svelte'
  import Section from '$lib/components/layout/Section.svelte'
  import SettingsForm from '$lib/components/layout/SettingsForm.svelte'
  import WatchedStatus from '$lib/components/layout/WatchedStatus.svelte'
  import { Settings } from '$lib/components/layout/settings.svelte'
  import { IPV4_HINT, IPV4_PATTERN } from '$lib/components/layout/validators'
  import { ipToString, ipToU32 } from '$lib/proto-api'
  import { dataBroker } from '$lib/transport/databroker'
  import { ask } from '$lib/transport/reply'
  import { APStatus, type APSettings } from '$lib/platform_shared/api'
  import { watch } from '$lib/robot/watch'

  // The form edits the addresses as text; the proto carries them as uint32.
  type ApForm = Omit<APSettings, 'localIp' | 'gatewayIp' | 'subnetMask'> & {
    localIp: string
    gatewayIp: string
    subnetMask: string
  }

  const toForm = (a: APSettings): ApForm => ({
    ...a,
    localIp: ipToString(a.localIp),
    gatewayIp: ipToString(a.gatewayIp),
    subnetMask: ipToString(a.subnetMask)
  })

  // The robot answers the subscribe with its current status and pushes every change.
  const apState = watch(APStatus)

  const STATUS: { variant: 'success' | 'error' | 'warning'; label: string }[] = [
    { variant: 'success', label: 'Active' },
    { variant: 'error', label: 'Inactive' },
    { variant: 'warning', label: 'Lingering' }
  ]

  const PROVISION_MODES = [
    { id: 0, text: 'Always' },
    { id: 1, text: 'When Wi-Fi is disconnected' },
    { id: 2, text: 'Never' }
  ]

  const settings = new Settings<ApForm>({
    what: 'the access point settings',
    read: async () => {
      const res = await ask(dataBroker, { apSettingsGet: {} }, 'the access point settings')
      if (!res.apSettings) throw new Error('the reply carried no settings')
      return toForm(res.apSettings)
    },
    // The robot answers a refused update with a 400 and the settings it kept, passwords left out.
    write: async value => {
      const res = await ask(
        dataBroker,
        {
          apSettingsUpdate: {
            provisionMode: Number(value.provisionMode),
            ssid: value.ssid,
            password: value.password,
            channel: Number(value.channel),
            ssidHidden: value.ssidHidden,
            maxClients: Number(value.maxClients),
            localIp: ipToU32(value.localIp),
            gatewayIp: ipToU32(value.gatewayIp),
            subnetMask: ipToU32(value.subnetMask)
          }
        },
        'the access point settings'
      )
      return res.apSettings && toForm(res.apSettings)
    },
    saved: 'Access Point settings updated.'
  })
</script>

<Section title="Status" icon={AP}>
  <WatchedStatus watched={$apState} what="the access point status" rows={4}>
    {#snippet children(status)}
      <ul class="list">
        <StatusItem
          icon={AP}
          title="Status"
          variant={STATUS[status.status]?.variant}
          description={STATUS[status.status]?.label}
        />
        <StatusItem icon={Home} title="IP address" description={ipToString(status.ipAddress)} />
        <StatusItem icon={MAC} title="MAC address" description={status.macAddress} />
        <StatusItem icon={Devices} title="Clients" description={status.stationNum} />
      </ul>
    {/snippet}
  </WatchedStatus>
</Section>

<SettingsForm title="Access point settings" icon={SettingsIcon} {settings} rows={6}>
  {#snippet children(form)}
    <Field label="Provide the access point" for="ap-mode">
      <select id="ap-mode" class="select w-full" bind:value={form.provisionMode}>
        {#each PROVISION_MODES as mode (mode.id)}
          <option value={mode.id}>{mode.text}</option>
        {/each}
      </select>
    </Field>
    <Field label="SSID" for="ap-ssid" hint="3 to 32 characters">
      <input
        id="ap-ssid"
        class="input validator w-full"
        type="text"
        required
        pattern={'.{3,32}'}
        maxlength="32"
        bind:value={form.ssid}
      />
    </Field>
    <Field label="Password" for="ap-password">
      <PasswordInput id="ap-password" placeholder="Unchanged" bind:value={form.password} />
    </Field>
    <Field label="Hide SSID" for="ap-hidden">
      <input id="ap-hidden" type="checkbox" class="toggle" bind:checked={form.ssidHidden} />
    </Field>
    <Field label="Preferred channel" for="ap-channel" hint="1 to 13">
      <input
        id="ap-channel"
        class="input validator w-full"
        type="number"
        required
        min="1"
        max="13"
        bind:value={form.channel}
      />
    </Field>
    <Field label="Max clients" for="ap-clients" hint="1 to 8">
      <input
        id="ap-clients"
        class="input validator w-full"
        type="number"
        required
        min="1"
        max="8"
        bind:value={form.maxClients}
      />
    </Field>
    <Advanced
      title="Addresses"
      description="Wrong addresses leave the access point unreachable until you correct them over another link."
    >
      <Field label="Local IP" for="ap-local-ip" hint={IPV4_HINT}>
        <input
          id="ap-local-ip"
          class="input validator w-full"
          type="text"
          required
          pattern={IPV4_PATTERN}
          bind:value={form.localIp}
        />
      </Field>
      <Field label="Gateway IP" for="ap-gateway" hint={IPV4_HINT}>
        <input
          id="ap-gateway"
          class="input validator w-full"
          type="text"
          required
          pattern={IPV4_PATTERN}
          bind:value={form.gatewayIp}
        />
      </Field>
      <Field label="Subnet mask" for="ap-subnet" hint={IPV4_HINT}>
        <input
          id="ap-subnet"
          class="input validator w-full"
          type="text"
          required
          pattern={IPV4_PATTERN}
          bind:value={form.subnetMask}
        />
      </Field>
    </Advanced>
  {/snippet}
</SettingsForm>
