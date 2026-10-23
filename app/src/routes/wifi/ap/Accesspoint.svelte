<script lang="ts">
  import { IpInput, PasswordInput } from '$lib/components/input'
  import { AP, Devices, Home, MAC, Settings as SettingsIcon } from '$lib/components/icons'
  import StatusItem from '$lib/components/StatusItem.svelte'
  import Advanced from '$lib/components/layout/Advanced.svelte'
  import ChangedNote from '$lib/components/layout/ChangedNote.svelte'
  import Field from '$lib/components/layout/Field.svelte'
  import Section from '$lib/components/layout/Section.svelte'
  import SettingsForm from '$lib/components/layout/SettingsForm.svelte'
  import WatchedStatus from '$lib/components/layout/WatchedStatus.svelte'
  import { Settings } from '$lib/components/layout/settings.svelte'
  import { IPV4_HINT } from '$lib/components/layout/validators'
  import { ipToString } from '$lib/proto-api'
  import { APSettings, APStatus } from '$lib/platform_shared/api'
  import { topicSource } from '$lib/robot/settings-sources'
  import { watch } from '$lib/robot/watch'

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

  // The robot answers a refused update with a 400; the password it never sends is kept when sent back empty.
  const settings = new Settings<APSettings>(
    topicSource(APSettings, {
      what: 'the access point settings',
      saved: 'Access Point settings updated.',
      update: value => ({
        apSettingsUpdate: {
          ...value,
          provisionMode: Number(value.provisionMode),
          channel: Number(value.channel),
          maxClients: Number(value.maxClients)
        }
      }),
      together: []
    })
  )
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
  {#snippet children(form, notes)}
    <Field label="Provide the access point" for="ap-mode">
      <select
        id="ap-mode"
        name="provisionMode"
        class="select w-full"
        bind:value={form.provisionMode}
      >
        {#each PROVISION_MODES as mode (mode.id)}
          <option value={mode.id}>{mode.text}</option>
        {/each}
      </select>
      <ChangedNote
        {notes}
        field="provisionMode"
        format={value => PROVISION_MODES.find(mode => mode.id === value)?.text ?? String(value)}
      />
    </Field>
    <Field label="SSID" for="ap-ssid" hint="3 to 32 characters">
      <input
        id="ap-ssid"
        name="ssid"
        class="input validator w-full"
        type="text"
        required
        pattern={'.{3,32}'}
        maxlength="32"
        bind:value={form.ssid}
      />
      <ChangedNote {notes} field="ssid" />
    </Field>
    <Field label="Password" for="ap-password">
      <PasswordInput
        id="ap-password"
        name="password"
        placeholder="Unchanged"
        bind:value={form.password}
      />
    </Field>
    <Field label="Hide SSID" for="ap-hidden">
      <input
        id="ap-hidden"
        name="ssidHidden"
        type="checkbox"
        class="toggle"
        bind:checked={form.ssidHidden}
      />
      <ChangedNote {notes} field="ssidHidden" format={value => (value ? 'hidden' : 'visible')} />
    </Field>
    <Field label="Preferred channel" for="ap-channel" hint="1 to 13">
      <input
        id="ap-channel"
        name="channel"
        class="input validator w-full"
        type="number"
        required
        min="1"
        max="13"
        bind:value={form.channel}
      />
      <ChangedNote {notes} field="channel" />
    </Field>
    <Field label="Max clients" for="ap-clients" hint="1 to 8">
      <input
        id="ap-clients"
        name="maxClients"
        class="input validator w-full"
        type="number"
        required
        min="1"
        max="8"
        bind:value={form.maxClients}
      />
      <ChangedNote {notes} field="maxClients" />
    </Field>
    <Advanced
      title="Addresses"
      description="Wrong addresses leave the access point unreachable until you correct them over another link."
    >
      <Field label="Local IP" for="ap-local-ip" hint={IPV4_HINT}>
        <IpInput id="ap-local-ip" name="localIp" bind:value={form.localIp} />
        <ChangedNote {notes} field="localIp" format={value => ipToString(value as number)} />
      </Field>
      <Field label="Gateway IP" for="ap-gateway" hint={IPV4_HINT}>
        <IpInput id="ap-gateway" name="gatewayIp" bind:value={form.gatewayIp} />
        <ChangedNote {notes} field="gatewayIp" format={value => ipToString(value as number)} />
      </Field>
      <Field label="Subnet mask" for="ap-subnet" hint={IPV4_HINT}>
        <IpInput id="ap-subnet" name="subnetMask" bind:value={form.subnetMask} />
        <ChangedNote {notes} field="subnetMask" format={value => ipToString(value as number)} />
      </Field>
    </Advanced>
  {/snippet}
</SettingsForm>
