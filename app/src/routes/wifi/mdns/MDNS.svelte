<script lang="ts">
  import { onMount } from 'svelte'
  import ChangedNote from '$lib/components/layout/ChangedNote.svelte'
  import Field from '$lib/components/layout/Field.svelte'
  import LoadFailure from '$lib/components/layout/LoadFailure.svelte'
  import Loading from '$lib/components/layout/Loading.svelte'
  import Section from '$lib/components/layout/Section.svelte'
  import SettingsForm from '$lib/components/layout/SettingsForm.svelte'
  import { Settings } from '$lib/components/layout/settings.svelte'
  import { Add, DNS, Delete, Devices } from '$lib/components/icons'
  import { dataBroker } from '$lib/transport/databroker'
  import { MDNSSettings, type MDNSServiceDef, type MDNSTxtRecord } from '$lib/platform_shared/api'
  import { topicSource } from '$lib/robot/settings-sources'
  import { ask } from '$lib/transport/reply'
  import {
    HOSTNAME_PATTERN,
    MDNS_LIMITS,
    advertisedLabel,
    localName,
    mdnsScan,
    scanTypes,
    serviceTypeLabel,
    sortScanResults,
    txtLabel
  } from '$lib/mdns'

  const SERVICE_PATTERN = String.raw`_[a-z0-9\-]{1,${MDNS_LIMITS.serviceType - 1}}`
  const PROTOCOL_PATTERN = '_(?:tcp|udp)'

  const source = topicSource(MDNSSettings, {
    what: 'the mDNS settings',
    saved: 'mDNS settings saved.',
    update: value => ({ mdnsSettingsUpdate: value }),
    together: [['services'], ['globalTxtRecords']]
  })
  const settings = new Settings<MDNSSettings>(source)
  const robot = source.watched

  const blankService = (): MDNSServiceDef => ({
    service: '_http',
    protocol: '_tcp',
    port: 80,
    txtRecords: []
  })
  const blankRecord = (): MDNSTxtRecord => ({ key: '', value: '' })

  const servicesLabel = (list: unknown) =>
    (list as MDNSServiceDef[]).map(advertisedLabel).join(', ') || 'none'
  const recordsLabel = (list: unknown) =>
    (list as MDNSTxtRecord[]).map(txtLabel).join(', ') || 'none'

  let selectedType = $state('_http._tcp')

  const types = $derived(scanTypes($robot.value?.services ?? []))

  const runScan = () => {
    const target = types.find(t => serviceTypeLabel(t) === selectedType)
    if (!target) return
    const type = selectedType
    mdnsScan.run(async () => {
      const res = await ask(dataBroker, { mdnsQuery: target }, 'the mDNS scan')
      return { type, results: sortScanResults(res.mdnsQueryResponse?.services ?? []) }
    })
  }

  onMount(runScan)
</script>

<SettingsForm
  title="Name and services"
  description="How the robot announces itself on the network. A new host name answers new lookups within moments; a connection already open keeps working."
  icon={DNS}
  {settings}
  rows={2}
>
  {#snippet children(form, notes)}
    <Field
      label="Host name"
      for="mdns-hostname"
      hint="Up to {MDNS_LIMITS.hostname} characters: a-z, 0-9 and -, none at either end. Reached as {localName(
        form.hostname || '...'
      )}"
    >
      <input
        id="mdns-hostname"
        name="hostname"
        class="input validator w-full"
        type="text"
        required
        pattern={HOSTNAME_PATTERN}
        maxlength={MDNS_LIMITS.hostname}
        bind:value={form.hostname}
      />
      <ChangedNote {notes} field="hostname" />
    </Field>
    <Field label="Instance name" for="mdns-instance" hint="Up to {MDNS_LIMITS.instance} characters">
      <input
        id="mdns-instance"
        name="instance"
        class="input validator w-full"
        type="text"
        required
        maxlength={MDNS_LIMITS.instance}
        bind:value={form.instance}
      />
      <ChangedNote {notes} field="instance" />
    </Field>

    <div class="grid gap-2">
      <div class="flex items-center justify-between gap-2">
        <h3 class="text-sm font-medium">Advertised services</h3>
        <button
          type="button"
          class="btn btn-sm"
          disabled={form.services.length >= MDNS_LIMITS.services}
          onclick={() => form.services.push(blankService())}
        >
          <Add class="size-4" />Add service
        </button>
      </div>
      {#each form.services as service, index (index)}
        <div class="border-base-300 rounded-box grid gap-2 border p-3">
          <div class="flex flex-wrap items-center gap-2">
            <input
              name="services.{index}.service"
              aria-label="Service {index + 1} type"
              title="A service type such as _http"
              class="input input-sm validator w-36"
              required
              pattern={SERVICE_PATTERN}
              maxlength={MDNS_LIMITS.serviceType}
              bind:value={service.service}
            />
            <input
              name="services.{index}.protocol"
              aria-label="Service {index + 1} protocol"
              title="_tcp or _udp"
              class="input input-sm validator w-20"
              required
              pattern={PROTOCOL_PATTERN}
              maxlength={MDNS_LIMITS.protocol}
              bind:value={service.protocol}
            />
            <input
              name="services.{index}.port"
              aria-label="Service {index + 1} port"
              title="1 to 65535"
              type="number"
              class="input input-sm validator w-24"
              required
              min="1"
              max="65535"
              step="1"
              bind:value={service.port}
            />
            <button
              type="button"
              class="btn btn-sm btn-square btn-ghost ml-auto"
              aria-label="Remove service {index + 1}"
              onclick={() => form.services.splice(index, 1)}
            >
              <Delete class="text-error size-4" />
            </button>
          </div>
          {#each service.txtRecords as record, recordIndex (recordIndex)}
            <div class="flex flex-wrap items-center gap-2">
              <input
                name="services.{index}.txtRecords.{recordIndex}.key"
                aria-label="Service {index + 1} TXT key"
                placeholder="TXT key"
                class="input input-sm validator w-36"
                required
                maxlength={MDNS_LIMITS.txtKey}
                bind:value={record.key}
              />
              <input
                name="services.{index}.txtRecords.{recordIndex}.value"
                aria-label="Service {index + 1} TXT value"
                placeholder="TXT value"
                class="input input-sm validator w-48"
                maxlength={MDNS_LIMITS.txtValue}
                bind:value={record.value}
              />
              <button
                type="button"
                class="btn btn-sm btn-square btn-ghost"
                aria-label="Remove service {index + 1} TXT record"
                onclick={() => service.txtRecords.splice(recordIndex, 1)}
              >
                <Delete class="text-error size-4" />
              </button>
            </div>
          {/each}
          {#if service.txtRecords.length < MDNS_LIMITS.txtPerService}
            <button
              type="button"
              class="btn btn-sm btn-ghost w-fit"
              onclick={() => service.txtRecords.push(blankRecord())}
            >
              <Add class="size-4" />Add TXT record
            </button>
          {/if}
        </div>
      {:else}
        <p class="text-base-content/70 text-sm">No services advertised.</p>
      {/each}
      <ChangedNote {notes} field="services" format={servicesLabel} />
    </div>

    <div class="grid gap-2">
      <div class="flex items-center justify-between gap-2">
        <h3 class="text-sm font-medium">TXT records</h3>
        <button
          type="button"
          class="btn btn-sm"
          disabled={form.globalTxtRecords.length >= MDNS_LIMITS.globalTxt}
          onclick={() => form.globalTxtRecords.push(blankRecord())}
        >
          <Add class="size-4" />Add record
        </button>
      </div>
      {#each form.globalTxtRecords as record, index (index)}
        <div class="flex flex-wrap items-center gap-2">
          <input
            name="globalTxtRecords.{index}.key"
            aria-label="TXT record {index + 1} key"
            placeholder="Key"
            class="input input-sm validator w-36"
            required
            maxlength={MDNS_LIMITS.txtKey}
            bind:value={record.key}
          />
          <input
            name="globalTxtRecords.{index}.value"
            aria-label="TXT record {index + 1} value"
            placeholder="Value"
            class="input input-sm validator w-48"
            maxlength={MDNS_LIMITS.txtValue}
            bind:value={record.value}
          />
          <button
            type="button"
            class="btn btn-sm btn-square btn-ghost"
            aria-label="Remove TXT record {index + 1}"
            onclick={() => form.globalTxtRecords.splice(index, 1)}
          >
            <Delete class="text-error size-4" />
          </button>
        </div>
      {:else}
        <p class="text-base-content/70 text-sm">No TXT records beyond the firmware version.</p>
      {/each}
      <ChangedNote {notes} field="globalTxtRecords" format={recordsLabel} />
    </div>
  {/snippet}
</SettingsForm>

<Section
  title="Devices on the robot's network"
  description="The robot asks its network for this service type, which takes about 3 seconds."
  icon={Devices}
>
  {#snippet actions()}
    <select
      class="select select-sm w-36"
      aria-label="Service type"
      bind:value={selectedType}
      disabled={$mdnsScan.state === 'scanning'}
    >
      {#each types as type (serviceTypeLabel(type))}
        <option value={serviceTypeLabel(type)}>{serviceTypeLabel(type)}</option>
      {/each}
    </select>
    <button
      class="btn btn-primary btn-sm"
      onclick={runScan}
      disabled={$mdnsScan.state === 'scanning'}
    >
      {#if $mdnsScan.state === 'scanning'}
        <span class="loading loading-spinner loading-xs"></span>
      {/if}
      Scan
    </button>
  {/snippet}

  {#if $mdnsScan.state === 'scanning'}
    <Loading label="Scanning the network" rows={2} />
  {:else if $mdnsScan.state === 'failed'}
    <LoadFailure message="Scan failed: {$mdnsScan.message}" />
  {:else if $mdnsScan.state === 'done' && $mdnsScan.result.results.length === 0}
    <p class="text-base-content/70 text-sm">No {$mdnsScan.result.type} services answered.</p>
  {:else if $mdnsScan.state === 'done'}
    <div class="overflow-x-auto">
      <table class="table table-sm">
        <thead>
          <tr>
            <th>Host</th>
            <th>IP address</th>
            <th>Port</th>
          </tr>
        </thead>
        <tbody>
          {#each $mdnsScan.result.results as service, index (index)}
            <tr>
              <td class="break-all">{service.name ? localName(service.name) : '-'}</td>
              <td class="font-mono">{service.ip || '-'}</td>
              <td class="font-mono">{service.port}</td>
            </tr>
          {/each}
        </tbody>
      </table>
    </div>
  {/if}
</Section>
