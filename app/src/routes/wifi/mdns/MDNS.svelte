<script lang="ts">
  import { onMount } from 'svelte'
  import StatusItem from '$lib/components/StatusItem.svelte'
  import LoadFailure from '$lib/components/layout/LoadFailure.svelte'
  import Loading from '$lib/components/layout/Loading.svelte'
  import Section from '$lib/components/layout/Section.svelte'
  import { AP, DNS, Devices, Remote, Server, info as Info } from '$lib/components/icons'
  import { dataBroker } from '$lib/transport/databroker'
  import type { MDNSStatus } from '$lib/platform_shared/api'
  import { ask } from '$lib/transport/reply'
  import {
    advertisedLabel,
    localName,
    mdnsScan,
    scanTypes,
    serviceTypeLabel,
    sortScanResults,
    txtLabel
  } from '$lib/mdns'

  let mdnsStatus: MDNSStatus | null | undefined = $state()
  let statusFailure = $state('')
  let selectedType = $state('_http._tcp')

  const types = $derived(scanTypes(mdnsStatus?.services ?? []))

  const runScan = () => {
    const target = types.find(t => serviceTypeLabel(t) === selectedType)
    if (!target) return
    const type = selectedType
    mdnsScan.run(async () => {
      const res = await ask(dataBroker, { mdnsQuery: target }, 'the mDNS scan')
      return { type, results: sortScanResults(res.mdnsQueryResponse?.services ?? []) }
    })
  }

  const readStatus = async () => {
    mdnsStatus = undefined
    const res = await ask(dataBroker, { mdnsStatusGet: {} }, 'the mDNS status').catch(
      (error: Error) => {
        statusFailure = error.message
        return null
      }
    )
    mdnsStatus = res?.mdnsStatus ?? null
  }

  onMount(() => {
    runScan()
    readStatus()
  })
</script>

<Section title="Status" icon={AP}>
  {#if mdnsStatus === undefined}
    <Loading label="Reading the mDNS status" rows={4} />
  {:else if mdnsStatus === null}
    <LoadFailure
      message="The robot did not report its mDNS status{statusFailure ? `: ${statusFailure}` : ''}."
      retry={readStatus}
    />
  {:else}
    <ul class="list">
      <StatusItem
        icon={AP}
        title="Status"
        variant={mdnsStatus.started ? 'success' : 'error'}
        description={mdnsStatus.started ? 'Advertising' : 'Not running'}
      />
      <StatusItem icon={DNS} title="Hostname" description={localName(mdnsStatus.hostname)} />
      <StatusItem icon={Server} title="Instance name" description={mdnsStatus.instance} />
      <StatusItem
        icon={Remote}
        title="Advertised services"
        description={mdnsStatus.services.map(advertisedLabel).join(', ') || 'None'}
      />
      {#if mdnsStatus.globalTxtRecords.length}
        <StatusItem
          icon={Info}
          title="TXT records"
          description={mdnsStatus.globalTxtRecords.map(txtLabel).join(', ')}
        />
      {/if}
    </ul>
  {/if}
</Section>

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
