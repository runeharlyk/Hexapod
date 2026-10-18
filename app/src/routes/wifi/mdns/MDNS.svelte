<script lang="ts">
  import { onMount } from 'svelte'
  import SettingsCard from '$lib/components/SettingsCard.svelte'
  import Spinner from '$lib/components/Spinner.svelte'
  import StatusItem from '$lib/components/StatusItem.svelte'
  import { AP, DNS, Devices, Remote, Server, info as Info } from '$lib/components/icons'
  import { dataBroker } from '$lib/transport/databroker'
  import type { MDNSQueryResult, MDNSStatus } from '$lib/platform_shared/api'
  import {
    advertisedLabel,
    localName,
    scanTypes,
    serviceTypeLabel,
    sortScanResults,
    txtLabel
  } from '$lib/mdns'

  type Scan =
    | { state: 'idle' }
    | { state: 'scanning' }
    | { state: 'done'; type: string; results: MDNSQueryResult[] }
    | { state: 'failed'; message: string }

  let mdnsStatus: MDNSStatus | null | undefined = $state()
  let scan: Scan = $state({ state: 'idle' })
  let selectedType = $state('_http._tcp')

  const types = $derived(scanTypes(mdnsStatus?.services ?? []))

  onMount(async () => {
    const res = await dataBroker.request({ mdnsStatusGet: {} }).catch(() => null)
    mdnsStatus = res?.mdnsStatus ?? null
  })

  const runScan = async () => {
    const target = types.find(t => serviceTypeLabel(t) === selectedType)
    if (!target) return
    scan = { state: 'scanning' }
    try {
      const res = await dataBroker.request({ mdnsQuery: target })
      scan = {
        state: 'done',
        type: selectedType,
        results: sortScanResults(res.mdnsQueryResponse?.services ?? [])
      }
    } catch (error) {
      scan = { state: 'failed', message: error instanceof Error ? error.message : String(error) }
    }
  }
</script>

<SettingsCard collapsible={false}>
  {#snippet icon()}
    <AP class="lex-shrink-0 mr-2 h-6 w-6 self-end" />
  {/snippet}
  {#snippet title()}
    <span>mDNS</span>
  {/snippet}

  {#if mdnsStatus === undefined}
    <Spinner />
  {:else if mdnsStatus === null}
    <div role="alert" class="alert alert-error">The robot did not report its mDNS status.</div>
  {:else}
    <div class="flex flex-col gap-1">
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
    </div>

    <div class="divider my-0"></div>

    <div class="flex flex-wrap items-end justify-between gap-2">
      <div>
        <h3 class="font-medium">Devices on the robot's network</h3>
        <p class="text-sm opacity-70">
          The robot asks its network for this service type, which takes about 3 seconds.
        </p>
      </div>
      <div class="flex gap-2">
        <select
          class="select select-sm w-36"
          aria-label="Service type"
          bind:value={selectedType}
          disabled={scan.state === 'scanning'}
        >
          {#each types as type (serviceTypeLabel(type))}
            <option value={serviceTypeLabel(type)}>{serviceTypeLabel(type)}</option>
          {/each}
        </select>
        <button
          class="btn btn-primary btn-sm"
          onclick={runScan}
          disabled={scan.state === 'scanning'}
        >
          {#if scan.state === 'scanning'}
            <span class="loading loading-ring loading-xs"></span>
          {/if}
          Scan
        </button>
      </div>
    </div>

    {#if scan.state === 'failed'}
      <div role="alert" class="alert alert-error">Scan failed: {scan.message}</div>
    {:else if scan.state === 'done' && scan.results.length === 0}
      <p class="text-sm opacity-70">No {scan.type} services answered.</p>
    {:else if scan.state === 'done'}
      <div class="w-full overflow-x-auto">
        <table class="table table-sm">
          <thead>
            <tr>
              <th></th>
              <th>Host</th>
              <th>IP address</th>
              <th>Port</th>
            </tr>
          </thead>
          <tbody>
            {#each scan.results as service, index (index)}
              <tr>
                <td><Devices class="h-5 w-5" /></td>
                <td>{service.name ? localName(service.name) : '-'}</td>
                <td class="font-mono">{service.ip || '-'}</td>
                <td class="font-mono">{service.port}</td>
              </tr>
            {/each}
          </tbody>
        </table>
      </div>
    {/if}
  {/if}
</SettingsCard>
