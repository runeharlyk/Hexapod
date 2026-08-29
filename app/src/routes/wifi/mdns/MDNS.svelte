<script lang="ts">
  import { onMount } from 'svelte'
  import SettingsCard from '$lib/components/SettingsCard.svelte'
  import { AP, Home, MAC, Devices } from '$lib/components/icons'
  import StatusItem from '$lib/components/StatusItem.svelte'
  import { cubicOut } from 'svelte/easing'
  import { slide } from 'svelte/transition'
  import { compareIp } from '$lib/utilities'
  import { dataBroker } from '$lib/transport/databroker'
  import type { MDNSQueryResult, MDNSStatus } from '$lib/platform_shared/api'

  let mdnsStatus: MDNSStatus | undefined = $state()
  let services: MDNSQueryResult[] = $state([])
  let isLoading = $state(false)

  const getMDNSStatus = async () => {
    const res = await dataBroker.request({ mdnsStatusGet: {} }).catch(() => null)
    if (res?.mdnsStatus) mdnsStatus = res.mdnsStatus
  }

  const queryMDNSServices = async () => {
    isLoading = true
    try {
      const res = await dataBroker.request({ mdnsQuery: { service: 'http', protocol: 'tcp' } })
      services = (res?.mdnsQueryResponse?.services ?? []).sort((a, b) => compareIp(a.ip, b.ip))
    } catch {
      services = []
    } finally {
      isLoading = false
    }
  }

  onMount(async () => {
    await getMDNSStatus()
    await queryMDNSServices()
  })

  const triggerScan = async () => {
    await queryMDNSServices()
  }
</script>

<SettingsCard collapsible={false}>
  {#snippet icon()}
    <AP class="lex-shrink-0 mr-2 h-6 w-6 self-end" />
  {/snippet}
  {#snippet title()}
    <span>MDNS</span>
  {/snippet}
  {#snippet right()}
    <button class="btn btn-primary" onclick={triggerScan} disabled={isLoading}>
      {#if isLoading}
        <span class="loading loading-ring loading-xs"></span>
      {:else}
        Scan
      {/if}
    </button>
  {/snippet}
  <div class="w-full overflow-x-auto">
    {#if mdnsStatus}
      <div
        class="flex w-full flex-col space-y-1"
        transition:slide|local={{ duration: 300, easing: cubicOut }}
      >
        <StatusItem icon={Home} title="IP Address" description={mdnsStatus.hostname} />

        <StatusItem icon={MAC} title="Instance" description={mdnsStatus.instance} />

        <StatusItem icon={Devices} title="Services" description={mdnsStatus.services.length} />

        <table class="table">
          <thead>
            <tr>
              <th></th>
              <th>Name</th>
              <th>Ip address</th>
              <th>Port</th>
            </tr>
          </thead>
          <tbody>
            {#each services as service}
              <tr>
                <td><Devices class="h-6 w-6" /></td>
                <td>{service.name}</td>
                <td>{service.ip}</td>
                <td>{service.port}</td>
              </tr>
            {/each}
          </tbody>
        </table>
      </div>
    {/if}
  </div>
</SettingsCard>
