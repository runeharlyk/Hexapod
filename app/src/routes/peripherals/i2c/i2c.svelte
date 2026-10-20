<script lang="ts">
  import { onMount } from 'svelte'
  import Loading from '$lib/components/layout/Loading.svelte'
  import LoadFailure from '$lib/components/layout/LoadFailure.svelte'
  import Section from '$lib/components/layout/Section.svelte'
  import { dataBroker } from '$lib/transport/databroker'
  import { Connection } from '$lib/components/icons'
  import { describeI2C, hexAddress, i2cScan } from '$lib/i2c'
  import { ask } from '$lib/transport/reply'

  const scan = () =>
    i2cScan.run(async () => {
      const res = await ask(dataBroker, { i2cScanDataRequest: {} }, 'the I2C scan')
      return describeI2C(res.i2cScanData?.devices.map(device => device.address) ?? [])
    })

  onMount(() => {
    scan()
  })
</script>

<Section
  title="Devices on the bus"
  description="The robot probes addresses 0x08 to 0x77 on its bus."
  icon={Connection}
>
  {#snippet actions()}
    <button class="btn btn-primary btn-sm" onclick={scan} disabled={$i2cScan.state === 'scanning'}>
      {#if $i2cScan.state === 'scanning'}
        <span class="loading loading-spinner loading-xs"></span>
      {/if}
      Scan
    </button>
  {/snippet}

  {#if $i2cScan.state === 'scanning'}
    <Loading label="Scanning the bus" />
  {:else if $i2cScan.state === 'failed'}
    <LoadFailure message="Scan failed: {$i2cScan.message}" />
  {:else if $i2cScan.state === 'done' && $i2cScan.result.length === 0}
    <p class="text-base-content/70 text-sm">No device answered on the bus.</p>
  {:else if $i2cScan.state === 'done'}
    <ul class="list">
      {#each $i2cScan.result as device (device.address)}
        <li class="list-row items-center">
          <span class="badge badge-ghost font-mono">{hexAddress(device.address)}</span>
          <div class="min-w-0">
            <div class="font-medium">{device.part ?? 'Unknown part'}</div>
            <div class="text-base-content/70 text-sm">{device.name}</div>
          </div>
        </li>
      {/each}
    </ul>
  {/if}
</Section>
