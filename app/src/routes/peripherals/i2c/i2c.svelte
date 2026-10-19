<script lang="ts">
  import { onMount } from 'svelte'
  import SettingsCard from '$lib/components/SettingsCard.svelte'
  import { dataBroker } from '$lib/transport/databroker'
  import { Connection } from '$lib/components/icons'
  import { describeI2C, hexAddress, i2cScan } from '$lib/i2c'
  import { answered } from '$lib/scan'
  import I2CSetting from './i2cSetting.svelte'

  const scan = () =>
    i2cScan.run(async () => {
      const res = answered(await dataBroker.request({ i2cScanDataRequest: {} }))
      return describeI2C(res.i2cScanData?.devices.map(device => device.address) ?? [])
    })

  onMount(() => {
    scan()
  })
</script>

<SettingsCard collapsible={false}>
  {#snippet icon()}
    <Connection class="lex-shrink-0 mr-2 h-6 w-6 self-end" />
  {/snippet}
  {#snippet title()}
    <span>I<sup>2</sup>C</span>
  {/snippet}
  {#snippet right()}
    <button class="btn btn-primary" onclick={scan} disabled={$i2cScan.state === 'scanning'}>
      {#if $i2cScan.state === 'scanning'}
        <span class="loading loading-ring loading-xs"></span>
      {/if}
      Scan
    </button>
  {/snippet}

  {#if $i2cScan.state === 'scanning'}
    <p class="text-sm opacity-70">Probing addresses 0x08 to 0x77 on the robot's bus.</p>
  {:else if $i2cScan.state === 'failed'}
    <div role="alert" class="alert alert-error">Scan failed: {$i2cScan.message}</div>
  {:else if $i2cScan.state === 'done' && $i2cScan.result.length === 0}
    <p class="text-sm opacity-70">No device answered on the bus.</p>
  {:else if $i2cScan.state === 'done'}
    <div class="w-full overflow-x-auto">
      <table class="table table-sm">
        <thead>
          <tr>
            <th>Address</th>
            <th>Part</th>
            <th>Description</th>
          </tr>
        </thead>
        <tbody>
          {#each $i2cScan.result as device (device.address)}
            <tr>
              <td class="font-mono">{hexAddress(device.address)}</td>
              <td>{device.part ?? '-'}</td>
              <td>{device.name}</td>
            </tr>
          {/each}
        </tbody>
      </table>
    </div>
  {/if}

  <I2CSetting />
</SettingsCard>
