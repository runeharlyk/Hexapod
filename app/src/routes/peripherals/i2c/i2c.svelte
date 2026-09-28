<script lang="ts">
  import SettingsCard from '$lib/components/SettingsCard.svelte'
  import { onMount } from 'svelte'
  import { dataBroker } from '$lib/transport/databroker'
  import type { I2CDevice } from '$lib/types/models'
  import { Connection } from '$lib/components/icons'
  import I2CSetting from './i2cSetting.svelte'
  import { notifications } from '$lib/components/toasts/notifications'

  const i2cDevices = [
    { address: 30, part_number: 'HMC5883', name: '3-Axis Digital Compass/Magnetometer IC' },
    { address: 64, part_number: 'PCA9685', name: '16-channel PWM driver default address' },
    { address: 65, part_number: 'PCA9685', name: '16-channel PWM driver (A0 set)' },
    { address: 72, part_number: 'ADS1115', name: '4-channel 16-bit ADC' },
    {
      address: 104,
      part_number: 'MPU6050',
      name: 'Six-Axis (Gyro + Accelerometer) MEMS MotionTracking™ Devices'
    },
    { address: 119, part_number: 'BMP085', name: 'Temp/Barometric' }
  ]

  let active_devices: I2CDevice[] = $state([])

  let isLoading = $state(false)

  onMount(() => {
    triggerScan()
  })

  const triggerScan = async () => {
    isLoading = true
    try {
      const res = await dataBroker.request({ i2cScanDataRequest: {} })
      const devices = res.i2cScanData?.devices ?? []
      active_devices = devices.map(
        ({ address }) =>
          i2cDevices.find(device => device.address === address) || {
            address,
            part_number: 'Unknown',
            name: 'Unknown'
          }
      )
    } catch (error) {
      notifications.error(
        `I2C scan failed: ${error instanceof Error ? error.message : error}`,
        4000
      )
    } finally {
      isLoading = false
    }
  }
</script>

<SettingsCard collapsible={false}>
  {#snippet icon()}
    <Connection class="lex-shrink-0 mr-2 h-6 w-6 self-end" />
  {/snippet}
  {#snippet title()}
    <span>I<sup>2</sup>C</span>
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

  <div class="grid">
    {#if active_devices.length === 0}
      <div>No I2C devices found</div>
    {:else}
      {#each active_devices as device}
        <div>[{device.address.toString(16)}] {device.part_number} - {device.name}</div>
      {/each}
    {/if}
  </div>
</SettingsCard>

<I2CSetting />
