<script lang="ts">
  import { onDestroy } from 'svelte'
  import { imu } from '$lib/stores/imu'
  import { dataBroker } from '$lib/transport/databroker'
  import { IMUData } from '$lib/platform_shared/message'
  import { useFeatureFlags } from '$lib/stores/featureFlags'
  import { Compass as CompassIcon, Rotate3d, Temperature } from '$lib/components/icons'
  import Compass from '$lib/components/Compass.svelte'
  import LineChart from '$lib/components/LineChart.svelte'
  import Section from '$lib/components/layout/Section.svelte'
  import { notifications } from '$lib/components/toasts/notifications'
  import { calibrateImu } from '$lib/imu-calibration'

  const features = useFeatureFlags()

  // The IMU streams faster than a chart is worth redrawing.
  const REDRAW_MS = 200

  let calibrating = $state(false)

  const calibrate = async () => {
    calibrating = true
    const result = await calibrateImu()
    calibrating = false
    if (result.ok) notifications.success('IMU calibrated and saved.', 4000)
    else notifications.error(`IMU calibration failed: ${result.error}`, 6000)
  }

  onDestroy(
    dataBroker.on(IMUData, data => imu.addData({ ...data, altitude: 0, bmp_temp: 0, pressure: 0 }))
  )
</script>

{#if $features.imu}
  <Section
    title="Orientation"
    description="With the robot deactivated, lying still on a level floor, calibrating zeroes the gyro bias and levels the tilt, and keeps the result across restarts."
    icon={Rotate3d}
  >
    {#snippet actions()}
      <button class="btn btn-primary btn-sm" onclick={calibrate} disabled={calibrating}>
        {#if calibrating}<span class="loading loading-spinner loading-xs"></span>{/if}
        Calibrate
      </button>
    {/snippet}
    <LineChart
      unit="Angle [°]"
      series={[
        { label: 'x', colour: 'primary', data: $imu.x },
        { label: 'y', colour: 'secondary', data: $imu.y },
        { label: 'z', colour: 'accent', data: $imu.z }
      ]}
      range="fit"
      every={REDRAW_MS}
    />
  </Section>
{/if}
{#if $features.mag}
  <Section title="Compass" icon={CompassIcon}>
    {#if $imu.heading.length}
      <Compass heading={$imu.heading.at(-1)} />
    {:else}
      <p class="text-base-content/70 text-sm">No compass reading yet.</p>
    {/if}
  </Section>
{/if}
{#if $features.bmp}
  <Section title="Barometer" icon={Temperature}>
    <LineChart
      unit="Temperature [°C]"
      series={[{ label: 'Barometer temperature', colour: 'secondary', data: $imu.bmp_temp }]}
      range="fit"
      every={REDRAW_MS}
    />
    <LineChart
      unit="Altitude [m]"
      series={[{ label: 'Altitude', colour: 'primary', data: $imu.altitude }]}
      range="fit"
      every={REDRAW_MS}
    />
  </Section>
{/if}
