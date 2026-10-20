<script lang="ts">
  import RequiresRobot from '$lib/components/RequiresRobot.svelte'
  import RequiresFeatures from '$lib/components/RequiresFeatures.svelte'
  import PageShell from '$lib/components/layout/PageShell.svelte'
  import IMU from './imu.svelte'
</script>

<PageShell>
  <RequiresRobot>
    <!-- The page shows the parts the flags name, so it waits for the flags. -->
    <RequiresFeatures
      has={features => features.imu || features.mag || features.bmp}
      absent="This robot reports no IMU, compass or barometer."
    >
      <IMU />
    </RequiresFeatures>
  </RequiresRobot>
</PageShell>
