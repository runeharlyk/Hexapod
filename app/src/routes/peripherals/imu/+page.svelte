<script lang="ts">
  import RequiresRobot from '$lib/components/RequiresRobot.svelte'
  import RequiresFeatures from '$lib/components/RequiresFeatures.svelte'
  import IMU from './imu.svelte'
</script>

<div class="mx-0 my-1 flex flex-col space-y-4 sm:mx-8 sm:my-8">
  <RequiresRobot>
    <!-- The page draws its charts on mount into the canvases the flags decide on, so it mounts once they are in. -->
    <RequiresFeatures
      has={features => features.imu || features.mag || features.bmp}
      absent="This robot reports no IMU, compass or barometer."
    >
      <IMU />
    </RequiresFeatures>
  </RequiresRobot>
</div>
