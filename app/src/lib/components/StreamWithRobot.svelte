<script lang="ts">
  import Stream from './Stream.svelte'
  import Visualization from './Visualization.svelte'

  // The camera picture with the robot's pose drawn over its bottom-right corner, so while driving by what the robot
  // sees its body and legs stay in view. On large screens it sits above the controller's keyboard hint.
  let { driveJoints = false }: { driveJoints?: boolean } = $props()
</script>

<div class="relative w-full h-full">
  <Stream />
  <div class="absolute bottom-4 lg:bottom-28 right-4 w-[28%] min-w-48 max-w-md aspect-4/3">
    <Visualization sky={false} ground={false} panel={false} {driveJoints} />
  </div>
</div>
