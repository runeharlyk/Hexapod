<script lang="ts">
  import Stream from './Stream.svelte'
  import Visualization from './Visualization.svelte'

  // The camera picture with the robot's pose in a corner, so while driving by what the robot sees its body and legs
  // stay in view.
</script>

<div class="relative w-full h-full">
  <Stream />
  <div
    class="absolute top-4 right-4 w-[28%] min-w-48 max-w-md aspect-4/3 rounded-box overflow-hidden shadow-lg bg-base-300/70"
  >
    <Visualization sky={false} panel={false} />
  </div>
</div>
