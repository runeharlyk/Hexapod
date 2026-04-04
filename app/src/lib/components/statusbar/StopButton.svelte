<script lang="ts">
  import { requestMode } from '$lib/control'
  import { MotionModes } from '$lib/motion'

  const deactivate = async () => {
    requestMode(MotionModes.DEACTIVATED)
  }
</script>

<button onclick={deactivate} class="bg-error text-white btn rounded-none">STOP</button>
