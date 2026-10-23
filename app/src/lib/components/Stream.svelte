<script lang="ts">
  import { onDestroy } from 'svelte'
  import { resolveUrl } from '$lib/proto-api'
  import { robotHost } from '$lib/stores/robot-host'

  // Absolute URL so it works from the dev origin too, at the host the socket reached, which is the saved robot's IP
  // while its name does not open; an <img> source isn't subject to CORS.
  let source = $derived(resolveUrl('/api/camera/stream', $robotHost))

  onDestroy(() => (source = '#'))
</script>

<div class="w-full h-full">
  <img
    src={source}
    class="absolute object-cover blur-3xl w-full h-full -z-10"
    alt="Live stream is down"
  />
  <img src={source} class="object-contain w-full h-full" alt="Live stream is down" />
</div>
