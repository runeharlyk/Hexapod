<script lang="ts">
  import { onDestroy } from 'svelte'
  import { resolveUrl } from '$lib/proto-api'

  // Absolute URL so it works from the dev origin too; an <img> source isn't subject to CORS.
  let source = $state(resolveUrl('/api/camera/stream'))

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
