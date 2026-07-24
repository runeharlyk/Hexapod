<script lang="ts">
  import { isFullscreen, toggleFullscreen } from '$lib/stores'
  import { MdiFullscreenExit, MdiFullscreen } from '../icons'

  const SvelteComponent = $derived($isFullscreen ? MdiFullscreenExit : MdiFullscreen)
</script>

<button
  class="btn btn-ghost btn-circle btn-sm"
  onclick={toggleFullscreen}
  aria-label={$isFullscreen ? 'Exit fullscreen' : 'Enter fullscreen'}
>
  <SvelteComponent class="h-6 w-6" />
</button>
