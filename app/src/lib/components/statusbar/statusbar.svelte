<script lang="ts">
  import { page } from '$app/state'
  import UpdateIndicator from '$lib/components/statusbar/UpdateIndicator.svelte'
  import LinkIndicator from '$lib/components/statusbar/LinkIndicator.svelte'
  import FullscreenButton from './FullscreenButton.svelte'
  import ViewSelector from './ViewSelector.svelte'
  import { Hamburger } from '../icons'
</script>

<div class="navbar bg-base-300 sticky top-0 z-20 h-12 min-h-fit gap-1 px-2 drop-shadow-lg lg:h-16">
  <div class="flex min-w-0 flex-1 gap-2">
    <label
      for="main-menu"
      class="btn btn-ghost btn-circle btn-sm drawer-button"
      aria-label="Open menu"
    >
      <Hamburger class="h-6 w-auto" />
    </label>
    {#if page.data.title === 'Controller'}
      <ViewSelector />
    {:else}
      <h1 class="truncate px-2 text-xl font-bold lg:text-2xl">{page.data.title}</h1>
    {/if}
  </div>

  <UpdateIndicator />

  <LinkIndicator />

  <FullscreenButton />
</div>
