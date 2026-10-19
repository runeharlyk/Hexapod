<script lang="ts">
  import { page } from '$app/state'
  import UpdateIndicator from '$lib/components/statusbar/UpdateIndicator.svelte'
  import LinkIndicator from '$lib/components/statusbar/LinkIndicator.svelte'
  import FullscreenButton from './FullscreenButton.svelte'
  import ViewSelector from './ViewSelector.svelte'
  import { Hamburger } from '../icons'
  import { handheldDriving } from '$lib/robot/state'
</script>

<div
  class="navbar bg-base-300/50 border-base-content/10 sticky top-0 z-20 h-12 min-h-fit gap-1 border-b px-2 backdrop-blur-md lg:h-16"
>
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

  {#if $handheldDriving}
    <span
      class="badge badge-warning badge-sm whitespace-nowrap"
      title="The robot ignores this app's sticks and refuses goals while the handheld controller drives"
    >
      Handheld driving
    </span>
  {/if}

  <UpdateIndicator />

  <LinkIndicator />

  <FullscreenButton />
</div>
