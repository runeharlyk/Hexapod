<script lang="ts">
  import type { Snippet } from 'svelte'
  import { resolve } from '$app/paths'
  import { link } from '$lib/stores/link'
  import { gateState } from '$lib/link-gate'
  import { Warning } from './icons'

  const { children }: { children: Snippet } = $props()

  let shownBefore = $state(false)
  const gate = $derived(gateState($link.status, shownBefore))

  $effect(() => {
    if (gate === 'show') shownBefore = true
  })
</script>

{#if gate === 'show'}
  {@render children()}
{:else if gate === 'connecting'}
  <div class="col-span-full flex w-full justify-center p-8">
    <span class="loading loading-spinner loading-md" aria-label="Connecting to the robot"></span>
  </div>
{:else}
  <div role="alert" class="alert alert-warning col-span-full">
    <Warning class="h-6 w-6 shrink-0" />
    <span>
      Not connected to the robot. Connect over WiFi, Bluetooth or USB from the
      <a class="link" href={resolve('/')}>home page</a> or the status bar to see these settings.
    </span>
  </div>
{/if}
