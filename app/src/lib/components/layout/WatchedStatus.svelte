<script lang="ts" generics="T">
  import type { Snippet } from 'svelte'
  import type { Watched } from '$lib/transport/topics'
  import { describeFailure } from '$lib/transport/reply'
  import LoadFailure from './LoadFailure.svelte'
  import Loading from './Loading.svelte'

  // A state topic of the robot (robot/watch.ts) as a list of readings: placeholders until its first value, the failure
  // when none came, and a note while the value shown is the last one before the link dropped.
  interface Props {
    watched: Watched<T>
    /** What the topic is, as the failure names it: "Could not read {what}". */
    what: string
    rows?: number
    /** The value as a `list` of readings (StatusItem). */
    children: Snippet<[T]>
  }

  const { watched, what, rows = 3, children }: Props = $props()
</script>

{#if watched.value === undefined && watched.failure}
  <LoadFailure message="Could not read {what}: {describeFailure(watched.failure, what)}" />
{:else if watched.value === undefined}
  <Loading label="Reading {what}" {rows} />
{:else}
  {#if watched.stale}
    <p class="text-warning text-sm">The robot is not connected: this is the last status it sent.</p>
  {/if}
  {@render children(watched.value)}
{/if}
