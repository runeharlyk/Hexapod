<script lang="ts">
  import type { Snippet } from 'svelte'
  import {
    featureFlagsState,
    refreshFeatureFlags,
    useFeatureFlags,
    type Features
  } from '$lib/stores/featureFlags'

  // Shows its children once the robot's feature flags are in and `has` finds the feature among them. Until the flags
  // arrive every one reads false, which says nothing about the robot, so the page waits rather than deciding on that;
  // when the read fails it says why and offers another, which a link coming back also starts.
  const {
    has,
    absent,
    children
  }: { has: (features: Features) => boolean; absent: string; children: Snippet } = $props()

  const features = useFeatureFlags()

  const retry = () => refreshFeatureFlags().catch(() => {})
</script>

{#if $featureFlagsState.kind === 'failed'}
  <div role="alert" class="alert alert-error w-full max-w-2xl self-center">
    <span>The robot's features could not be read: {$featureFlagsState.message}</span>
    <button class="btn btn-sm" onclick={retry}>Retry</button>
  </div>
{:else if $featureFlagsState.kind === 'pending'}
  <div class="flex w-full justify-center p-8">
    <span class="loading loading-spinner loading-md" aria-label="Reading the robot's features"
    ></span>
  </div>
{:else if has($features)}
  {@render children()}
{:else}
  <div role="alert" class="alert alert-info w-full max-w-2xl self-center">
    <span>{absent}</span>
  </div>
{/if}
