<script lang="ts">
  // The simulation's code, physics engine and virtual robot load only when this view is shown. The robot-hosted
  // build has no simulation view, and the build-time constant lets the bundler drop the import there.
  const view =
    import.meta.env.VITE_USE_HOST_NAME === 'true' ? undefined : import('./SimulationView.svelte')
</script>

{#if view}
  {#await view}
    <div class="flex h-full w-full items-center justify-center">
      <span class="loading loading-spinner loading-lg text-primary"></span>
    </div>
  {:then { default: SimulationView }}
    <SimulationView />
  {:catch error}
    <div role="alert" class="alert alert-error m-4">
      The simulation could not load: {error.message}
    </div>
  {/await}
{/if}
