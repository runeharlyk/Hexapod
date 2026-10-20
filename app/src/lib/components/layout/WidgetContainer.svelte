<script lang="ts">
  import WidgetContainer from './WidgetContainer.svelte'
  import { type WidgetContainerConfig, isWidgetConfig } from '$lib/stores/application'
  import Widget from './Widget.svelte'
  import Visualization from '$lib/components/Visualization.svelte'
  import Stream from '$lib/components/Stream.svelte'
  import StreamWithRobot from '$lib/components/StreamWithRobot.svelte'
  import LazySimulationView from '$lib/components/LazySimulationView.svelte'

  interface Props {
    container: WidgetContainerConfig
    // Its 3D views drive the robot's joints while the robot idles (Visualization's driveJoints).
    driveJoints?: boolean
  }

  let { container, driveJoints = false }: Props = $props()
</script>

<div class="w-full h-full flex flex-col overflow-hidden">
  <div
    class="flex w-full h-full"
    class:flex-row={container.layout === 'column'}
    class:flex-col={container.layout === 'row'}
    class:flex-wrap={container.layout === 'wrap'}
  >
    {#each container.widgets as widget, index (widget.id + '-' + index)}
      <Widget>
        {#if isWidgetConfig(widget)}
          {#if widget.component === 'Visualization'}
            <Visualization {driveJoints} />
          {:else if widget.component === 'Simulation'}
            <LazySimulationView />
          {:else if widget.component === 'StreamWithRobot'}
            <StreamWithRobot {driveJoints} />
          {:else}
            <Stream />
          {/if}
        {:else if widget.widgets}
          <WidgetContainer container={widget} {driveJoints} />
        {/if}
      </Widget>
      {#if index !== container.widgets.length - 1}
        <div
          class="divider bg-base-300 m-0"
          class:divider-horizontal={container.layout === 'column'}
        ></div>
      {/if}
    {/each}
  </div>
</div>
