<script lang="ts">
  import Controls from './Controls.svelte'
  import WidgetContainer from '$lib/components/layout/WidgetContainer.svelte'
  import {
    isWidgetConfig,
    selectedView,
    views,
    type WidgetContainerConfig
  } from '$lib/stores/application'
  import { driveSimulation } from '$lib/stores/drive'

  const hasSimulation = (container: WidgetContainerConfig): boolean =>
    container.widgets.some(widget =>
      isWidgetConfig(widget) ? widget.component === 'Simulation' : hasSimulation(widget)
    )

  let layout = $derived($views.find(v => v.name === $selectedView) ?? $views[0])
  let simulated = $derived(
    import.meta.env.VITE_USE_HOST_NAME !== 'true' && hasSimulation(layout.content)
  )

  $effect(() => {
    driveSimulation(simulated)
    return () => driveSimulation(false)
  })
</script>

<div class="absolute top-0 select-none w-full h-dvh">
  <Controls />
  <div class="absolute w-full h-full top-0 overflow-hidden">
    <WidgetContainer container={layout.content} driveJoints />
  </div>
</div>
