<script lang="ts">
  import Controls from './Controls.svelte'
  import WidgetContainer from '$lib/components/layout/WidgetContainer.svelte'
  import { selectedView, views } from '$lib/stores/application'

  let layout = $derived($views.find(v => v.name === $selectedView) ?? $views[0])
</script>

<div class="absolute top-0 select-none w-screen h-screen">
  <Controls />
  <div class="absolute w-full h-screen top-0 overflow-hidden">
    <WidgetContainer container={layout.content} driveJoints />
  </div>
</div>
