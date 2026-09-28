<script lang="ts">
  import Controls from './Controls.svelte'
  import WidgetContainer from '$lib/components/layout/WidgetContainer.svelte'
  import { selectedView, views } from '$lib/stores/application'
  import { onMount } from 'svelte'
  import { imu } from '$lib/stores/imu'
  import { dataBroker } from '$lib/transport/databroker'
  import { IMUData } from '$lib/platform_shared/message'

  let layout = $derived($views.find(v => v.name === $selectedView) ?? $views[0])

  onMount(() =>
    dataBroker.on(IMUData, data => {
      imu.addData({ ...data, altitude: 0, bmp_temp: 0, pressure: 0 })
    })
  )
</script>

<div class="absolute top-0 select-none w-screen h-screen">
  <Controls />
  <div class="absolute w-full h-screen top-0 overflow-hidden">
    <WidgetContainer container={layout.content} />
  </div>
</div>
