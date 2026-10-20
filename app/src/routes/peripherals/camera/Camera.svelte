<script lang="ts">
  import CameraSetting from './CameraSetting.svelte'
  import Stream from '$lib/components/Stream.svelte'
  import Section from '$lib/components/layout/Section.svelte'
  import { Camera } from '$lib/components/icons'
</script>

<Section title="Stream" icon={Camera}>
  <div class="rounded-box relative aspect-video overflow-hidden">
    <Stream />
  </div>
</Section>
<CameraSetting />
