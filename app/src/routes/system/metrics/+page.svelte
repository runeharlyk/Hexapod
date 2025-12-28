<script lang="ts">
  import SystemMetrics from './SystemMetrics.svelte'
  import { goto } from '$app/navigation'
  import { useFeatureFlags } from '$lib/stores/featureFlags'
  import { resolve } from '$app/paths'

  const features = useFeatureFlags()

  if (!$features.analytics) {
    goto(resolve('/'))
  }
</script>

<div class="mx-0 my-1 flex flex-col space-y-4 sm:mx-8 sm:my-8">
  <SystemMetrics />
</div>
