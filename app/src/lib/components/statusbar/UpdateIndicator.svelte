<script lang="ts">
  import { page } from '$app/state'
  import { resolve } from '$app/paths'
  import { useFeatureFlags } from '$lib/stores/featureFlags'
  import { isLinked } from '$lib/stores/link'
  import { latestReleaseOncePerSession, newerRelease } from '$lib/services/firmware-update'
  import type { GithubRelease } from '$lib/types/models'
  import { Firmware } from '../icons'

  const features = useFeatureFlags()

  let latest: GithubRelease | undefined = $state()

  // Only a connected robot has a running version worth comparing; the lookup itself runs once per session.
  $effect(() => {
    if (!$isLinked || latest) return
    void latestReleaseOncePerSession(page.data.github).then(release => (latest = release))
  })

  const available = $derived(
    $isLinked && latest ?
      newerRelease(latest, $features.firmware_version, $features.firmware_built_target)
    : undefined
  )
</script>

{#if available}
  <a
    class="indicator btn btn-square btn-ghost h-9 w-9 flex-none"
    href={resolve('/system/update')}
    title="Firmware {available.tag_name} is available"
    aria-label="Firmware {available.tag_name} is available"
  >
    <span
      class="indicator-item indicator-top indicator-center badge badge-info badge-xs top-2 scale-75 lg:top-1"
    >
      {available.tag_name}
    </span>
    <Firmware class="h-7 w-7" />
  </a>
{/if}
