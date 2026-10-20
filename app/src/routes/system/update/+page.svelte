<script lang="ts">
  import { onMount } from 'svelte'
  import PageShell from '$lib/components/layout/PageShell.svelte'
  import UploadFirmware from './UploadFirmware.svelte'
  import GithubFirmwareManager from './GithubFirmwareManager.svelte'
  import { useFeatureFlags } from '$lib/stores'
  import { telemetry } from '$lib/stores/telemetry'
  import { OtaState } from '$lib/platform_shared/message'
  import { otaStatus } from '$lib/robot/state'

  const features = useFeatureFlags()

  const OTA_STATE_NAMES: Record<number, string> = {
    [OtaState.OTA_IDLE]: 'idle',
    [OtaState.OTA_PROGRESS]: 'progress',
    [OtaState.OTA_FINISHED]: 'finished',
    [OtaState.OTA_ERROR]: 'error'
  }

  // The robot answers the subscribe with the update's current state and pushes every change; an earlier update's end
  // is not shown (robot/state.ts).
  onMount(() =>
    otaStatus.subscribe(value => {
      if (value)
        telemetry.setDownloadOTA({
          status: OTA_STATE_NAMES[value.state] ?? 'idle',
          progress: value.progress,
          error: value.error
        })
    })
  )
</script>

<PageShell>
  {#if $features.download_firmware}
    <GithubFirmwareManager />
  {/if}

  {#if $features.upload_firmware}
    <UploadFirmware />
  {/if}
</PageShell>
