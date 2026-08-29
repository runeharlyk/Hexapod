<script lang="ts">
  import { onMount } from 'svelte'
  import UploadFirmware from './UploadFirmware.svelte'
  import GithubFirmwareManager from './GithubFirmwareManager.svelte'
  import { useFeatureFlags } from '$lib/stores'
  import { telemetry } from '$lib/stores/telemetry'
  import { dataBroker } from '$lib/transport/databroker'
  import { OtaState, OtaStatusData } from '$lib/platform_shared/message'

  const features = useFeatureFlags()

  const OTA_STATE_NAMES: Record<number, string> = {
    [OtaState.OTA_IDLE]: 'idle',
    [OtaState.OTA_PROGRESS]: 'progress',
    [OtaState.OTA_FINISHED]: 'finished',
    [OtaState.OTA_ERROR]: 'error'
  }

  onMount(() =>
    dataBroker.on(OtaStatusData, data =>
      telemetry.setDownloadOTA({
        status: OTA_STATE_NAMES[data.state] ?? 'idle',
        progress: data.progress,
        error: data.error
      })
    )
  )
</script>

<div class="mx-0 my-1 flex flex-col space-y-4 sm:mx-8 sm:my-8">
  {#if $features.download_firmware}
    <GithubFirmwareManager />
  {/if}

  {#if $features.upload_firmware}
    <UploadFirmware />
  {/if}
</div>
