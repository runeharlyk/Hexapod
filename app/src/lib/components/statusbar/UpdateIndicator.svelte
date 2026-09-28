<script lang="ts">
  import { page } from '$app/state'
  import { modals } from 'svelte-modals'
  import { notifications } from '$lib/components/toasts/notifications'
  import ConfirmDialog from '$lib/components/ConfirmDialog.svelte'
  import GithubUpdateDialog from '$lib/components/GithubUpdateDialog.svelte'
  import { compareVersions } from 'compare-versions'
  import { onDestroy, onMount } from 'svelte'
  import { useFeatureFlags } from '$lib/stores/featureFlags'
  import {
    fetchLatestRelease,
    findFirmwareAsset,
    requestFirmwareDownload
  } from '$lib/services/firmware-update'
  import { Cancel, CloudDown, Firmware } from '../icons'

  const features = useFeatureFlags()
  const CHECK_INTERVAL_MS = 60 * 60 * 1000

  interface Props {
    update?: boolean
  }

  let { update = $bindable(false) }: Props = $props()

  let firmwareVersion: string = $state('')
  let firmwareDownloadLink: string = $state('')
  let checkIntervalId: ReturnType<typeof setInterval> | undefined

  async function checkForUpdate() {
    const result = await fetchLatestRelease(page.data.github)
    if (result.isErr()) {
      console.warn('Could not fetch the latest release:', result.inner)
      return
    }

    const release = result.inner
    update = false
    firmwareVersion = ''

    if (!$features.firmware_version) return
    if (compareVersions(release.tag_name, $features.firmware_version) !== 1) return
    const asset = findFirmwareAsset(release.assets, $features.firmware_built_target)
    if (!asset) return

    update = true
    firmwareVersion = release.tag_name
    firmwareDownloadLink = asset.browser_download_url
    notifications.info('Firmware update available.', 5000)
  }

  onMount(async () => {
    if (!$features.download_firmware) return
    await checkForUpdate()
    checkIntervalId = setInterval(checkForUpdate, CHECK_INTERVAL_MS)
  })

  onDestroy(() => clearInterval(checkIntervalId))

  function confirmGithubUpdate(url: string) {
    modals.open(ConfirmDialog, {
      title: 'Confirm flashing new firmware to the device',
      message: 'Are you sure you want to overwrite the existing firmware with a new one?',
      labels: {
        cancel: { label: 'Abort', icon: Cancel },
        confirm: { label: 'Update', icon: CloudDown }
      },
      onConfirm: () => {
        requestFirmwareDownload(url)
        modals.open(GithubUpdateDialog, {
          onConfirm: () => modals.closeAll()
        })
      }
    })
  }
</script>

{#if update}
  <div class="indicator flex-none">
    <button
      class="btn btn-square btn-ghost h-9 w-9"
      onclick={() => confirmGithubUpdate(firmwareDownloadLink)}
    >
      <span
        class="indicator-item indicator-top indicator-center badge badge-info badge-xs top-2 scale-75 lg:top-1"
      >
        {firmwareVersion}
      </span>
      <Firmware class="h-7 w-7" />
    </button>
  </div>
{/if}
