<script lang="ts">
  import { page } from '$app/state'
  import { modals } from 'svelte-modals'
  import ConfirmDialog from '$lib/components/ConfirmDialog.svelte'
  import LoadFailure from '$lib/components/layout/LoadFailure.svelte'
  import Loading from '$lib/components/layout/Loading.svelte'
  import Section from '$lib/components/layout/Section.svelte'

  import { compareVersions } from 'compare-versions'
  import GithubUpdateDialog from '$lib/components/GithubUpdateDialog.svelte'
  import InfoDialog from '$lib/components/InfoDialog.svelte'
  import {
    fetchReleases,
    findFirmwareAsset,
    requestFirmwareDownload
  } from '$lib/services/firmware-update'
  import type { GithubAsset } from '$lib/types/models'
  import { useFeatureFlags } from '$lib/stores'
  import { Cancel, Check, CloudDown, Github } from '$lib/components/icons'

  const features = useFeatureFlags()

  async function getGithubReleases() {
    const result = await fetchReleases(page.data.github)
    if (result.isErr()) throw result.inner
    return result.inner
  }

  let releases = $state(getGithubReleases())

  const released = (date: string) =>
    new Intl.DateTimeFormat('en-GB', { dateStyle: 'medium' }).format(new Date(date))

  // compareVersions throws on a non-semver string, which the version is until the flags load.
  const isInstalled = (tag: string) =>
    !!$features.firmware_version && compareVersions($features.firmware_version, tag) === 0

  function confirmGithubUpdate(assets: GithubAsset[]) {
    const url = findFirmwareAsset(assets, $features.firmware_built_target)?.browser_download_url
    if (!url) {
      modals.open(InfoDialog, {
        title: 'No matching firmware found',
        message:
          'No matching firmware was found for the current device. Upload the firmware manually or build from sources.',
        dismiss: { label: 'OK', icon: Check },
        onDismiss: () => modals.close()
      })
      return
    }

    modals.open(ConfirmDialog, {
      title: 'Confirm flashing new firmware to the device',
      message: 'Are you sure you want to overwrite the existing firmware with a new one?',
      labels: {
        cancel: { label: 'Abort', icon: Cancel },
        confirm: { label: 'Update', icon: CloudDown }
      },
      onConfirm: () => {
        requestFirmwareDownload(url, $features.firmware_elf_sha256)
        modals.open(GithubUpdateDialog, {
          onConfirm: () => modals.closeAll()
        })
      }
    })
  }
</script>

<Section title="Releases" description="Firmware published on GitHub." icon={Github}>
  {#await releases}
    <Loading label="Reading the releases" />
  {:then githubReleases}
    <ul class="list">
      {#each githubReleases as release (release.tag_name)}
        <li class="list-row items-center">
          <div class="list-col-grow min-w-0">
            <a
              href={release.html_url}
              class="link link-hover font-semibold"
              target="_blank"
              rel="noopener noreferrer">{release.name}</a
            >
            <div class="text-base-content/70 text-sm">{released(release.published_at)}</div>
          </div>
          <div class="flex items-center gap-2">
            {#if release.prerelease}
              <span class="badge badge-accent badge-soft badge-sm">Pre-release</span>
            {/if}
            {#if isInstalled(release.tag_name)}
              <span class="badge badge-primary badge-sm">Installed</span>
            {:else}
              <button
                class="btn btn-sm"
                aria-label="Install {release.name}"
                onclick={() => confirmGithubUpdate(release.assets)}
              >
                <CloudDown class="size-4" />Install
              </button>
            {/if}
          </div>
        </li>
      {/each}
    </ul>
  {:catch}
    <LoadFailure
      message="Please connect to a network with internet access to perform a firmware update."
      retry={() => (releases = getGithubReleases())}
    />
  {/await}
</Section>
