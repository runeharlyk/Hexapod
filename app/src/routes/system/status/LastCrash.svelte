<script lang="ts">
  import { modals } from 'svelte-modals'
  import ConfirmDialog from '$lib/components/ConfirmDialog.svelte'
  import SettingsCard from '$lib/components/SettingsCard.svelte'
  import Spinner from '$lib/components/Spinner.svelte'
  import { Cancel, Delete, DownloadIcon, Warning } from '$lib/components/icons'
  import type { CoreDumpReport } from '$lib/platform_shared/message'
  import { dataBroker } from '$lib/transport/databroker'
  import {
    coreDumpFilename,
    downloadCoreDump,
    eraseCoreDump,
    lastCrash,
    readCoreDump
  } from '$lib/stores/core-dump'

  let report: CoreDumpReport | null = $state(null)
  let loadError = $state('')
  let actionError = $state('')
  let downloaded: number | null = $state(null)
  let erasing = $state(false)

  const crash = $derived(report ? lastCrash(report) : null)
  const busy = $derived(downloaded !== null || erasing)

  readCoreDump(dataBroker).then(
    read => (report = read),
    error => (loadError = (error as Error).message)
  )

  async function download() {
    if (!report) return
    actionError = ''
    downloaded = 0
    try {
      const bytes = await downloadCoreDump(dataBroker, (done, total) => (downloaded = done / total))
      const link = document.createElement('a')
      link.href = URL.createObjectURL(new Blob([bytes], { type: 'application/octet-stream' }))
      link.download = coreDumpFilename(report)
      link.click()
      setTimeout(() => URL.revokeObjectURL(link.href))
    } catch (error) {
      actionError = (error as Error).message
    } finally {
      downloaded = null
    }
  }

  function confirmErase() {
    modals.open(ConfirmDialog, {
      title: 'Erase the Core Dump',
      message:
        'The crash can no longer be decoded once its dump is erased. The robot must be deactivated or idle.',
      labels: {
        cancel: { label: 'Abort', icon: Cancel },
        confirm: { label: 'Erase', icon: Delete }
      },
      onConfirm: async () => {
        modals.close()
        actionError = ''
        erasing = true
        try {
          report = await eraseCoreDump(dataBroker)
        } catch (error) {
          actionError = (error as Error).message
        } finally {
          erasing = false
        }
      }
    })
  }
</script>

<SettingsCard collapsible={false}>
  {#snippet icon()}
    <Warning class="mr-2 h-6 w-6 shrink-0 self-end" />
  {/snippet}
  {#snippet title()}
    <span>Last Crash</span>
  {/snippet}

  {#if loadError}
    <div class="alert alert-error" role="alert">The core dump could not be read: {loadError}</div>
  {:else if !crash}
    <Spinner />
  {:else if crash.kind === 'none'}
    <p>No crash stored.</p>
  {:else if crash.kind === 'unreadable'}
    <div class="alert alert-warning" role="alert">
      A dump of {crash.size} bytes is stored, but it fails its checksum, so it has no summary.
    </div>
  {:else}
    <dl class="grid grid-cols-[max-content_1fr] gap-x-4 gap-y-1 text-sm">
      {#if crash.panicReason}
        <dt class="font-bold">Reason</dt>
        <dd class="whitespace-pre-line">{crash.panicReason}</dd>
      {/if}
      <dt class="font-bold">Task</dt>
      <dd>{crash.task}</dd>
      <dt class="font-bold">PC</dt>
      <dd class="font-mono">{crash.pc}</dd>
      {#if crash.backtrace}
        <dt class="font-bold">Backtrace</dt>
        <dd class="font-mono break-all">{crash.backtrace}</dd>
      {/if}
      <dt class="font-bold">Firmware ELF</dt>
      <dd class="font-mono break-all">{crash.elf}</dd>
    </dl>
    {#if !crash.runningBuild}
      <div class="alert alert-warning mt-2" role="alert">
        The crash came from another build than the one running now: decode it with the ELF of the
        build whose SHA-256 begins as above.
      </div>
    {/if}
  {/if}

  {#if crash && crash.kind !== 'none'}
    <p class="mt-2 text-sm opacity-75">
      Decode the downloaded dump with <code>esp-coredump info_corefile -t raw -c</code> and the
      build's ELF, which each release attaches as <code>esp32-wroom-camera.elf</code>.
    </p>
  {/if}

  {#if downloaded !== null}
    <progress class="progress progress-primary mt-2 w-full" value={downloaded} max="1"></progress>
  {/if}
  {#if actionError}
    <div class="alert alert-error mt-2" role="alert">{actionError}</div>
  {/if}

  {#if report?.stored}
    <div class="mt-4 flex flex-wrap justify-end gap-2">
      <button class="btn btn-primary inline-flex items-center" disabled={busy} onclick={download}>
        <DownloadIcon class="mr-2 h-5 w-5" />
        <span>Download</span>
      </button>
      <button
        class="btn btn-secondary inline-flex items-center"
        disabled={busy}
        onclick={confirmErase}
      >
        <Delete class="mr-2 h-5 w-5" />
        <span>Erase</span>
      </button>
    </div>
  {/if}
</SettingsCard>
