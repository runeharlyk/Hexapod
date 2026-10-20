<script lang="ts">
  import { modals } from 'svelte-modals'
  import ConfirmDialog from '$lib/components/ConfirmDialog.svelte'
  import Card from '$lib/components/layout/Card.svelte'
  import LoadFailure from '$lib/components/layout/LoadFailure.svelte'
  import Loading from '$lib/components/layout/Loading.svelte'
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

  const read = () => {
    loadError = ''
    readCoreDump(dataBroker).then(
      read => (report = read),
      error => (loadError = (error as Error).message)
    )
  }
  read()

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

{#snippet dumpActions()}
  <button class="btn btn-primary" disabled={busy} onclick={download}>
    <DownloadIcon class="size-5" />Download
  </button>
  <button class="btn" disabled={busy} onclick={confirmErase}>
    <Delete class="size-5" />Erase
  </button>
{/snippet}

<Card title="Last crash" icon={Warning} footer={report?.stored ? dumpActions : undefined}>
  {#if loadError}
    <LoadFailure message="The core dump could not be read: {loadError}" retry={read} />
  {:else if !crash}
    <Loading label="Reading the core dump" rows={1} />
  {:else if crash.kind === 'none'}
    <p class="text-sm">No crash stored.</p>
  {:else if crash.kind === 'unreadable'}
    <div class="alert alert-warning alert-soft" role="alert">
      A dump of {crash.size} bytes is stored, but the robot could not read a summary from it: it fails
      its checksum, as a dump cut short does, or is not an ELF core dump.
    </div>
  {:else}
    <dl class="grid grid-cols-[max-content_1fr] gap-x-4 gap-y-1 text-sm">
      <dt class="font-semibold">Reason</dt>
      {#if crash.panicReason}
        <dd class="whitespace-pre-line">{crash.panicReason}</dd>
      {:else}
        <dd class="text-base-content/70">
          Not in the dump, which keeps only a task watchdog's; an exception's, assert's or abort's
          is on the serial log.
        </dd>
      {/if}
      <dt class="font-semibold">Task</dt>
      <dd>{crash.task}</dd>
      <dt class="font-semibold">PC</dt>
      <dd class="font-mono">{crash.pc}</dd>
      {#if crash.backtrace}
        <dt class="font-semibold">Backtrace</dt>
        <dd class="font-mono break-all">{crash.backtrace}</dd>
      {/if}
      {#if crash.elf}
        <dt class="font-semibold">Firmware ELF</dt>
        <dd class="font-mono break-all">{crash.elf}</dd>
      {/if}
    </dl>
    {#if crash.build === 'other'}
      <div class="alert alert-warning alert-soft" role="alert">
        The crash came from another build than the one running now: decode it with the ELF of the
        build whose SHA-256 begins as above.
      </div>
    {:else if crash.build === 'unnamed'}
      <div class="alert alert-warning alert-soft" role="alert">
        The dump does not name the build that crashed: decode it with the ELF of the build that was
        running then.
      </div>
    {/if}
  {/if}

  {#if crash && crash.kind !== 'none'}
    <p class="text-base-content/70 text-sm">
      Decode the downloaded dump with <code>esp-coredump info_corefile -t raw -c</code> and the
      build's ELF, which each release attaches as <code>esp32-wroom-camera.elf</code>.
    </p>
  {/if}

  {#if downloaded !== null}
    <progress class="progress progress-primary w-full" value={downloaded} max="1"></progress>
  {/if}
  {#if actionError}
    <div class="alert alert-error alert-soft" role="alert">{actionError}</div>
  {/if}
</Card>
