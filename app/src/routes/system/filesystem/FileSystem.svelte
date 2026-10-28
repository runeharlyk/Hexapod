<script lang="ts">
  import Folder from './Folder.svelte'
  import { DirectoryTree } from './tree.svelte'
  import {
    CHUNK,
    childPath,
    deleteFile as removeFile,
    fileExists,
    makeDirectory,
    readFile,
    writeFile
  } from '$lib/files'
  import ConfirmDialog from '$lib/components/ConfirmDialog.svelte'
  import { FolderIcon, Add, FileIcon, Cancel, Check } from '$lib/components/icons'
  import LoadFailure from '$lib/components/layout/LoadFailure.svelte'
  import Loading from '$lib/components/layout/Loading.svelte'
  import Section from '$lib/components/layout/Section.svelte'
  import { guardUnsaved } from '$lib/components/layout/unsaved-guard'
  import { notifications } from '$lib/components/toasts/notifications'
  import { modals } from 'svelte-modals'
  import NewFolderDialog from './NewFolderDialog.svelte'
  import NewFileDialog from './NewFileDialog.svelte'

  // A transfer of this many bytes or fewer is over before a progress bar would be read.
  const PROGRESS_FROM = 4 * CHUNK

  const tree = new DirectoryTree()
  tree.load('/')

  let filename = $state('')
  // The file's text as the robot holds it, once read; the editor's text is `content`.
  let original = $state<string>()
  // The size of a file read whole that is not UTF-8 text, which the page neither shows nor edits.
  let binarySize = $state<number>()
  let content = $state('')
  let isEditing = $state(false)
  let saving = $state(false)
  let transfer = $state<{ done: number; total: number }>()
  let reading = $state<Promise<void>>()

  const dirty = $derived(isEditing && content !== original)
  guardUnsaved(() => dirty)

  const parentOf = (path: string) => path.slice(0, path.lastIndexOf('/')) || '/'
  const reasonOf = (error: unknown) => (error instanceof Error ? error.message : String(error))
  // A transfer shows its progress, and clears it at its end, only while its file is the one open.
  const track = (path: string) => (done: number, total: number) => {
    if (filename === path) transfer = { done, total }
  }

  // Fatal, so a file that is not UTF-8 is never edited into replacement characters; a byte order mark stays in the
  // text, so a save writes it back.
  const textOf = (bytes: Uint8Array) => {
    try {
      return new TextDecoder('utf-8', { fatal: true, ignoreBOM: true }).decode(bytes)
    } catch {
      return undefined
    }
  }

  const getContent = async (path: string) => {
    try {
      const bytes = await readFile(path, track(path))
      if (filename !== path) return
      const text = textOf(bytes)
      if (text === undefined) {
        binarySize = bytes.length
        return
      }
      original = text
      content = text
    } finally {
      if (filename === path) transfer = undefined
    }
  }

  const saveContent = async () => {
    const path = filename
    const text = content
    saving = true
    try {
      await writeFile(path, new TextEncoder().encode(text), track(path))
      if (filename === path) {
        original = text
        isEditing = false
      }
    } catch (error) {
      notifications.error(`Could not save ${path}: ${reasonOf(error)}`, 5000)
    } finally {
      saving = false
      if (filename === path) transfer = undefined
      // A write that failed halfway leaves its .part file, which the listing shows.
      tree.refresh(parentOf(path))
    }
  }

  const askToDiscard = (discard: () => void) =>
    modals.open(ConfirmDialog, {
      title: 'Discard changes',
      message: `${filename} has edits that are not saved. Discard them?`,
      labels: {
        cancel: { label: 'Keep', icon: Cancel },
        confirm: { label: 'Discard', icon: Check }
      },
      onConfirm: () => {
        modals.close()
        discard()
      }
    })

  const cancelEditing = () => {
    const stop = () => {
      content = original ?? ''
      isEditing = false
    }
    if (dirty) askToDiscard(stop)
    else stop()
  }

  const deleteFile = async (path: string) => {
    if (!confirm(`Are you sure you want to delete ${path}?`)) return
    try {
      await removeFile(path)
      if (path === filename) {
        filename = ''
        original = undefined
        binarySize = undefined
        content = ''
        isEditing = false
      }
    } catch (error) {
      notifications.error(`Could not delete ${path}: ${reasonOf(error)}`, 5000)
    }
    tree.refresh(parentOf(path))
  }

  const createFolder = async (folderName: string) => {
    if (!folderName) return
    try {
      await makeDirectory('/' + folderName)
    } catch (error) {
      notifications.error(`Could not create ${folderName}: ${reasonOf(error)}`, 5000)
    }
    tree.refresh('/')
  }

  const openFile = (path: string) => {
    filename = path
    original = undefined
    binarySize = undefined
    content = ''
    isEditing = false
    reading = getContent(path)
  }

  const selectFile = (path: string) => {
    if (dirty) askToDiscard(() => openFile(path))
    else openFile(path)
  }

  const openNewFolderDialog = () => {
    modals.open(NewFolderDialog, {
      onConfirm: createFolder
    })
  }

  const createFile = async (fileName: string) => {
    if (!fileName) return
    const path = childPath('/', fileName)
    try {
      if (await fileExists(path)) return notifications.error(`${fileName} already exists`, 5000)
      await writeFile(path, new Uint8Array())
    } catch (error) {
      notifications.error(`Could not create ${fileName}: ${reasonOf(error)}`, 5000)
      return
    }
    tree.refresh(parentOf(path))
    selectFile(path)
  }

  const openNewFileDialog = () => {
    modals.open(NewFileDialog, {
      onConfirm: createFile
    })
  }
</script>

{#snippet progress()}
  {#if transfer && transfer.total > PROGRESS_FROM}
    <progress
      class="progress progress-primary mb-2 w-full"
      value={transfer.done}
      max={transfer.total}
    ></progress>
  {/if}
{/snippet}

<Section title="Files" description="The robot's LittleFS partition." icon={FolderIcon}>
  {#snippet actions()}
    <button class="btn btn-sm" onclick={openNewFileDialog}>
      <FileIcon class="size-4" />New file
    </button>
    <button class="btn btn-sm" onclick={openNewFolderDialog}>
      <Add class="size-4" />New folder
    </button>
  {/snippet}

  <div class="flex w-full flex-col gap-4 md:flex-row">
    <div
      class="border-base-300 w-full border-b pb-4 md:w-72 md:shrink-0 md:border-r md:border-b-0 md:pr-4 md:pb-0"
    >
      <Folder name="/" path="/" {tree} expanded selected={selectFile} onDelete={deleteFile} />
    </div>

    <div class="min-w-0 flex-1">
      {#if filename && reading}
        <div
          class="mb-4 flex flex-col items-start justify-between gap-2 sm:flex-row sm:items-center"
        >
          <h3 class="min-w-0 truncate font-semibold">{filename}</h3>
          <div class="flex gap-2">
            {#if isEditing}
              <button
                class="btn btn-sm btn-primary"
                disabled={saving || !dirty}
                onclick={saveContent}
              >
                Save
              </button>
              <button class="btn btn-sm btn-ghost" disabled={saving} onclick={cancelEditing}>
                Cancel
              </button>
            {:else}
              {#if original !== undefined}
                <button class="btn btn-sm" onclick={() => (isEditing = true)}>Edit</button>
              {/if}
              <button class="btn btn-sm btn-error btn-soft" onclick={() => deleteFile(filename)}>
                Delete
              </button>
            {/if}
          </div>
        </div>

        {#await reading}
          {@render progress()}
          <Loading label="Reading {filename}" rows={1} />
        {:then}
          {#if isEditing}
            {@render progress()}
            <textarea
              class="textarea h-72 w-full font-mono sm:h-[32rem]"
              disabled={saving}
              bind:value={content}
            ></textarea>
          {:else if binarySize !== undefined}
            <p class="text-base-content/70 text-sm">
              {binarySize} bytes that are not UTF-8 text, which this page cannot show or edit.
            </p>
          {:else}
            <pre
              class="bg-base-200 rounded-box max-h-72 overflow-auto p-4 text-sm sm:max-h-[32rem]">{content}</pre>
          {/if}
        {:catch error}
          <LoadFailure
            message="Could not read {filename}: {error.message}"
            retry={() => openFile(filename)}
          />
        {/await}
      {:else}
        <p class="text-base-content/70 text-sm">Select a file to view its contents.</p>
      {/if}
    </div>
  </div>
</Section>
