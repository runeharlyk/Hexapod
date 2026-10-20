<script lang="ts">
  import Folder from './Folder.svelte'
  import { api } from '$lib/api'
  import { resolveUrl } from '$lib/proto-api'
  import type { Directory } from '$lib/types/models'
  import { FolderIcon, Add, FileIcon } from '$lib/components/icons'
  import LoadFailure from '$lib/components/layout/LoadFailure.svelte'
  import Loading from '$lib/components/layout/Loading.svelte'
  import Section from '$lib/components/layout/Section.svelte'
  import { modals } from 'svelte-modals'
  import NewFolderDialog from './NewFolderDialog.svelte'
  import NewFileDialog from './NewFileDialog.svelte'

  let filename = $state('')
  let content = $state('')
  let isEditing = $state(false)

  const getFiles = async () => {
    const result = await api.get<Directory>('/api/files')
    if (result.isErr()) throw new Error(String(result.exception))
    if (!result.inner) throw new Error('the reply carried no files')
    return result.inner
  }

  // The tree stays as it is while a refresh reads it again, so a new file or a delete does not fold it up.
  let directory = $state<Directory>()
  let readError = $state('')
  const refreshFiles = async () => {
    try {
      directory = await getFiles()
      readError = ''
    } catch (error) {
      readError = error instanceof Error ? error.message : String(error)
    }
  }
  refreshFiles()

  const getContent = async (path: string) => {
    const res = await fetch(resolveUrl('/api/files/content?path=' + path))
    if (!res.ok) throw new Error(`Request failed with status ${res.status}`)
    content = await res.text()
    return content
  }

  let reading = $state<Promise<string>>()

  const saveContent = async () => {
    if (!filename) return
    const result = await api.post('/api/files/edit', { path: filename, content })
    if (result.isOk()) isEditing = false
  }

  const deleteFile = async (path: string) => {
    if (!confirm(`Are you sure you want to delete ${path}?`)) return
    const result = await api.post('/api/files/delete', { path })
    if (result.isOk()) {
      filename = ''
      content = ''
      refreshFiles()
    }
  }

  const createFolder = async (folderName: string) => {
    if (!folderName) return
    const result = await api.post('/api/files/mkdir', { path: '/' + folderName })
    if (result.isOk()) refreshFiles()
  }

  const updateSelected = (path: string) => {
    filename = path
    isEditing = false
    reading = getContent(path)
  }

  const openNewFolderDialog = () => {
    modals.open(NewFolderDialog, {
      onConfirm: createFolder
    })
  }

  const createFile = async (fileName: string) => {
    if (!fileName) return
    const path = '/' + fileName
    const result = await api.post('/api/files/edit', { path, content: '' })
    if (result.isOk()) {
      refreshFiles()
      updateSelected(path)
    }
  }

  const openNewFileDialog = () => {
    modals.open(NewFileDialog, {
      onConfirm: createFile
    })
  }
</script>

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
      {#if readError}
        <LoadFailure message="Could not read the files: {readError}" retry={refreshFiles} />
      {/if}
      {#if directory}
        <Folder
          name="/"
          path=""
          files={directory.root as Directory}
          expanded
          selected={updateSelected}
          onDelete={deleteFile}
        />
      {:else if !readError}
        <Loading label="Reading the files" />
      {/if}
    </div>

    <div class="min-w-0 flex-1">
      {#if filename && reading}
        <div
          class="mb-4 flex flex-col items-start justify-between gap-2 sm:flex-row sm:items-center"
        >
          <h3 class="min-w-0 truncate font-semibold">{filename}</h3>
          <div class="flex gap-2">
            {#if isEditing}
              <button class="btn btn-sm btn-primary" onclick={saveContent}>Save</button>
              <button class="btn btn-sm btn-ghost" onclick={() => (isEditing = false)}>
                Cancel
              </button>
            {:else}
              <button class="btn btn-sm" onclick={() => (isEditing = true)}>Edit</button>
              <button class="btn btn-sm btn-error btn-soft" onclick={() => deleteFile(filename)}>
                Delete
              </button>
            {/if}
          </div>
        </div>

        {#await reading}
          <Loading label="Reading {filename}" rows={1} />
        {:then}
          {#if isEditing}
            <textarea class="textarea h-72 w-full font-mono sm:h-[32rem]" bind:value={content}
            ></textarea>
          {:else}
            <pre
              class="bg-base-200 rounded-box max-h-72 overflow-auto p-4 text-sm sm:max-h-[32rem]">{content}</pre>
          {/if}
        {:catch error}
          <LoadFailure
            message="Could not read {filename}: {error.message}"
            retry={() => updateSelected(filename)}
          />
        {/await}
      {:else}
        <p class="text-base-content/70 text-sm">Select a file to view its contents.</p>
      {/if}
    </div>
  </div>
</Section>
