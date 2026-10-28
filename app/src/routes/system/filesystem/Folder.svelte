<script lang="ts">
  import Folder from './Folder.svelte'
  import File from './File.svelte'
  import type { DirectoryTree } from './tree.svelte'
  import { childPath } from '$lib/files'
  import { FolderIcon, FolderOpenOutline } from '$lib/components/icons'
  import LoadFailure from '$lib/components/layout/LoadFailure.svelte'
  import Loading from '$lib/components/layout/Loading.svelte'

  interface Props {
    expanded?: boolean
    name: string
    path: string
    tree: DirectoryTree
    selected: (path: string) => void
    onDelete: (path: string) => void
  }

  let { expanded = $bindable(false), name, path, tree, selected, onDelete }: Props = $props()

  const listing = $derived(tree.listings[path])

  function toggle() {
    expanded = !expanded
    if (expanded) tree.load(path)
  }
</script>

<div class="folder-item">
  <button class="hover:bg-base-200 flex w-full items-center rounded py-1 pl-2" onclick={toggle}>
    {#if expanded}
      <FolderOpenOutline class="w-5 h-5 mr-1" />
    {:else}
      <FolderIcon class="w-5 h-5 mr-1" />
    {/if}
    <span class="text-sm">{name}</span>
  </button>

  {#if expanded}
    <ul class="border-base-300 mt-1 ml-4 border-l">
      {#if listing?.error}
        <li class="py-1 pl-2">
          <LoadFailure
            message="Could not list {path}: {listing.error}"
            retry={() => tree.load(path)}
          />
        </li>
      {/if}
      {#each listing?.entries ?? [] as entry (entry.name)}
        <li class="py-1">
          {#if entry.isDirectory}
            <Folder
              name={entry.name}
              path={childPath(path, entry.name)}
              {tree}
              {selected}
              {onDelete}
            />
          {:else}
            <File name={entry.name} path={childPath(path, entry.name)} {selected} {onDelete} />
          {/if}
        </li>
      {:else}
        {#if !listing || listing.loading}
          <li class="py-1 pl-2"><Loading label="Reading {path}" rows={1} /></li>
        {:else if !listing.error}
          <li class="text-base-content/70 py-1 pl-2 text-sm">Empty</li>
        {/if}
      {/each}
    </ul>
  {/if}
</div>
