<script lang="ts">
  import Folder from './Folder.svelte'
  import File from './File.svelte'
  import { FolderIcon, FolderOpenOutline } from '$lib/components/icons'
  import type { Directory } from '$lib/types/models'

  interface Props {
    expanded?: boolean
    name: string
    path?: string
    files: Directory
    selected: (path: string) => void
    onDelete: (path: string) => void
  }

  let { expanded = $bindable(false), name, path = '', files, selected, onDelete }: Props = $props()

  function toggle() {
    expanded = !expanded
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
      {#each Object.entries(files) as [itemName, content] (itemName)}
        <li class="py-1">
          {#if typeof content === 'object'}
            <Folder
              name={itemName}
              path={`${path}/${itemName}`}
              files={content}
              {selected}
              {onDelete}
            />
          {:else}
            <File name={itemName} path={`${path}/${itemName}`} {selected} {onDelete} />
          {/if}
        </li>
      {/each}
    </ul>
  {/if}
</div>
