<script lang="ts">
  import { FileIcon, TrashIcon } from '$lib/components/icons'

  interface Props {
    name: string
    path: string
    selected: (path: string) => void
    onDelete: (path: string) => void
  }

  let { name, path, selected, onDelete }: Props = $props()
</script>

<div class="group hover:bg-base-200 flex items-center rounded py-1 pl-4">
  <button class="flex items-center gap-2 flex-grow" onclick={() => selected(path)}>
    <FileIcon class="w-4 h-4" />
    <span class="text-sm">{name}</span>
  </button>

  <button
    class="hover:text-error p-1 opacity-0 group-hover:opacity-100 focus:opacity-100"
    aria-label="Delete {name}"
    onclick={() => onDelete(path)}
  >
    <TrashIcon class="w-4 h-4" />
  </button>
</div>
