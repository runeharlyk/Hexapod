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

<div class="flex items-center pl-4 group hover:bg-gray-700 rounded py-1">
  <button class="flex items-center gap-2 flex-grow" onclick={() => selected(path)}>
    <FileIcon class="w-4 h-4" />
    <span class="text-sm">{name}</span>
  </button>

  <button
    class="opacity-0 group-hover:opacity-100 p-1 hover:text-red-500"
    onclick={() => onDelete(path)}
  >
    <TrashIcon class="w-4 h-4" />
  </button>
</div>
