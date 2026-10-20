<script lang="ts">
  // Placeholder rows while something is read, named for assistive technology by what is read.
  interface Props {
    label: string
    rows?: number
  }

  const { label, rows = 3 }: Props = $props()

  const placeholders = $derived(Array.from({ length: rows }, (_, row) => row))
</script>

<div class="grid gap-2" role="status" aria-label={label}>
  {#each placeholders as row (row)}
    <div class="skeleton h-12 w-full"></div>
  {/each}
</div>
