<script lang="ts">
  import type { Snippet } from 'svelte'

  interface Props {
    label: string
    /** The id of the control the children render, which the label names. */
    for: string
    /**
     * What the control accepts. Shown below it, and in the error colour while the control, given the `validator` class,
     * fails its native constraints after the user has edited it or tried to save.
     */
    hint?: string
    children: Snippet
  }

  const { label, for: control, hint, children }: Props = $props()
</script>

<div class="grid gap-x-6 gap-y-1.5 sm:grid-cols-[11rem_minmax(0,28rem)]">
  <label class="text-sm font-medium sm:pt-2.5" for={control}>{label}</label>
  <!-- A toggle or a range, lower than an input, is moved down onto the label's line. -->
  <div class="min-w-0 sm:[&>.range]:mt-3 sm:[&>.toggle]:mt-2">
    {@render children()}
    {#if hint}
      <p class="validator-hint visible mt-1 text-xs opacity-70">{hint}</p>
    {/if}
  </div>
</div>
