<script lang="ts">
  import type { Snippet } from 'svelte'
  import type { IconComponent } from '$lib/components/icons'

  // A part of a settings page, with no box: the page's spacing and the rules PageShell draws between sections group it.
  interface Props {
    title: string
    description?: string
    icon?: IconComponent
    /** Beside the title: what acts on the whole section, such as a scan. */
    actions?: Snippet
    /** Below the content, right-aligned: what commits the section, such as save. */
    footer?: Snippet
    children: Snippet
  }

  const { title, description, icon: Icon, actions, footer, children }: Props = $props()

  const titleId = $props.id()
</script>

<section class="flex min-w-0 flex-col gap-4" aria-labelledby={titleId}>
  <header class="flex flex-wrap items-start justify-between gap-x-4 gap-y-2">
    <div class="min-w-48 flex-1 space-y-1">
      <h2 id={titleId} class="flex items-center gap-2 text-base font-semibold sm:text-lg">
        {#if Icon}<Icon class="size-5 shrink-0 opacity-70" />{/if}
        {title}
      </h2>
      {#if description}
        <p class="text-base-content/70 text-sm">{description}</p>
      {/if}
    </div>
    {#if actions}
      <div class="flex flex-wrap gap-2">{@render actions()}</div>
    {/if}
  </header>
  {@render children()}
  {#if footer}
    <div class="flex flex-wrap justify-end gap-2">{@render footer()}</div>
  {/if}
</section>

<style>
  /* Without a box around it, a list or table starts at the section's own edge, where its heading and fields do. */
  section :global(.list-row) {
    padding-inline: 0;
  }
  section :global(.list-row::after) {
    inset-inline: 0;
  }
  section :global(.table :is(th, td):first-child) {
    padding-inline-start: 0;
  }
  section :global(.table :is(th, td):last-child) {
    padding-inline-end: 0;
  }
</style>
