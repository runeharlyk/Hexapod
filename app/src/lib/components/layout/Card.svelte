<script lang="ts">
  import type { Snippet } from 'svelte'
  import type { IconComponent } from '$lib/components/icons'

  interface Props {
    title: string
    description?: string
    icon?: IconComponent
    /** Beside the title: what acts on the whole card, such as a scan. */
    actions?: Snippet
    /** Below a rule at the bottom, right-aligned: what commits the card, such as save. */
    footer?: Snippet
    children: Snippet
  }

  const { title, description, icon: Icon, actions, footer, children }: Props = $props()

  const titleId = $props.id()
</script>

<section class="card border-base-300 bg-base-200 border" aria-labelledby={titleId}>
  <div class="card-body gap-4 p-4 sm:p-6">
    <header class="flex items-start justify-between gap-4">
      <div class="min-w-0 flex-1 space-y-1">
        <h2 id={titleId} class="card-title text-base sm:text-lg">
          {#if Icon}<Icon class="size-5 shrink-0 opacity-70" />{/if}
          {title}
        </h2>
        {#if description}
          <p class="text-base-content/70 text-sm">{description}</p>
        {/if}
      </div>
      {#if actions}
        <div class="flex shrink-0 flex-wrap gap-2">{@render actions()}</div>
      {/if}
    </header>
    {@render children()}
    {#if footer}
      <div class="card-actions border-base-300 justify-end border-t pt-4">{@render footer()}</div>
    {/if}
  </div>
</section>
