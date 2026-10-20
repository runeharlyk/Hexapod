<script lang="ts">
  import type { Snippet } from 'svelte'

  // The statusbar shows the route's title as the page's h1, so the shell repeats it nowhere.
  interface Props {
    /** form: one readable column; wide: tables and charts that use the screen. */
    width?: 'form' | 'wide'
    /**
     * sections: a settings page, its parts (Section) divided by rules; cards: a dashboard, each part (Card) a box that
     * stands alone.
     */
    grouping?: 'sections' | 'cards'
    children: Snippet
  }

  const { width = 'form', grouping = 'sections', children }: Props = $props()

  const widths = { form: 'max-w-3xl', wide: 'max-w-7xl' }
  // Cards on a wide page sit two abreast from 1024px; a column never grows past the screen for a wide table in it.
  const layout = $derived(
    grouping === 'cards' ?
      `grid grid-cols-[minmax(0,1fr)] items-start gap-4 sm:gap-6 ${width === 'wide' ? 'lg:grid-cols-2' : ''}`
    : 'flex flex-col gap-6 *:not-first:border-base-300 *:not-first:border-t *:not-first:pt-6'
  )
</script>

<main class="mx-auto w-full min-w-0 p-4 sm:p-6 lg:px-8 {layout} {widths[width]}">
  {@render children()}
</main>
