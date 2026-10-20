<script lang="ts">
  import type { Snippet } from 'svelte'
  import { Down } from '$lib/components/icons'

  interface Props {
    title: string
    description?: string
    children: Snippet
  }

  const { title, description, children }: Props = $props()

  // A save checks every field, folded ones too, but the browser cannot point at one it does not show: it gives up on a
  // folded field in silence. So the disclosure opens on the field's `invalid` event, and in the next frame, once the
  // field is laid out, the field reports itself when it is the form's first invalid one, as the browser would have.
  const openOnInvalid = (event: Event & { currentTarget: HTMLDetailsElement }) => {
    const disclosure = event.currentTarget
    if (disclosure.open) return
    disclosure.open = true
    const field = event.target as HTMLInputElement
    if (field.form?.querySelector(':is(input, select, textarea):invalid') === field)
      requestAnimationFrame(() => field.reportValidity())
  }
</script>

<details class="group" oninvalidcapture={openOnInvalid}>
  <summary
    class="flex w-fit cursor-pointer list-none items-center gap-2 py-1 text-sm font-medium [&::-webkit-details-marker]:hidden"
  >
    <Down class="size-4 -rotate-90 transition-transform group-open:rotate-0" />
    {title}
  </summary>
  <div class="mt-3 grid gap-4">
    {#if description}
      <p class="text-base-content/70 text-sm">{description}</p>
    {/if}
    {@render children()}
  </div>
</details>
