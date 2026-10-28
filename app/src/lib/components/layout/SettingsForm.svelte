<script lang="ts" generics="T extends object">
  import { onMount, type Snippet } from 'svelte'
  import type { IconComponent } from '$lib/components/icons'
  import LoadFailure from './LoadFailure.svelte'
  import Section from './Section.svelte'
  import { guardUnsaved } from './unsaved-guard'
  import type { Settings } from './settings.svelte'

  interface Props {
    title: string
    description?: string
    icon?: IconComponent
    settings: Settings<T>
    /** Placeholder rows while the settings are read: as many as the form shows fields. */
    rows?: number
    /** Beside the title, shown once the settings are read: what adds to them, such as a scan. */
    actions?: Snippet
    /** Asks before a save, and calls `save` to go ahead. */
    confirm?: (save: () => void) => void
    /** Edits held outside the draft, such as an open editor's, which leaving the page would lose as well. */
    pending?: boolean
    /**
     * The fields, bound to the draft it is given. Each input of a watched form is named by its dotted path in the group;
     * `notes` holds the robot's value of each field it changed under the user's edit, by field (`fieldOf`).
     */
    children: Snippet<[T, Record<string, unknown>]>
  }

  const {
    title,
    description,
    icon,
    settings,
    rows = 3,
    actions,
    confirm,
    pending = false,
    children
  }: Props = $props()

  const placeholders = $derived(Array.from({ length: rows }, (_, row) => row))

  onMount(() => {
    if (settings.follows) return settings.start()
    settings.load()
  })

  const nameOf = (event: FocusEvent) => (event.target as HTMLInputElement).name

  const save = () => settings.save()

  let formElement: HTMLFormElement

  /** Saves as the Save button does, through the browser's check of every field and the confirmation. */
  export const requestSave = () => formElement.requestSubmit()

  const unsaved = $derived(settings.dirty || pending)

  guardUnsaved(() => unsaved)

  // The browser has checked every field against its constraints before this runs.
  const submit = (event: SubmitEvent) => {
    event.preventDefault()
    if (confirm) confirm(save)
    else save()
  }
</script>

{#snippet commit()}
  <button
    type="button"
    class="btn btn-ghost"
    disabled={!settings.dirty || settings.pending}
    onclick={() => settings.reset()}
  >
    Reset
  </button>
  <button type="submit" class="btn btn-primary" disabled={!settings.dirty || settings.pending}>
    {#if settings.pending}<span class="loading loading-spinner loading-xs"></span>{/if}
    Save
  </button>
{/snippet}

<form
  bind:this={formElement}
  onsubmit={submit}
  onfocusin={event => settings.focus(nameOf(event))}
  onfocusout={event => settings.blur(nameOf(event))}
  aria-busy={settings.status === 'loading' || settings.pending}
>
  <Section
    {title}
    {description}
    {icon}
    actions={settings.status === 'ready' ? actions : undefined}
    footer={settings.status === 'ready' ? commit : undefined}
  >
    {#if settings.status === 'loading'}
      <div class="grid gap-4" role="status" aria-label="Reading {settings.what}">
        {#each placeholders as row (row)}
          <div class="grid gap-x-6 gap-y-1.5 sm:grid-cols-[11rem_minmax(0,28rem)]">
            <div class="skeleton h-4 w-24 sm:mt-3"></div>
            <div class="skeleton h-10 w-full"></div>
          </div>
        {/each}
      </div>
    {:else if settings.status === 'failed'}
      <LoadFailure
        message={settings.error}
        retry={settings.follows ? undefined : () => settings.load()}
      />
    {:else}
      <fieldset class="fieldset gap-4 p-0 text-sm" disabled={settings.pending}>
        {@render children(settings.value, settings.notes)}
      </fieldset>
    {/if}
  </Section>
</form>
