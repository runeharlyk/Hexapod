<script lang="ts">
  import { get } from 'svelte/store'
  import { modals } from 'svelte-modals'
  import ConfirmDialog from '$lib/components/ConfirmDialog.svelte'
  import { Cancel, Check } from '$lib/components/icons'
  import { notifications } from '$lib/components/toasts/notifications'
  import type { Animation } from '$lib/platform_shared/animation'
  import type { DraftRef } from '$lib/animation/library'
  import { editor } from '$lib/stores/animation-editor'
  import Library from './Library.svelte'
  import Editor from './Editor.svelte'

  let view = $state<'library' | 'editor'>('library')

  const replaceDocument = (a: Animation | null, draft?: DraftRef) => {
    const error = a ? editor.open(a, draft) : (editor.newDocument(), null)
    if (error) notifications.error(`Cannot open ${a?.name}: ${error}`, 5000)
    else view = 'editor'
  }

  // Both views open documents into the one editor, so unsaved edits are confirmed here.
  const openDocument = (a: Animation | null, draft?: DraftRef) => {
    if (!get(editor).dirty) return replaceDocument(a, draft)
    modals.open(ConfirmDialog, {
      title: 'Discard changes',
      message: `${get(editor).document.name} has unsaved changes. Discard them?`,
      labels: {
        cancel: { label: 'Keep', icon: Cancel },
        confirm: { label: 'Discard', icon: Check }
      },
      onConfirm: () => {
        modals.close()
        replaceDocument(a, draft)
      }
    })
  }

  // Unsaved edits normally survive a reload through the autosave; only a refused one asks.
  const guardUnload = (e: BeforeUnloadEvent) => {
    if (!get(editor).dirty || get(editor.autosaved)) return
    e.preventDefault()
    // Older Chromium and Safari prompt only when returnValue is set.
    e.returnValue = ''
  }
</script>

<svelte:window onbeforeunload={guardUnload} />

<div class="mx-0 my-1 flex flex-col space-y-4 sm:mx-8 sm:my-8">
  <div role="tablist" class="tabs tabs-box self-start">
    <button
      role="tab"
      class="tab"
      class:tab-active={view === 'library'}
      onclick={() => (view = 'library')}>Library</button
    >
    <button
      role="tab"
      class="tab"
      class:tab-active={view === 'editor'}
      onclick={() => (view = 'editor')}>Editor</button
    >
  </div>
  {#if view === 'library'}
    <Library onEdit={openDocument} />
  {:else}
    <Editor onOpen={openDocument} />
  {/if}
</div>
