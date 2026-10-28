import { onMount } from 'svelte'
import { beforeNavigate, goto } from '$app/navigation'
import { modals } from 'svelte-modals'
import ConfirmDialog from '$lib/components/ConfirmDialog.svelte'
import { Cancel, Check } from '$lib/components/icons'

/**
 * Holds the page while `unsaved` says it has changes that are not saved. Moving on inside the app asks first, and
 * leaving goes on to where the person was headed; a reload, a closed tab or another site is the browser's to ask
 * about. Called while a component initialises.
 */
export const guardUnsaved = (unsaved: () => boolean) => {
  let leaving = false
  beforeNavigate(navigation => {
    const { to, from, type, delta, willUnload } = navigation
    if (leaving || !unsaved() || willUnload || !to) return
    if (to.url.pathname === from?.url.pathname) return
    navigation.cancel()
    modals.open(ConfirmDialog, {
      title: 'Unsaved changes',
      message: 'This page has changes that are not saved. Leave it and lose them?',
      labels: {
        cancel: { label: 'Stay', icon: Cancel },
        confirm: { label: 'Leave', icon: Check }
      },
      onConfirm: () => {
        leaving = true
        modals.closeAll()
        if (type === 'popstate' && delta) history.go(delta)
        else goto(to.url)
      }
    })
  })

  onMount(() => {
    const ask = (event: BeforeUnloadEvent) => {
      if (!unsaved()) return
      event.preventDefault()
      // Older Chromium and Safari prompt only when returnValue is set.
      event.returnValue = ''
    }
    window.addEventListener('beforeunload', ask)
    return () => window.removeEventListener('beforeunload', ask)
  })
}
