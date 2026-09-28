import { readable } from 'svelte/store'

const isDocumentFullscreen = () => typeof document !== 'undefined' && !!document.fullscreenElement

// Follows the browser, so leaving fullscreen with Esc or F11 is reflected too.
export const isFullscreen = readable(isDocumentFullscreen(), set => {
  const update = () => set(isDocumentFullscreen())
  document.addEventListener('fullscreenchange', update)
  return () => document.removeEventListener('fullscreenchange', update)
})

export function toggleFullscreen() {
  const request =
    document.fullscreenElement ?
      document.exitFullscreen()
    : document.documentElement.requestFullscreen()
  request.catch(error => console.warn('Fullscreen change refused:', error))
}
