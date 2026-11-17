// Best-effort haptic feedback. navigator.vibrate is absent on desktop browsers and iOS Safari and is ignored until the
// user has interacted with the page, so every call is advisory.
const vibrate = (pattern: number | number[]) => {
  if (typeof navigator === 'undefined') return
  navigator.vibrate?.(pattern)
}

export const haptics = {
  /** Deactivating: a distinct double pulse, so it is not mistaken for a mode change. */
  stop: () => vibrate([40, 30, 40]),
  /** Any other mode change. */
  modeChange: () => vibrate(15)
}
