import { readable, derived } from 'svelte/store'

export type GamepadState = {
  available: boolean
  gamepads: Gamepad[]
}

const supports = typeof navigator !== 'undefined' && 'getGamepads' in navigator

export const gamepads = readable<GamepadState>({ available: supports, gamepads: [] }, set => {
  if (!supports) return

  let raf: number | null = null
  let visible = typeof document === 'undefined' ? true : document.visibilityState !== 'hidden'
  let prevTimestamps: number[] = []

  const get = () => (navigator.getGamepads?.() ?? []).filter((g): g is Gamepad => !!g)

  const changed = (pads: Gamepad[]) => {
    if (pads.length !== prevTimestamps.length) return true
    for (let i = 0; i < pads.length; i++) if (pads[i].timestamp !== prevTimestamps[i]) return true
    return false
  }

  const tick = () => {
    if (!visible) {
      stop()
      return
    }
    const pads = get()
    if (changed(pads)) {
      prevTimestamps = pads.map(p => p.timestamp)
      set({ available: true, gamepads: pads })
    }
    if (pads.length) raf = requestAnimationFrame(tick)
    else stop()
  }

  const start = () => {
    if (raf != null || !visible) return
    raf = requestAnimationFrame(tick)
  }

  const stop = () => {
    if (raf == null) return
    cancelAnimationFrame(raf)
    raf = null
  }

  const onConnect = () => {
    const pads = get()
    set({ available: true, gamepads: pads })
    if (pads.length) start()
  }

  const onDisconnect = () => {
    const pads = get()
    set({ available: true, gamepads: pads })
    if (!pads.length) stop()
  }

  const onVisibility = () => {
    visible = typeof document === 'undefined' ? true : document.visibilityState !== 'hidden'
    if (visible) start()
    else stop()
  }

  window.addEventListener('gamepadconnected', onConnect)
  window.addEventListener('gamepaddisconnected', onDisconnect)
  if (typeof document !== 'undefined') document.addEventListener('visibilitychange', onVisibility)

  onConnect()

  return () => {
    stop()
    window.removeEventListener('gamepadconnected', onConnect)
    window.removeEventListener('gamepaddisconnected', onDisconnect)
    if (typeof document !== 'undefined')
      document.removeEventListener('visibilitychange', onVisibility)
  }
})

export const gamepad = derived(gamepads, $gamepads =>
  $gamepads.available && $gamepads.gamepads.length > 0 ? $gamepads.gamepads[0] : null
)

export const gamepadAxes = derived(gamepad, $gamepad => $gamepad?.axes ?? [0, 0, 0, 0])

export const gamepadButtons = derived(gamepad, $gamepad => $gamepad?.buttons ?? [])

export const hasGamepad = derived(
  gamepads,
  $gamepads => $gamepads.available && $gamepads.gamepads.length > 0
)
