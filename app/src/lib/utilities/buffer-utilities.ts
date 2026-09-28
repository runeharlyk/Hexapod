// Trailing throttle: at most one call per window, and the call that runs is the latest one
// scheduled inside that window.
export class throttler {
  private pending: (() => void) | undefined
  private timer: ReturnType<typeof setTimeout> | undefined

  throttle = (callback: () => void, time: number) => {
    this.pending = callback
    if (this.timer) return
    this.timer = setTimeout(() => {
      const latest = this.pending
      this.timer = undefined
      this.pending = undefined
      latest?.()
    }, time)
  }

  cancel = () => {
    if (this.timer) clearTimeout(this.timer)
    this.timer = undefined
    this.pending = undefined
  }
}
