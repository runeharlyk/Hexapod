import { writable, type Readable } from 'svelte/store'
import { robotData } from '$lib/robot/data'

export type ScanState<R> =
  | { state: 'idle' }
  | { state: 'scanning' }
  | { state: 'done'; result: R }
  | { state: 'failed'; message: string }

export type Scan<R> = Readable<ScanState<R>> & {
  // Starts a scan unless one runs, which the caller then waits for instead; settles when the scan has.
  run: (request: () => Promise<R>) => Promise<void>
}

// A page's scan of the robot, kept outside the page: a page mounted again by a hot reload or a reconnect, while its
// first scan is still out, joins that scan instead of starting a second one on the robot. Another robot chosen
// (robotData.reset) forgets the scan, and a scan of the last robot that ends after it shows nothing.
export const createScan = <R>(): Scan<R> => {
  const { subscribe, set } = writable<ScanState<R>>({ state: 'idle' })
  let running: Promise<void> | null = null
  let robot = 0

  robotData.register(() => {
    robot++
    running = null
    set({ state: 'idle' })
  })

  const run = (request: () => Promise<R>) => {
    if (running) return running
    const scanned = robot
    const show = (state: ScanState<R>) => {
      if (scanned === robot) set(state)
    }
    set({ state: 'scanning' })
    const scan: Promise<void> = request()
      .then(
        result => show({ state: 'done', result }),
        (error: unknown) =>
          show({ state: 'failed', message: error instanceof Error ? error.message : String(error) })
      )
      .finally(() => {
        if (running === scan) running = null
      })
    running = scan
    return scan
  }

  return { subscribe, run }
}
