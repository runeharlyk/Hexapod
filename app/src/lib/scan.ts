import { writable, type Readable } from 'svelte/store'
import type { CorrelationResponse } from '$lib/platform_shared/message'

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
// first scan is still out, joins that scan instead of starting a second one on the robot.
export const createScan = <R>(): Scan<R> => {
  const { subscribe, set } = writable<ScanState<R>>({ state: 'idle' })
  let running: Promise<void> | null = null

  const run = (request: () => Promise<R>) => {
    if (running) return running
    set({ state: 'scanning' })
    running = request()
      .then(
        result => set({ state: 'done', result }),
        (error: unknown) =>
          set({ state: 'failed', message: error instanceof Error ? error.message : String(error) })
      )
      .finally(() => {
        running = null
      })
    return running
  }

  return { subscribe, run }
}

// The data broker resolves every reply, refusals included; a refused scan must fail rather than read as empty.
export const answered = (res: CorrelationResponse): CorrelationResponse => {
  if (res.statusCode !== 200)
    throw new Error(res.errorMessage || `the robot answered with status ${res.statusCode}`)
  return res
}
