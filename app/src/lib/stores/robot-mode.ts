import { writable, type Readable, type Writable } from 'svelte/store'
import { ModeData } from '$lib/platform_shared/message'
import { MotionModes } from '$lib/motion'
import { mode } from './model-store'
import { ask, type Reply } from '$lib/transport/reply'

/** Whether the ESP-NOW handheld drives the robot, which then drops this app's sticks and refuses its goals. */
export const handheldDriving: Writable<boolean> = writable(false)

/** What the robot's mode needs of the data broker (transport/databroker.ts): its pushes and one request. */
export interface ModeLink {
  on(fns: typeof ModeData, callback: (data: ModeData) => void): () => void
  request(payload: { modeGet: object }): Promise<{ mode?: ModeData } & Reply>
}

const show = (data: ModeData) => {
  const next = Object.values(MotionModes)[data.mode]
  if (next !== undefined) mode.set(next)
  handheldDriving.set(data.handheld)
}

/**
 * Follows the robot's mode and whether the handheld drives: every push sets both, and the flag clears when the link
 * drops. The robot pushes only on a change, so the current value is requested once each time the link comes up; a push
 * that beat the reply is newer than it. Returns the function that stops following.
 */
export const followMode = (broker: ModeLink, linked: Readable<boolean>) => {
  let pushed = false
  const stopPushes = broker.on(ModeData, data => {
    pushed = true
    show(data)
  })
  const requestCurrent = async () => {
    pushed = false
    // A firmware without the request, or a link that drops, leaves the mode to the pushes.
    const res = await ask(broker, { modeGet: {} }, 'the mode').catch(() => null)
    if (res?.mode && !pushed) show(res.mode)
  }
  const stopLink = linked.subscribe(up => {
    if (!up) return handheldDriving.set(false)
    requestCurrent()
  })
  return () => {
    stopPushes()
    stopLink()
  }
}
