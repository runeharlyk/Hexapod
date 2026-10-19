import type { Readable } from 'svelte/store'
import type { ControllerInputData } from '$lib/platform_shared/message'
import type { MotionModes } from '$lib/motion'
import type { Command } from '$lib/simulation/virtual-robot'
import { COMMAND_HEARTBEAT_MS, needsHeartbeat } from './control-link'
import { toControllerInput } from './controller-input'
import { throttler } from './utilities'

/** A moving stick goes out at most this often. */
export const STICK_INTERVAL_MS = 40

const centred = (c: Command) => c[0] === 0 && c[1] === 0 && c[2] === 0 && c[3] === 0

/**
 * Sends the controller's sticks to the robot: a moving stick at most every STICK_INTERVAL_MS, its centring at once, and
 * the last command again every COMMAND_HEARTBEAT_MS in the modes that need a heartbeat. Every frame is motion, which
 * `send` drops when the link cannot take it now (DataBroker.emitVolatile). A link that drops forgets the last command,
 * so no heartbeat repeats it after the link comes back until the next stick event; the firmware's dead-man stops the
 * robot meanwhile. Returns the function that stops sending.
 */
export const followSticks = ({
  commands,
  mode,
  linked,
  send
}: {
  commands: Readable<Command>
  mode: Readable<MotionModes>
  linked: Readable<boolean>
  send: (input: ControllerInputData) => void
}) => {
  const throttle = new throttler()
  let last: Command | undefined
  let wasCentred = true
  let currentMode: MotionModes | undefined
  let isLinked = false

  const stops = [
    commands.subscribe(command => {
      last = [...command]
      if (centred(command)) {
        throttle.cancel()
        if (!wasCentred) send(toControllerInput(command))
        wasCentred = true
      } else {
        wasCentred = false
        throttle.throttle(() => send(toControllerInput(command)), STICK_INTERVAL_MS)
      }
    }),
    mode.subscribe(value => (currentMode = value)),
    linked.subscribe(up => {
      isLinked = up
      if (up) return
      throttle.cancel()
      last = undefined
    })
  ]
  const heartbeat = setInterval(() => {
    if (isLinked && last && currentMode && needsHeartbeat(currentMode))
      send(toControllerInput(last))
  }, COMMAND_HEARTBEAT_MS)

  return () => {
    clearInterval(heartbeat)
    throttle.cancel()
    stops.forEach(stop => stop())
  }
}
