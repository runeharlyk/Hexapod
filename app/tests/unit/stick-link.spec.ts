// The controller's sticks on their way to the robot (src/lib/stick-link.ts): which frames it sends, and when. What the
// link does with them is motion-frames.spec.ts's; nothing older than the last stick event is sent after a link comes back.
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { writable } from 'svelte/store'
import { followSticks, STICK_INTERVAL_MS } from '$lib/stick-link'
import { COMMAND_HEARTBEAT_MS } from '$lib/control-link'
import { MotionModes } from '$lib/motion'
import type { ControllerInputData } from '$lib/platform_shared/message'
import type { Command } from '$lib/simulation/virtual-robot'

const CENTRED: Command = [0, 0, 0, 0, 0, 0, 0, 0, 0]
const stick = (ly: number): Command => [0, ly, 0, 0, 0, 0, 0, 0, 0]

describe('followSticks', () => {
  const commands = writable<Command>(CENTRED)
  const mode = writable(MotionModes.STAND)
  const linked = writable(true)
  let sent: ControllerInputData[]
  let stop: () => void

  beforeEach(() => {
    vi.useFakeTimers()
    commands.set(CENTRED)
    mode.set(MotionModes.STAND)
    linked.set(true)
    sent = []
    stop = followSticks({ commands, mode, linked, send: input => void sent.push(input) })
  })

  afterEach(() => {
    stop()
    vi.useRealTimers()
  })

  const forward = () => sent.map(input => input.left?.y)

  it('sends a moving stick at most once per interval, the latest, and its centring at once', () => {
    mode.set(MotionModes.IDLE)
    commands.set(stick(0.2))
    commands.set(stick(0.4))
    expect(sent).toHaveLength(0)
    vi.advanceTimersByTime(STICK_INTERVAL_MS)
    expect(forward()).toEqual([0.4])

    commands.set(CENTRED)
    expect(forward()).toEqual([0.4, 0])
  })

  it('repeats the last command as a heartbeat in the modes that need one, and only while linked', () => {
    commands.set(stick(0.5))
    vi.advanceTimersByTime(STICK_INTERVAL_MS)
    sent = []

    vi.advanceTimersByTime(2 * COMMAND_HEARTBEAT_MS)
    expect(forward()).toEqual([0.5, 0.5])

    mode.set(MotionModes.IDLE)
    vi.advanceTimersByTime(2 * COMMAND_HEARTBEAT_MS)
    expect(sent).toHaveLength(2)
  })

  // After a drop the robot's dead-man stops it; a stick held through the drop is no reason to walk again by itself.
  it('sends no heartbeat after a link comes back until the next stick event', () => {
    commands.set(stick(0.5))
    vi.advanceTimersByTime(STICK_INTERVAL_MS)
    linked.set(false)
    linked.set(true)
    sent = []

    vi.advanceTimersByTime(3 * COMMAND_HEARTBEAT_MS)
    expect(sent).toHaveLength(0)

    commands.set(stick(0.3))
    vi.advanceTimersByTime(COMMAND_HEARTBEAT_MS)
    expect(forward()).toEqual([0.3, 0.3])
  })
})
