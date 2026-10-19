import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { get, writable } from 'svelte/store'
import { ModeData, ModesEnum } from '../../src/lib/platform_shared/message'
import { MotionModes } from '../../src/lib/motion'
import { mode } from '../../src/lib/stores/model-store'
import { followMode, handheldDriving, type ModeLink } from '../../src/lib/stores/robot-mode'
import { RequestFailed, type Reply } from '../../src/lib/transport/reply'

// The data broker as followMode uses it: its mode pushes, and requests answered when the test says.
const fakeBroker = () => {
  const pushes = new Set<(m: ModeData) => void>()
  const replies: ((reply: { mode?: ModeData } & Reply) => void)[] = []
  const rejections: ((error: Error) => void)[] = []
  const broker: ModeLink = {
    on: (_fns, callback) => {
      pushes.add(callback)
      return () => pushes.delete(callback)
    },
    request: () =>
      new Promise((resolve, reject) => {
        replies.push(resolve)
        rejections.push(reject)
      })
  }
  return {
    broker,
    push: (m: ModeData) => pushes.forEach(callback => callback(m)),
    replies,
    rejections,
    listening: () => pushes.size
  }
}

const ok = (fields: { mode?: ModeData }) => ({ statusCode: 200, errorMessage: '', ...fields })

// Every reply handed to the fake has been read once the microtasks queued before this macrotask have run.
const settled = () => new Promise(resolve => setTimeout(resolve, 0))

describe('followMode', () => {
  const linked = writable(false)
  let stop: () => void = () => {}

  beforeEach(() => {
    linked.set(false)
    mode.set(MotionModes.DEACTIVATED)
    handheldDriving.set(false)
  })
  afterEach(() => stop())

  it('follows the mode and the handheld flag on every push', () => {
    const fake = fakeBroker()
    stop = followMode(fake.broker, linked)
    fake.push({ mode: ModesEnum.WALK, handheld: true })
    expect(get(mode)).toBe(MotionModes.WALK)
    expect(get(handheldDriving)).toBe(true)
    fake.push({ mode: ModesEnum.WALK, handheld: false })
    expect(get(handheldDriving)).toBe(false)
  })

  it('reads a frame from a firmware without the flag as no handheld', () => {
    const fake = fakeBroker()
    stop = followMode(fake.broker, linked)
    handheldDriving.set(true)
    fake.push(ModeData.decode(Uint8Array.of(0x08, ModesEnum.STAND)))
    expect(get(mode)).toBe(MotionModes.STAND)
    expect(get(handheldDriving)).toBe(false)
  })

  it('asks once when the link comes up, for a mode and a drive that began before it listened', async () => {
    const fake = fakeBroker()
    stop = followMode(fake.broker, linked)
    expect(fake.replies).toHaveLength(0)
    linked.set(true)
    expect(fake.replies).toHaveLength(1)
    fake.replies[0](ok({ mode: { mode: ModesEnum.STAND, handheld: true } }))
    await vi.waitFor(() => expect(get(handheldDriving)).toBe(true))
    expect(get(mode)).toBe(MotionModes.STAND)
  })

  it('keeps a push that beat the reply, as it is newer', async () => {
    const fake = fakeBroker()
    stop = followMode(fake.broker, linked)
    linked.set(true)
    fake.push({ mode: ModesEnum.WALK, handheld: false })
    fake.replies[0](ok({ mode: { mode: ModesEnum.STAND, handheld: true } }))
    await settled()
    expect(get(mode)).toBe(MotionModes.WALK)
    expect(get(handheldDriving)).toBe(false)
  })

  it('leaves the stores to the pushes when the request is refused or lost', async () => {
    const fake = fakeBroker()
    stop = followMode(fake.broker, linked)
    linked.set(true)
    fake.replies[0]({ statusCode: 400, errorMessage: '' }) // a firmware from before the request
    linked.set(false)
    linked.set(true)
    fake.rejections[1](new RequestFailed({ kind: 'timeout' }))
    await settled()
    expect(get(mode)).toBe(MotionModes.DEACTIVATED)
    expect(get(handheldDriving)).toBe(false)
  })

  it('clears the flag at once when the link drops', () => {
    const fake = fakeBroker()
    stop = followMode(fake.broker, linked)
    linked.set(true)
    fake.push({ mode: ModesEnum.WALK, handheld: true })
    linked.set(false)
    expect(get(handheldDriving)).toBe(false)
  })

  it('stops listening when stopped', () => {
    const fake = fakeBroker()
    followMode(fake.broker, linked)()
    expect(fake.listening()).toBe(0)
    linked.set(true)
    expect(fake.replies).toHaveLength(0)
  })
})
