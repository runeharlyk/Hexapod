// A mode, a gait or an animation play moves the robot, so while no link is up it fails at once and says so, and nothing
// reaches the robot when a link comes later (src/lib/control.ts).
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { dataBroker } from '$lib/transport/databroker'
import { notifications } from '$lib/components/toasts/notifications'
import { playAnimation, requestGait, requestMode } from '$lib/control'
import { MotionModes } from '$lib/motion'
import { GaitType } from '$lib/gait'
import { FakeRobot, fakeLink } from '../fake-robot/transport'

const robot = new FakeRobot()
const link = fakeLink(robot)
dataBroker.addTransport(link)

const moves = () => robot.received.filter(m => m.mode || m.gait || m.animationPlay)

describe('the actions that move the robot', () => {
  let errors: string[]

  beforeEach(() => {
    errors = []
    vi.spyOn(notifications, 'error').mockImplementation(message => {
      errors.push(message)
      return ''
    })
    robot.received.length = 0
  })

  afterEach(async () => {
    vi.restoreAllMocks()
    await link.disconnect()
  })

  it('fail at once while no link is up, say so, and are not sent later', async () => {
    requestMode(MotionModes.WALK)
    requestGait(GaitType.WAVE)
    playAnimation('wave', new Map())

    expect(errors).toEqual([
      'Mode not changed: the robot is not connected',
      'Gait not changed: the robot is not connected',
      'wave not played: the robot is not connected'
    ])
    await link.connect()
    expect(moves()).toHaveLength(0)
  })

  it('go out at once while a link is up', async () => {
    await link.connect()

    requestMode(MotionModes.WALK)
    playAnimation('wave', new Map())

    expect(errors).toEqual([])
    expect(moves()).toHaveLength(2)
  })
})
