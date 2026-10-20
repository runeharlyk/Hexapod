// The fake robot's subscription contract, the one firmware/test/test_topic_snapshot asserts on the firmware: a state
// topic answers a subscribe with its value and then the ack, a stream with the ack alone, and a repeated subscribe is
// answered again without doubling the pushes. The app's specs run against this fake, so it must not drift from the robot.
import { describe, expect, it } from 'vitest'
import { FakeRobot, type Client } from '../fake-robot/robot'
import { IMUData, Message, ModeData, SubscribeNotification } from '$lib/platform_shared/message'
import { tagOf } from '$lib/transport/message-codec'

const MODE = tagOf(ModeData)
const IMU = tagOf(IMUData)

const connected = (robot: FakeRobot) => {
  const frames: Message[] = []
  const client: Client = { deliver: bytes => frames.push(Message.decode(bytes)) }
  robot.connect(client)
  const subscribe = (tag: number) =>
    robot.receive(
      client,
      Message.encode(Message.create({ subNotif: SubscribeNotification.create({ tag }) })).finish()
    )
  const lines = () =>
    frames.map(m =>
      m.subNotif ? `ack ${m.subNotif.tag}`
      : m.mode ? `mode ${m.mode.mode}`
      : m.imu ? `imu ${m.imu.x}`
      : 'other'
    )
  return { subscribe, lines, frames }
}

describe('the fake robot', () => {
  it('answers a subscribe to a state topic with its value, then the ack', () => {
    const robot = new FakeRobot()
    robot.set(ModeData, { mode: 3, handheld: false })
    const client = connected(robot)

    client.subscribe(MODE)

    expect(client.lines()).toEqual(['mode 3', `ack ${MODE}`])
  })

  it('answers a stream and a tag that is no topic with the ack alone', () => {
    const client = connected(new FakeRobot())

    client.subscribe(IMU)
    client.subscribe(999)

    expect(client.lines()).toEqual([`ack ${IMU}`, 'ack 999'])
  })

  it('answers a repeated subscribe again and pushes each change once', () => {
    const robot = new FakeRobot()
    const client = connected(robot)
    client.subscribe(MODE)
    client.subscribe(MODE)
    client.frames.length = 0

    robot.set(ModeData, { mode: 4, handheld: false })
    robot.push(IMUData, IMUData.create({ x: 1 }))

    expect(client.lines()).toEqual(['mode 4'])
  })

  it('cannot be built without a value for every state topic', () => {
    expect(() => new FakeRobot({ mode: ModeData.create() })).toThrow(/gait/)
  })
})
