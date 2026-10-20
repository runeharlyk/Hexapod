// Motion frames (sticks, heartbeats, preview poses; DataBroker.emitVolatile) on each real transport. A frame goes out
// at once on an idle link. While a write is under way, only the newest frame of each kind waits, and it goes out once
// the write completes, so the last frame, such as the stop of a released stick, always reaches the robot, while no
// frame queues behind older ones. Nothing waits while the link is down, so nothing old is sent after a reconnect.
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { get, writable } from 'svelte/store'
import { DataBroker } from '$lib/transport/databroker'
import type { ITransport } from '$lib/interfaces/transport.interface'
import {
  ControllerInputData,
  JointAngleData,
  Message,
  PoseData
} from '$lib/platform_shared/message'
import { followSticks, STICK_INTERVAL_MS } from '$lib/stick-link'
import { MotionModes } from '$lib/motion'
import type { Command } from '$lib/simulation/virtual-robot'
import { createSerialAdapter } from '$lib/transport/serial-adapter'
import { FakeSerialPort, fakeSerial, pongs, serialRobot } from './fake-serial'

/** A transport over a stand-in for the browser's API, whose writes can be held under way. */
interface Rig {
  transport: ITransport
  connect(): Promise<void>
  /** The writes from now stay under way until `release`. */
  hold(): void
  release(): void
  /** The frames that reached the robot, in order. */
  received(): Message[]
}

const serialRig = (): Rig => {
  const port = new FakeSerialPort()
  vi.stubGlobal('navigator', { serial: fakeSerial([], port).serial })
  const robot = serialRobot(port, pongs)
  const transport = createSerialAdapter()
  let release = () => {}
  return {
    transport,
    connect: async () => {
      await transport.connect()
      await vi.waitFor(() => expect(get(transport.status)).toBe('connected'))
    },
    hold: () => (release = port.hold()),
    release: () => release(),
    received: () => robot.received
  }
}

const bleRig = async (): Promise<Rig> => {
  const received: Message[] = []
  let held: Promise<void> | undefined
  let release = () => {}
  const rx = {
    writeValueWithoutResponse: async (chunk: Uint8Array) => {
      await held
      received.push(Message.decode(chunk.slice(2)))
    }
  }
  const tx = { startNotifications: async () => {}, addEventListener: () => {} }
  const service = {
    getCharacteristic: async (uuid: string) => (uuid.startsWith('6e400002') ? rx : tx)
  }
  const gatt = {
    connected: false,
    connect: async () => {
      gatt.connected = true
      return { getPrimaryService: async () => service }
    },
    disconnect: () => (gatt.connected = false)
  }
  const device = { name: 'Hexapod', gatt, addEventListener: () => {} }
  vi.stubGlobal('navigator', { bluetooth: { requestDevice: async () => device } })
  const { ble } = await import('$lib/transport/ble-adapter')
  return {
    transport: ble,
    connect: () => ble.connect(),
    hold: () => {
      held = new Promise(resolve => (release = resolve))
    },
    release: () => {
      held = undefined
      release()
    },
    received: () => received
  }
}

const websocketRig = async (): Promise<Rig & { serverCloses(): void }> => {
  const received: Message[] = []
  let holding = false
  // The browser's WebSocket: bufferedAmount counts what send() took and the network has not, here while held.
  class FakeSocket {
    static readonly CONNECTING = 0
    static readonly OPEN = 1
    static readonly CLOSING = 2
    static readonly CLOSED = 3
    static last: FakeSocket | undefined
    readyState = FakeSocket.CONNECTING
    bufferedAmount = 0
    binaryType = 'blob'
    onopen: (() => void) | null = null
    onclose: (() => void) | null = null
    onmessage: (() => void) | null = null
    onerror: (() => void) | null = null
    constructor() {
      FakeSocket.last = this
      queueMicrotask(() => {
        this.readyState = FakeSocket.OPEN
        this.onopen?.()
      })
    }
    send(data: Uint8Array) {
      received.push(Message.decode(data))
      if (holding) this.bufferedAmount += data.length
    }
    close() {
      this.readyState = FakeSocket.CLOSED
    }
  }
  vi.stubGlobal('WebSocket', FakeSocket)
  const { websocket } = await import('$lib/transport/websocket-adapter')
  return {
    transport: websocket,
    connect: async () => {
      await websocket.connect()
      await vi.waitFor(() => expect(get(websocket.status)).toBe('connected'))
    },
    hold: () => (holding = true),
    release: () => {
      holding = false
      if (FakeSocket.last) FakeSocket.last.bufferedAmount = 0
    },
    // What the network took: a frame is on its way once send() has it.
    received: () => received,
    // The robot closes the socket; the browser's close event follows the closing handshake, later.
    serverCloses: () => {
      if (FakeSocket.last) FakeSocket.last.readyState = FakeSocket.CLOSING
    }
  }
}

const RIGS: [string, () => Rig | Promise<Rig>][] = [
  ['serial', serialRig],
  ['BLE', bleRig],
  ['WebSocket', websocketRig]
]

const sticks = (y: number) => ControllerInputData.create({ left: { x: 0, y } })
const forward = (messages: Message[]) => messages.flatMap(m => (m.input ? [m.input.left?.y] : []))

describe.each(RIGS)('motion frames over %s', (_, makeRig) => {
  let rig: Rig
  let broker: DataBroker

  beforeEach(async () => {
    vi.resetModules()
    rig = await makeRig()
    broker = new DataBroker()
    broker.addTransport(rig.transport)
    await rig.connect()
  })

  afterEach(async () => {
    rig.release()
    await rig.transport.disconnect()
    vi.unstubAllGlobals()
  })

  it('sends a frame at once on an idle link', async () => {
    expect(broker.emitVolatile(ControllerInputData, sticks(0.5))).toBe(true)
    await vi.waitFor(() => expect(forward(rig.received())).toEqual([0.5]))
  })

  it('sends only the newest of the frames sent during a write, once the write completes', async () => {
    rig.hold()
    broker.emitVolatile(ControllerInputData, sticks(0.125))
    broker.emitVolatile(ControllerInputData, sticks(0.25))
    broker.emitVolatile(ControllerInputData, sticks(0.375))
    broker.emitVolatile(ControllerInputData, sticks(-0.5))
    await new Promise(resolve => setTimeout(resolve, 30))
    expect(forward(rig.received()).filter(y => y !== 0.125)).toEqual([])

    rig.release()

    await vi.waitFor(() => expect(forward(rig.received())).toEqual([0.125, -0.5]))
  })

  it('keeps the newest frame of each kind, not one for all', async () => {
    rig.hold()
    broker.emitVolatile(ControllerInputData, sticks(0.125))
    broker.emitVolatile(ControllerInputData, sticks(0.25))
    broker.emitVolatile(
      PoseData,
      PoseData.create({ body: { roll: 0, pitch: 0, yaw: 0, x: 0, y: 0, z: 7 } })
    )

    rig.release()

    await vi.waitFor(() =>
      expect(
        rig.received().flatMap(m =>
          m.input ? ['input']
          : m.pose ? ['pose']
          : []
        )
      ).toEqual(['input', 'input', 'pose'])
    )
    expect(forward(rig.received())).toEqual([0.125, 0.25])
  })

  // A drag moves one joint after another, so each servo is a kind of its own: the newest angle of every joint dragged
  // during a write goes out, and no joint's last angle replaces another's.
  it('keeps the newest angle of each joint, not one for all the joints', async () => {
    rig.hold()
    const joint = (servo: number, degrees: number) =>
      broker.emitVolatile(JointAngleData, { servo, degrees }, servo)
    joint(4, 1)
    joint(4, 2)
    joint(4, 3)
    joint(7, 10)
    joint(7, 11)

    rig.release()

    await vi.waitFor(() =>
      expect(
        rig
          .received()
          .flatMap(m => (m.jointAngle ? [[m.jointAngle.servo, m.jointAngle.degrees]] : []))
      ).toEqual([
        [4, 1],
        [4, 3],
        [7, 11]
      ])
    )
  })

  it('sends nothing that waited for a link that went down, and takes nothing while it is down', async () => {
    rig.hold()
    broker.emitVolatile(ControllerInputData, sticks(0.125))
    broker.emitVolatile(ControllerInputData, sticks(0.25))

    const closing = rig.transport.disconnect()
    rig.release()
    await closing

    expect(broker.emitVolatile(ControllerInputData, sticks(0.375))).toBe(false)
    await new Promise(resolve => setTimeout(resolve, 30))
    expect(forward(rig.received()).filter(y => y !== 0.125)).toEqual([])
  })

  // The review's case: driving at the stick rate, the user lets go while the last stick is still being written. The
  // centred frame is the stop; dropped, the robot walks on until the firmware's dead-man.
  it('stops the robot when the stick is released while a frame is being written', async () => {
    const CENTRED: Command = [0, 0, 0, 0, 0, 0, 0, 0, 0]
    const commands = writable<Command>(CENTRED)
    const stop = followSticks({
      commands,
      mode: writable(MotionModes.IDLE),
      linked: writable(true),
      send: input => broker.emitVolatile(ControllerInputData, input)
    })
    commands.set([0, 0.5, 0, 0, 0, 0, 0, 0, 0])
    await new Promise(resolve => setTimeout(resolve, STICK_INTERVAL_MS + 10))
    await vi.waitFor(() => expect(forward(rig.received())).toEqual([0.5]))

    rig.hold()
    commands.set([0, 0.75, 0, 0, 0, 0, 0, 0, 0])
    await new Promise(resolve => setTimeout(resolve, STICK_INTERVAL_MS + 10))
    commands.set(CENTRED)
    rig.release()

    await vi.waitFor(() => expect(forward(rig.received()).at(-1)).toBe(0))
    stop()
  })
})

describe('motion frames over a WebSocket replaced while it closes', () => {
  afterEach(() => vi.unstubAllGlobals())

  // A connect while the robot's close is still under way opens a new socket before the old one's close event, which
  // then finds another socket in its place. A frame that waited for the old socket would move the robot with the past.
  it('sends nothing that waited for the socket replaced', async () => {
    vi.resetModules()
    const rig = await websocketRig()
    const broker = new DataBroker()
    broker.addTransport(rig.transport)
    await rig.connect()
    rig.hold()
    broker.emitVolatile(ControllerInputData, sticks(0.125))
    broker.emitVolatile(ControllerInputData, sticks(0.25))

    rig.serverCloses()
    await rig.connect()
    rig.release()

    await new Promise(resolve => setTimeout(resolve, 30))
    expect(forward(rig.received())).toEqual([0.125])
    await rig.transport.disconnect()
  })
})
