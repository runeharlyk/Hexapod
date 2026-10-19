// Choosing a robot (src/lib/robot/data.ts): the user's choice of an address, a Bluetooth device or a serial port is the
// only sign the app gets of another robot, since the transports cannot tell one from a fail-over. It forgets what the
// app knew of the last robot, its state topics, feature flags and scans, and asks the robot reached again.
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { get } from 'svelte/store'

// The radios are stubbed: the link actions are what is under test.
const radios = vi.hoisted(() => ({ bleConnect: vi.fn(), choosePort: vi.fn() }))
vi.mock('$lib/transport/ble-adapter', async () => {
  const { writable } = await import('svelte/store')
  return { ble: { status: writable('disconnected'), connect: radios.bleConnect } }
})
vi.mock('$lib/transport/serial-adapter', async () => {
  const { writable } = await import('svelte/store')
  return {
    serial: { status: writable('disconnected'), choosePort: radios.choosePort },
    serialSupported: false
  }
})
vi.mock('$lib/transport/websocket-adapter', async () => {
  const { writable } = await import('svelte/store')
  return {
    websocket: {
      status: writable('disconnected'),
      connect: vi.fn(() => Promise.resolve()),
      disconnect: vi.fn(() => Promise.resolve())
    }
  }
})

import { dataBroker } from '$lib/transport/databroker'
import { robotData } from '$lib/robot/data'
import { modeState } from '$lib/robot/state'
import { i2cScan } from '$lib/i2c'
import { featureFlagsState, refreshFeatureFlags } from '$lib/stores/featureFlags'
import { chooseSerialPort, connectBluetooth, connectToAddress } from '$lib/stores/link'
import { location } from '$lib/stores'
import { FeaturesDataResponse, ModeData, ModesEnum } from '$lib/platform_shared/message'
import { FakeRobot, fakeLink } from '../fake-robot/transport'

const robot = new FakeRobot()
robot.respond('featuresDataRequest', () => ({
  featuresDataResponse: FeaturesDataResponse.create({ firmwareVersion: '1.2.3' })
}))
const link = fakeLink(robot)
dataBroker.addTransport(link)

const settle = () => new Promise(resolve => setTimeout(resolve, 0))

describe('robotData.reset', () => {
  let following: () => void

  beforeEach(async () => {
    await link.connect()
    following = modeState.subscribe(() => {})
    robot.set(ModeData, { mode: ModesEnum.STAND, handheld: false })
    await settle()
  })

  afterEach(async () => {
    following()
    await link.disconnect()
  })

  it('forgets the state topics, the flags and the scans, and asks the robot again', async () => {
    await refreshFeatureFlags()
    await i2cScan.run(async () => [])
    robot.set(ModeData, { mode: ModesEnum.WALK, handheld: false })
    await settle()

    robotData.reset()

    expect(get(modeState)).toMatchObject({ value: undefined, stale: false, status: 'subscribing' })
    expect(get(i2cScan)).toEqual({ state: 'idle' })
    expect(get(featureFlagsState)).toEqual({ kind: 'pending' })
    await vi.waitFor(() =>
      expect(get(modeState)).toMatchObject({ value: { mode: ModesEnum.WALK } })
    )
    await vi.waitFor(() => expect(get(featureFlagsState)).toEqual({ kind: 'known' }))
  })

  it('shows nothing of a scan of the last robot that ends after the switch', async () => {
    let finish: (rows: []) => void = () => {}
    const scanning = i2cScan.run(() => new Promise<[]>(resolve => (finish = resolve)))

    robotData.reset()
    finish([])
    await scanning

    expect(get(i2cScan)).toEqual({ state: 'idle' })
  })
})

describe('choosing a robot', () => {
  let reset: ReturnType<typeof vi.spyOn>

  beforeEach(() => {
    reset = vi.spyOn(robotData, 'reset')
    radios.bleConnect.mockReset()
    radios.choosePort.mockReset()
  })

  afterEach(() => reset.mockRestore())

  it('resets once a Bluetooth device or a serial port is chosen, not when the chooser is closed', async () => {
    const closed = new DOMException('closed', 'NotFoundError')
    radios.bleConnect.mockRejectedValueOnce(closed).mockResolvedValueOnce(undefined)
    radios.choosePort.mockRejectedValueOnce(closed).mockResolvedValueOnce(undefined)

    await connectBluetooth()
    await chooseSerialPort()
    expect(reset).not.toHaveBeenCalled()

    await connectBluetooth()
    await chooseSerialPort()
    expect(reset).toHaveBeenCalledTimes(2)
  })

  it('resets for another address, not for the one connected to', async () => {
    location.set('hexapod-a.local')

    await connectToAddress(' hexapod-a.local ')
    expect(reset).not.toHaveBeenCalled()

    await connectToAddress('hexapod-b.local')
    expect(reset).toHaveBeenCalledTimes(1)
    expect(get(location)).toBe('hexapod-b.local')
  })
})
