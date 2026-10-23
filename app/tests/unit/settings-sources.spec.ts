// A settings form follows its topic through the feature flags' gate (src/lib/robot/settings-sources.ts). The flags are
// read again whenever the link reaches the robot another way or at another address: a fail-over, or the saved robot
// following a rename. The draft must outlive that read.
import { beforeEach, describe, expect, it, vi } from 'vitest'
import { writable, type Writable } from 'svelte/store'
import type { WifiSettings } from '$lib/platform_shared/api'
import type { ITransport } from '$lib/interfaces/transport.interface'
import type { Settings } from '$lib/components/layout/settings.svelte'
import { fakeLink } from '../fake-robot/transport'
import { stockRobot, type StockRobot } from '../fake-robot/stock'

let robot: StockRobot
let websocket: ITransport
let reached: Writable<string | null>
let settings: Settings<WifiSettings>

const featureReads = () => robot.requests('featuresDataRequest').length

// The broker and the flags are module state, and the broker knows a topic by its message type: each test loads them
// afresh, the types with them.
beforeEach(async () => {
  vi.resetModules()
  const { WifiSettings } = await import('$lib/platform_shared/api')
  const { dataBroker } = await import('$lib/transport/databroker')
  const { followFeatureFlags } = await import('$lib/stores/featureFlags')
  const { topicSource } = await import('$lib/robot/settings-sources')
  const { Settings } = await import('$lib/components/layout/settings.svelte')
  robot = stockRobot()
  websocket = fakeLink(robot)
  const ble = fakeLink(robot)
  dataBroker.addTransport(websocket)
  dataBroker.addTransport(ble)
  await websocket.connect()
  await ble.connect()
  reached = writable('websocket hexapod.local')
  followFeatureFlags(reached)
  settings = new Settings(
    topicSource(WifiSettings, {
      what: 'the Wi-Fi settings',
      saved: 'Saved.',
      update: value => ({ wifiSettingsUpdate: value }),
      together: [['wifiNetworks', 'selectedNetwork']]
    })
  )
  settings.start()
  await vi.waitFor(() => expect(settings.status).toBe('ready'))
  settings.value.hostname = 'mine'
})

// The flags read again, and another client's change after: the form still follows the topic and keeps the draft.
const flagsReadAgain = async () => {
  await vi.waitFor(() => expect(featureReads()).toBe(2))
  await vi.waitFor(() => expect(settings.status).toBe('ready'))
  robot.hold('wifiSettings', { ...robot.held.wifiSettings, priorityRssi: false })
  await vi.waitFor(() => expect(settings.value.priorityRssi).toBe(false))
}

describe('a settings form whose robot is reached anew', () => {
  it('keeps its draft when the saved robot follows a rename', async () => {
    reached.set('websocket kitchen.local')
    await flagsReadAgain()

    expect(settings.value.hostname).toBe('mine')
    expect(settings.dirty).toBe(true)
  })

  it('keeps its draft through a fail-over to another link', async () => {
    await websocket.disconnect()
    reached.set('ble')
    await flagsReadAgain()

    expect(settings.value.hostname).toBe('mine')
    expect(settings.dirty).toBe(true)
  })
})
