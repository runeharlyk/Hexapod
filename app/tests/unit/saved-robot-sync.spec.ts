// The saved robot kept true to the robot the link reaches (src/lib/robot/saved-robot-sync.ts): its identity is the
// robot_id of the feature flags, its IP the WiFi status's, its name the mDNS settings'. The topics and flags are plain
// stores here: what is under test is what the sync makes of them.
import { beforeEach, describe, expect, it, vi } from 'vitest'
import { get, writable } from 'svelte/store'
import type { MDNSSettings, WifiStatus } from '$lib/platform_shared/api'
import type { Features, FeatureFlagsState } from '$lib/stores/featureFlags'
import type { TransportKind } from '$lib/stores/link'
import type { Robot } from '$lib/stores/robots'
import type { Watched } from '$lib/transport/topics'

const A = 'aaaaaaaaaaaa'
const B = 'bbbbbbbbbbbb'
const IP = '192.168.1.50'
// 192.168.1.50 as the firmware packs it: byte 0 is the first octet.
const IP_U32 = 192 + (168 << 8) + (1 << 16) + (50 << 24)

const featuresOf = (robotId: string): Features => ({
  firmware_name: 'Hexapod',
  firmware_version: '0.2.0',
  firmware_built_target: 'esp32-wroom-camera',
  firmware_elf_sha256: '',
  camera: true,
  imu: true,
  mag: true,
  bmp: false,
  servo: true,
  mdns: true,
  settings_topics: true,
  robot_id: robotId,
  analytics: true,
  sleep: true,
  ota: true,
  download_firmware: true,
  upload_firmware: true
})

const watched = <T>(value: T | undefined, stale = false): Watched<T> => ({
  value,
  status: 'live',
  stale,
  failure: null,
  pending: false
})

const mdnsOf = (hostname: string, stale = false) =>
  watched<MDNSSettings>({ hostname, instance: '', services: [], globalTxtRecords: [] }, stale)

const wifiOf = (localIp: number) =>
  watched<WifiStatus>({
    status: 3,
    localIp,
    macAddress: '',
    rssi: -50,
    ssid: '',
    bssid: '',
    channel: 1,
    subnetMask: 0,
    gatewayIp: 0,
    dnsIp1: 0,
    dnsIp2: 0
  })

const loadSync = async (saved: object[], at: { location: string; pageHostname?: string }) => {
  vi.resetModules()
  const { robots } = await import('$lib/stores/robots')
  const { location } = await import('$lib/stores/location-store')
  robots.set(saved.map(r => ({ lastSeenAt: null, ...r })) as Robot[])
  location.set(at.location)
  const { fallbackHost } = await import('$lib/stores/robot-host')
  const { notifications } = await import('$lib/components/toasts/notifications')
  const { startSavedRobotSync } = await import('$lib/robot/saved-robot-sync')

  const transport = writable<TransportKind | null>(null)
  const features = writable<Features>(featuresOf(''))
  const featuresState = writable<FeatureFlagsState>({ kind: 'pending' })
  const mdns = writable<Watched<MDNSSettings>>(watched<MDNSSettings>(undefined))
  const wifi = writable<Watched<WifiStatus>>(watched<WifiStatus>(undefined))
  const disconnect = vi.fn(async () => {})
  const stop = startSavedRobotSync({
    transport,
    features,
    featuresState,
    mdns,
    wifi,
    disconnect,
    pageHostname: at.pageHostname ?? 'localhost'
  })

  // A link comes up and its feature flags are read on it, as followFeatureFlags does.
  const reach = (via: TransportKind, robotId: string) => {
    transport.set(via)
    features.set(featuresOf(robotId))
    featuresState.set({ kind: 'known' })
  }
  return {
    robots,
    location,
    fallbackHost,
    notifications,
    transport,
    features,
    mdns,
    wifi,
    disconnect,
    stop,
    reach
  }
}

const saved = (address: string, robotId: string, rest: object = {}) => ({
  name: address,
  address,
  robotId,
  ...rest
})

describe('startSavedRobotSync', () => {
  beforeEach(() => vi.useFakeTimers())

  it('a renamed robot moves its saved address and the location', async () => {
    const t = await loadSync([saved('esp32.local', A), saved('hexapod.local', B)], {
      location: 'esp32.local'
    })
    t.mdns.set(mdnsOf('kitchen'))

    t.reach('websocket', A)

    expect(get(t.robots).map(r => r.address)).toEqual(['kitchen.local', 'hexapod.local'])
    expect(get(t.location)).toBe('kitchen.local')
  })

  it('moves the location only when it held the old address', async () => {
    const t = await loadSync([saved('esp32.local', A)], { location: '192.168.1.99' })
    t.mdns.set(mdnsOf('kitchen'))

    t.reach('websocket', A)

    expect(get(t.robots)[0].address).toBe('kitchen.local')
    expect(get(t.location)).toBe('192.168.1.99')
  })

  it('follows a rename pushed while the link is up', async () => {
    const t = await loadSync([saved('hexapod.local', A)], { location: 'hexapod.local' })
    t.mdns.set(mdnsOf('hexapod'))
    t.reach('websocket', A)
    expect(get(t.robots)[0].address).toBe('hexapod.local')

    t.mdns.set(mdnsOf('kitchen'))

    expect(get(t.robots)[0].address).toBe('kitchen.local')
    expect(get(t.location)).toBe('kitchen.local')
  })

  it('a rename seen in the snapshot after reaching the robot by its last IP counts', async () => {
    const t = await loadSync([saved('esp32.local', A, { lastIp: IP })], {
      location: 'esp32.local'
    })
    t.fallbackHost.set(IP)

    t.reach('websocket', A)
    t.mdns.set(mdnsOf('hexapod'))

    expect(get(t.robots)[0].address).toBe('hexapod.local')
    expect(get(t.location)).toBe('hexapod.local')
    expect(get(t.fallbackHost)).toBe(IP)
  })

  it('records the last IP the robot reports, on the robot with its id', async () => {
    const t = await loadSync([saved('a.local', A), saved('b.local', B)], { location: 'a.local' })

    t.reach('websocket', B)
    t.wifi.set(wifiOf(IP_U32))

    expect(get(t.robots).map(r => r.lastIp)).toEqual([undefined, IP])
  })

  it("records no IP while the robot has none, or the status is a last link's", async () => {
    const t = await loadSync([saved('a.local', A)], { location: 'a.local' })

    t.reach('websocket', A)
    t.wifi.set(wifiOf(0))
    t.wifi.set({ ...wifiOf(IP_U32), stale: true })

    expect(get(t.robots)[0].lastIp).toBeUndefined()
  })

  it('learns the id of a saved robot reached by its address', async () => {
    const t = await loadSync([{ name: 'hexapod.local', address: 'hexapod.local' }], {
      location: 'hexapod.local'
    })

    t.reach('websocket', A)

    expect(get(t.robots)[0].robotId).toBe(A)
  })

  it('learns no id from a link that does not name the saved robot', async () => {
    const t = await loadSync([{ name: 'hexapod.local', address: 'hexapod.local' }], {
      location: 'hexapod.local'
    })

    t.reach('bluetooth', A)

    expect(get(t.robots)[0].robotId).toBeUndefined()
  })

  it('a robot over BLE with another id changes nothing', async () => {
    const t = await loadSync([saved('esp32.local', A, { lastIp: '192.168.1.7' })], {
      location: 'esp32.local'
    })
    const before = get(t.robots)
    t.mdns.set(mdnsOf('hexapod'))
    t.wifi.set(wifiOf(IP_U32))

    t.reach('bluetooth', B)

    expect(get(t.robots)).toEqual(before)
    expect(get(t.location)).toBe('esp32.local')
  })

  it('a robot over BLE with its id is followed like one over WiFi', async () => {
    const t = await loadSync([saved('esp32.local', A)], { location: 'esp32.local' })
    t.mdns.set(mdnsOf('hexapod'))
    t.wifi.set(wifiOf(IP_U32))

    t.reach('bluetooth', A)

    expect(get(t.robots)[0]).toMatchObject({ address: 'hexapod.local', lastIp: IP })
  })

  it('an IP that answers as another robot', async () => {
    const t = await loadSync([saved('hexapod.local', A, { lastIp: IP })], {
      location: 'hexapod.local'
    })
    t.fallbackHost.set(IP)
    t.mdns.set(mdnsOf('hexapod'))
    t.wifi.set(wifiOf(IP_U32))
    const before = get(t.robots)

    t.reach('websocket', B)

    expect(get(t.fallbackHost)).toBeNull()
    expect(t.disconnect).toHaveBeenCalledTimes(1)
    expect(get(t.notifications).map(n => [n.type, n.message])).toEqual([
      ['warning', expect.stringContaining(IP)]
    ])
    expect(get(t.robots)).toEqual([{ ...before[0], lastIp: undefined }])
    expect(get(t.location)).toBe('hexapod.local')
  })

  it('leaves the fallback of a WiFi link alone while Bluetooth reaches another robot', async () => {
    const t = await loadSync([saved('hexapod.local', A, { lastIp: IP })], {
      location: 'hexapod.local'
    })
    t.fallbackHost.set(IP)

    t.reach('bluetooth', B)

    expect(get(t.fallbackHost)).toBe(IP)
    expect(t.disconnect).not.toHaveBeenCalled()
    expect(get(t.notifications)).toEqual([])
  })

  it('keeps the fallback while the IP answers as the saved robot', async () => {
    const t = await loadSync([saved('hexapod.local', A, { lastIp: IP })], {
      location: 'hexapod.local'
    })
    t.fallbackHost.set(IP)

    t.reach('websocket', A)

    expect(get(t.fallbackHost)).toBe(IP)
    expect(t.disconnect).not.toHaveBeenCalled()
  })

  it('acts on no flags that were read before the link reached the IP', async () => {
    const t = await loadSync([saved('hexapod.local', A, { lastIp: IP })], {
      location: 'hexapod.local'
    })
    t.reach('websocket', A)
    t.transport.set(null)
    t.fallbackHost.set(IP)

    // The link reaches another robot at the IP: its snapshot arrives before its flags are read, and its name is not
    // the saved robot's new name.
    t.transport.set('websocket')
    t.mdns.set(mdnsOf('esp32'))
    expect(get(t.robots)[0].address).toBe('hexapod.local')
    expect(get(t.fallbackHost)).toBe(IP)

    t.features.set(featuresOf(B))

    expect(get(t.robots)[0].address).toBe('hexapod.local')
    expect(get(t.fallbackHost)).toBeNull()
    expect(t.disconnect).toHaveBeenCalledTimes(1)
  })

  it('does nothing from a snapshot a dropped link left stale', async () => {
    const t = await loadSync([saved('esp32.local', A)], { location: 'esp32.local' })
    t.mdns.set(mdnsOf('hexapod', true))

    t.reach('websocket', A)

    expect(get(t.robots)[0].address).toBe('esp32.local')
  })

  it('does nothing for firmware that reports no id', async () => {
    const t = await loadSync([saved('esp32.local', A)], { location: 'esp32.local' })
    t.mdns.set(mdnsOf('hexapod'))

    t.reach('websocket', '')

    expect(get(t.robots)[0].address).toBe('esp32.local')
  })

  it('tells the robot-hosted app of a rename once, with a link, and changes nothing else', async () => {
    const t = await loadSync([saved('esp32.local', A)], {
      location: '',
      pageHostname: 'esp32.local'
    })
    const before = get(t.robots)
    t.mdns.set(mdnsOf('hexapod'))
    t.wifi.set(wifiOf(IP_U32))

    t.reach('websocket', A)
    t.mdns.set(mdnsOf('hexapod'))

    expect(get(t.notifications)).toHaveLength(1)
    expect(get(t.notifications)[0].link).toEqual({
      href: 'http://hexapod.local',
      label: 'open http://hexapod.local'
    })
    expect(get(t.location)).toBe('')
    expect(get(t.robots)).toEqual(before)
  })

  it('writes no id into an entry typed into the address field while the link is up', async () => {
    const t = await loadSync([saved('a.local', A), { name: 'c.local', address: 'c.local' }], {
      location: 'a.local'
    })
    t.mdns.set(mdnsOf('a'))
    t.reach('websocket', A)

    // The field stays editable while the socket is up; its new value makes the flags read again over the same link.
    t.location.set('c.local')
    t.features.set(featuresOf(A))

    expect(get(t.robots).map(r => r.robotId)).toEqual([A, undefined])
  })

  it('follows no rename and records no IP once the address is not the one the link opened at', async () => {
    const t = await loadSync([saved('a.local', A)], { location: 'a.local' })
    t.reach('websocket', A)
    t.location.set('somewhere.local')

    t.mdns.set(mdnsOf('kitchen'))
    t.wifi.set(wifiOf(IP_U32))

    expect(get(t.robots)[0].address).toBe('a.local')
    expect(get(t.robots)[0].lastIp).toBeUndefined()
  })

  it('records the IP after the rename it followed', async () => {
    const t = await loadSync([saved('esp32.local', A)], { location: 'esp32.local' })
    t.mdns.set(mdnsOf('hexapod'))
    t.reach('websocket', A)

    t.wifi.set(wifiOf(IP_U32))

    expect(get(t.robots)[0]).toMatchObject({ address: 'hexapod.local', lastIp: IP })
  })

  it('takes no host name the firmware would refuse, for a saved robot or a notice', async () => {
    const hosted = await loadSync([], { location: '', pageHostname: 'esp32.local' })
    hosted.mdns.set(mdnsOf('x@evil.com'))
    hosted.reach('websocket', A)
    expect(get(hosted.notifications)).toEqual([])

    const t = await loadSync([saved('esp32.local', A)], { location: 'esp32.local' })
    t.mdns.set(mdnsOf('x@evil.com'))
    t.reach('websocket', A)
    expect(get(t.robots)[0].address).toBe('esp32.local')
    expect(get(t.location)).toBe('esp32.local')
  })

  it('does not tell the robot-hosted app of a name it is served from', async () => {
    const t = await loadSync([], { location: '', pageHostname: 'hexapod.local' })
    t.mdns.set(mdnsOf('hexapod'))

    t.reach('websocket', A)

    expect(get(t.notifications)).toEqual([])
  })

  it('stops following when stopped', async () => {
    const t = await loadSync([saved('esp32.local', A)], { location: 'esp32.local' })
    t.reach('websocket', A)
    t.stop()

    t.mdns.set(mdnsOf('hexapod'))

    expect(get(t.robots)[0].address).toBe('esp32.local')
  })
})
