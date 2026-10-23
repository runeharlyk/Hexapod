import { get, type Readable } from 'svelte/store'
import type { MDNSSettings, WifiStatus } from '$lib/platform_shared/api'
import { notifications } from '$lib/components/toasts/notifications'
import { isHostname, localName } from '$lib/mdns'
import type { Features, FeatureFlagsState } from '$lib/stores/featureFlags'
import type { TransportKind } from '$lib/stores/link'
import { location } from '$lib/stores/location-store'
import { fallbackHost } from '$lib/stores/robot-host'
import { followRename, forgetLastIp, recordLastIp, recordRobotId, robots } from '$lib/stores/robots'
import { ipToString } from '$lib/proto-api'
import type { Watched } from '$lib/transport/topics'

const NOTICE_MS = 20000
const LOCAL_NAME = /^([^.:]+)\.local$/i

export interface SavedRobotSyncDeps {
  transport: Readable<TransportKind | null>
  features: Readable<Features>
  featuresState: Readable<FeatureFlagsState>
  mdns: Readable<Watched<MDNSSettings>>
  wifi: Readable<Watched<WifiStatus>>
  disconnect: () => Promise<void>
  /** The host the app is served from: the robot's own name in the robot-hosted build. */
  pageHostname: string
}

/**
 * Keeps the saved robot the link reaches true to the robot: its last IP, its `<name>.local` address after a rename, and
 * the fallback to that IP, which is dropped when the IP answers as another robot. The robot is told by its `robot_id`,
 * since every robot ships with the same name. Nothing is done from a link whose feature flags are not yet the ones read
 * on it: they may still be the last link's robot's. Nor is anything written for an address that is not the one the link
 * opened at, since the address field stays editable while the link is up and the robot linked is not the one typed.
 * In the robot-hosted build, whose saved robots are not this page's to rewrite, a rename is only announced.
 * Returns the function that stops it.
 */
export const startSavedRobotSync = (deps: SavedRobotSyncDeps) => {
  let transport: TransportKind | null = null
  let flags = get(deps.features)
  let flagsAtLinkUp: Features | null = null
  let locationAtLinkUp = get(location)
  let noticedHost: string | null = null

  const flagsAreRead = () =>
    get(deps.featuresState).kind === 'known' && flags !== flagsAtLinkUp && flags.robot_id !== ''

  const noticeRename = (hostname: string) => {
    const name = LOCAL_NAME.exec(deps.pageHostname)?.[1]
    if (transport !== 'websocket' || !name || !isHostname(hostname)) return
    if (name.toLowerCase() === hostname.toLowerCase() || noticedHost === hostname) return
    noticedHost = hostname
    notifications.info(
      `This robot is now ${localName(hostname)}, but this page is still served from ${deps.pageHostname}:`,
      NOTICE_MS,
      { href: `http://${localName(hostname)}`, label: `open http://${localName(hostname)}` }
    )
  }

  const dropFallback = (robotId: string) => {
    const expected = get(robots).find(r => r.address === get(location))
    const ip = get(fallbackHost)
    if (
      transport !== 'websocket' ||
      ip === null ||
      !expected?.robotId ||
      expected.robotId === robotId
    )
      return false
    fallbackHost.set(null)
    forgetLastIp(expected.address)
    void deps.disconnect()
    notifications.warning(
      `${ip} is another robot than ${expected.name}, so ${expected.name} was not reached there`,
      NOTICE_MS
    )
    return true
  }

  const sync = () => {
    if (transport === null || !flagsAreRead() || get(location) !== locationAtLinkUp) return
    const { value: mdns, stale: mdnsStale } = get(deps.mdns)
    if (get(location) === '') {
      if (mdns && !mdnsStale) noticeRename(mdns.hostname)
      return
    }

    const robotId = flags.robot_id
    if (dropFallback(robotId)) return

    if (transport === 'websocket' && get(fallbackHost) === null)
      recordRobotId(get(location), robotId)

    const { value: wifi, stale: wifiStale } = get(deps.wifi)
    if (wifi && !wifiStale && wifi.localIp) recordLastIp(robotId, ipToString(wifi.localIp))

    if (!mdns || mdnsStale || !mdns.hostname) return
    const renamed = followRename(robotId, mdns.hostname)
    if (renamed && get(location) === renamed.from) {
      // The socket is still on the host it reached; only the address it reopens at changes.
      const reached = get(fallbackHost)
      location.set(renamed.to)
      fallbackHost.set(reached)
      locationAtLinkUp = renamed.to
    }
  }

  const stops = [
    deps.features.subscribe(next => {
      flags = next
      sync()
    }),
    deps.featuresState.subscribe(sync),
    deps.transport.subscribe(next => {
      if (next !== null && next !== transport) {
        flagsAtLinkUp = flags
        locationAtLinkUp = get(location)
      }
      transport = next
      sync()
    }),
    deps.wifi.subscribe(sync),
    deps.mdns.subscribe(sync)
  ]
  return () => stops.forEach(stop => stop())
}
