import { api } from '$lib/api'
import { dataBroker } from '$lib/transport/databroker'
import { ask } from '$lib/transport/reply'
import { get, writable, type Readable } from 'svelte/store'
import { canReachRobot, location } from './location-store'

export interface Features {
  firmware_name: string
  firmware_version: string
  firmware_built_target: string
  firmware_elf_sha256: string
  camera: boolean
  imu: boolean
  mag: boolean
  bmp: boolean
  servo: boolean
  mdns: boolean
  analytics: boolean
  sleep: boolean
  ota: boolean
  download_firmware: boolean
  upload_firmware: boolean
}

const unknownFeatures: Features = {
  firmware_name: '',
  firmware_version: '',
  firmware_built_target: '',
  firmware_elf_sha256: '',
  camera: false,
  imu: false,
  mag: false,
  bmp: false,
  servo: false,
  mdns: false,
  analytics: false,
  sleep: false,
  ota: false,
  download_firmware: false,
  upload_firmware: false
}

// FeaturesDataResponse carries only the hardware-dependent flags. The rest are fixed for the
// single firmware build (the same constants /api/features reports in firmware/src/main.cpp).
const buildFeatures = {
  firmware_built_target: 'esp32-wroom-camera',
  bmp: false,
  analytics: true,
  sleep: true,
  ota: true,
  download_firmware: true,
  upload_firmware: true
}

const requestFeatures = async (signal: AbortSignal): Promise<Features> => {
  const { featuresDataResponse: f } = await ask(
    dataBroker,
    { featuresDataRequest: {} },
    'the feature list',
    { signal }
  )
  if (!f) throw new Error('features response missing')
  return {
    ...buildFeatures,
    firmware_name: f.firmwareName,
    firmware_version: f.firmwareVersion,
    firmware_elf_sha256: f.firmwareElfSha256,
    camera: f.camera,
    imu: f.imu,
    mag: f.mag,
    servo: f.servo,
    mdns: f.mdns
  }
}

const fetchFeaturesOverHttp = async (): Promise<Features> => {
  const result = await api.get<Features>('/api/features')
  if (result.isErr()) throw result.inner
  return { ...unknownFeatures, ...result.inner }
}

// A page that can reach the robot can still ask its webserver when the link fails. When that fails too, the link's
// failure is the one the page shows: it carries the robot's own reason.
const fetchFeatures = (signal: AbortSignal) =>
  requestFeatures(signal).catch(error => {
    if (signal.aborted || !canReachRobot(new URL(window.location.href), get(location))) throw error
    return fetchFeaturesOverHttp().catch(() => {
      throw error
    })
  })

/** Whether the flags are the robot's: pending until they are in, or failed with the sentence a page shows. */
export type FeatureFlagsState =
  { kind: 'pending' } | { kind: 'known' } | { kind: 'failed'; message: string }

const flags = writable<Features>(unknownFeatures)
const state = writable<FeatureFlagsState>({ kind: 'pending' })

/**
 * Until the flags are the robot's every one reads false, so a page that depends on one waits for this rather than
 * deciding on that default, and shows the failure when the read fails.
 */
export const featureFlagsState: Readable<FeatureFlagsState> = { subscribe: state.subscribe }

// The read under way, which a newer read or a forgotten robot aborts, so its answer can never land.
let reading: AbortController | null = null

/** Reads the flags afresh: after a restart, which may have brought other firmware, or to retry a failed read. */
export function refreshFeatureFlags(): Promise<Features> {
  reading?.abort()
  const read = new AbortController()
  reading = read
  if (get(state).kind === 'failed') state.set({ kind: 'pending' })
  return fetchFeatures(read.signal).then(
    features => {
      if (reading === read) {
        reading = null
        flags.set(features)
        state.set({ kind: 'known' })
      }
      return features
    },
    error => {
      if (reading === read) {
        reading = null
        state.set({
          kind: 'failed',
          message: error instanceof Error ? error.message : String(error)
        })
      }
      throw error
    }
  )
}

// A read that a newer one or a forgotten robot aborted failed nothing.
const readInBackground = () => {
  refreshFeatureFlags().catch(error => {
    if (!(error instanceof DOMException && error.name === 'AbortError'))
      console.warn('Feature flags could not be read:', error)
  })
}

/** The robot's flags. Reads them unless they are in or being read, so a page that mounts after a failure retries. */
export function useFeatureFlags(): Readable<Features> {
  if (get(state).kind !== 'known' && !reading) readInBackground()
  return { subscribe: flags.subscribe }
}

const forget = () => {
  reading?.abort()
  reading = null
  flags.set(unknownFeatures)
  state.set({ kind: 'pending' })
}

/**
 * Follows the robot the link reaches (`robot`, null while no link is up): its flags are forgotten when the link drops
 * or reaches another robot, so a page never shows one robot's flags for another, and read whenever a link comes up,
 * which also retries a failed read. A read still under way then began while no link was up, so the broker sends it to
 * the robot now reached, and it is left to answer. Returns the function that stops following.
 */
export const followFeatureFlags = (robot: Readable<string | null>) => {
  let current: string | null = null
  return robot.subscribe(next => {
    if (next === current) return
    if (current !== null) forget()
    current = next
    if (next !== null && !reading) readInBackground()
  })
}
