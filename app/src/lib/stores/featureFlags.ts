import { api } from '$lib/api'
import { dataBroker } from '$lib/transport/databroker'
import { writable, type Writable } from 'svelte/store'

export interface Features {
  firmware_name: string
  firmware_version: string
  firmware_built_target: string
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

const requestFeatures = async (): Promise<Features> => {
  const { featuresDataResponse: f } = await dataBroker.request({ featuresDataRequest: {} })
  if (!f) throw new Error('features response missing')
  return {
    ...buildFeatures,
    firmware_name: f.firmwareName,
    firmware_version: f.firmwareVersion,
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

let featureFlagsStore: Writable<Features> | undefined
let loading = false

export function useFeatureFlags() {
  featureFlagsStore ??= writable<Features>(unknownFeatures)
  const store = featureFlagsStore

  if (!loading) {
    loading = true
    requestFeatures()
      .catch(error => {
        // The robot-hosted http page can still ask the webserver when the link is not up yet.
        if (window.location.protocol !== 'http:') throw error
        return fetchFeaturesOverHttp()
      })
      .then(features => store.set(features))
      .catch(error => {
        // Leave the flags unknown and let the next page that needs them try again.
        loading = false
        console.warn('Feature flags could not be fetched:', error)
      })
  }

  return store
}
