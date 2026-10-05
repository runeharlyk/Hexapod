import { describe, it, expect } from 'vitest'
import { findFirmwareAsset } from '../../src/lib/services/firmware-update'

const asset = (name: string) => ({ name, browser_download_url: `https://example.com/${name}` })

describe('findFirmwareAsset', () => {
  it('picks the app image over the factory image whatever the order', () => {
    const assets = [asset('esp32-wroom-camera.factory.bin'), asset('esp32-wroom-camera.bin')]
    expect(findFirmwareAsset(assets, 'esp32-wroom-camera')?.name).toBe('esp32-wroom-camera.bin')
  })

  it('finds nothing in a release that only carries the factory image', () => {
    expect(
      findFirmwareAsset([asset('esp32-wroom-camera.factory.bin')], 'esp32-wroom-camera')
    ).toBeUndefined()
  })

  it('does not match a target whose name only contains the built target', () => {
    expect(
      findFirmwareAsset([asset('esp32-wroom-camera-v2.bin')], 'esp32-wroom-camera')
    ).toBeUndefined()
  })
})
