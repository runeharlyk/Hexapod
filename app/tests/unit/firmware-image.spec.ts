import { describe, it, expect } from 'vitest'
import { IMAGE_HEAD_BYTES, inspectImage } from '../../src/lib/firmware/image'

const SHA = Array.from({ length: 32 }, (_, i) => i)

// Laid out as the release build's firmware.bin: image header, segment header, esp_app_desc_t.
const appImage = (chipId = 9) => {
  const bytes = new Uint8Array(IMAGE_HEAD_BYTES + 64)
  const view = new DataView(bytes.buffer)
  bytes[0] = 0xe9
  view.setUint16(12, chipId, true)
  view.setUint32(32, 0xabcd5432, true)
  bytes.set(new TextEncoder().encode('v1.2.3'), 48)
  bytes.set(new TextEncoder().encode('Hexapod'), 80)
  bytes.set(SHA, 176)
  return bytes
}

// firmware.factory.bin opens with the bootloader: also 0xE9, but its descriptor magic byte is 0x50.
const factoryImage = () => {
  const bytes = appImage()
  new DataView(bytes.buffer).setUint32(32, 0x50, true)
  return bytes
}

describe('inspectImage', () => {
  it('reads the version, project and ELF hash of an app image', () => {
    expect(inspectImage(appImage())).toEqual({
      ok: true,
      image: {
        version: 'v1.2.3',
        projectName: 'Hexapod',
        elfSha256: SHA.map(b => b.toString(16).padStart(2, '0')).join('')
      }
    })
  })

  it('refuses a factory image', () => {
    const check = inspectImage(factoryImage())
    expect(check.ok).toBe(false)
    if (!check.ok) expect(check.error).toMatch(/factory/)
  })

  it('refuses an image for another chip', () => {
    const check = inspectImage(appImage(0))
    expect(check.ok).toBe(false)
    if (!check.ok) expect(check.error).toMatch(/chip/)
  })

  it('refuses a file that is no image, and one too short to tell', () => {
    expect(inspectImage(new TextEncoder().encode('{"not":"firmware"}')).ok).toBe(false)
    expect(inspectImage(appImage().subarray(0, IMAGE_HEAD_BYTES - 1)).ok).toBe(false)
  })
})
