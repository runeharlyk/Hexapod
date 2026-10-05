// Reads the head of an ESP-IDF application image, the layout firmware/include/ota_image.h checks on
// the robot: the image header (magic 0xE9, chip id at byte 12), then esp_app_desc_t at byte 32.

const IMAGE_MAGIC = 0xe9
const APP_DESC_OFFSET = 32
const APP_DESC_MAGIC = 0xabcd5432
const ESP32_S3_CHIP_ID = 9

// Offsets inside esp_app_desc_t (esp_app_format/include/esp_app_desc.h).
const VERSION = APP_DESC_OFFSET + 16
const PROJECT_NAME = APP_DESC_OFFSET + 48
const ELF_SHA256 = APP_DESC_OFFSET + 144

export const IMAGE_HEAD_BYTES = ELF_SHA256 + 32

export type AppImage = { version: string; projectName: string; elfSha256: string }

export type ImageCheck = { ok: true; image: AppImage } | { ok: false; error: string }

const cString = (bytes: Uint8Array, offset: number, length: number) => {
  const field = bytes.subarray(offset, offset + length)
  const end = field.indexOf(0)
  return new TextDecoder().decode(end < 0 ? field : field.subarray(0, end))
}

const hex = (bytes: Uint8Array) => Array.from(bytes, b => b.toString(16).padStart(2, '0')).join('')

export const inspectImage = (head: Uint8Array): ImageCheck => {
  if (head.length < IMAGE_HEAD_BYTES || head[0] !== IMAGE_MAGIC)
    return { ok: false, error: 'This is not an ESP32 firmware image.' }
  const view = new DataView(head.buffer, head.byteOffset, head.byteLength)
  if (view.getUint32(APP_DESC_OFFSET, true) !== APP_DESC_MAGIC)
    return {
      ok: false,
      error: 'This is a factory image (bootloader and partitions); it goes through the web flasher.'
    }
  if (view.getUint16(12, true) !== ESP32_S3_CHIP_ID)
    return { ok: false, error: 'This image is built for a different chip than the ESP32-S3.' }
  return {
    ok: true,
    image: {
      version: cString(head, VERSION, 32),
      projectName: cString(head, PROJECT_NAME, 32),
      elfSha256: hex(head.subarray(ELF_SHA256, ELF_SHA256 + 32))
    }
  }
}
