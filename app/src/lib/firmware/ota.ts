import type { CorrelationResponse } from '$lib/platform_shared/message'
import { ask, type RequestPayload } from '$lib/transport/reply'

// The robot's OtaChunk.data limit (platform_shared/message.options).
export const OTA_CHUNK_BYTES = 512

type Request = (payload: RequestPayload) => Promise<CorrelationResponse>

// Sends an app image over the protobuf link (Bluetooth or USB), for pages that cannot reach the
// robot's HTTP update endpoint. The robot validates the image, selects it and restarts once the last
// chunk is in; any refusal leaves it running the firmware it has.
export async function sendFirmware(
  image: Uint8Array,
  request: Request,
  onProgress?: (sent: number, total: number) => void
) {
  const link = { request }
  await ask(link, { otaStart: { size: image.length } }, 'starting the update')
  for (let offset = 0, index = 0; offset < image.length; offset += OTA_CHUNK_BYTES, index++) {
    const data = image.subarray(offset, offset + OTA_CHUNK_BYTES)
    await ask(link, { otaChunk: { index, data } }, `chunk ${index} of the update`)
    onProgress?.(offset + data.length, image.length)
  }
  await ask(link, { otaFinish: {} }, 'finishing the update')
}
