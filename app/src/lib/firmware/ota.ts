import type { CorrelationRequest, CorrelationResponse } from '$lib/platform_shared/message'

// The robot's OtaChunk.data limit (platform_shared/message.options).
export const OTA_CHUNK_BYTES = 512

type Request = (payload: Omit<CorrelationRequest, 'correlationId'>) => Promise<CorrelationResponse>

export class OtaError extends Error {}

const expectOk = (res: CorrelationResponse, step: string) => {
  if (res.statusCode !== 200)
    throw new OtaError(res.errorMessage || `${step} failed with status ${res.statusCode}`)
}

// Sends an app image over the protobuf link (Bluetooth or USB), for pages that cannot reach the
// robot's HTTP update endpoint. The robot validates the image, selects it and restarts once the last
// chunk is in; any refusal leaves it running the firmware it has.
export async function sendFirmware(
  image: Uint8Array,
  request: Request,
  onProgress?: (sent: number, total: number) => void
) {
  expectOk(await request({ otaStart: { size: image.length } }), 'Starting the update')
  for (let offset = 0, index = 0; offset < image.length; offset += OTA_CHUNK_BYTES, index++) {
    const data = image.subarray(offset, offset + OTA_CHUNK_BYTES)
    expectOk(await request({ otaChunk: { index, data } }), `Chunk ${index}`)
    onProgress?.(offset + data.length, image.length)
  }
  expectOk(await request({ otaFinish: {} }), 'Finishing the update')
}
