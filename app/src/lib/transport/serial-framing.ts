// Protobuf messages on the robot's native USB port. A frame is 0x00, COBS(version, message, CRC-16),
// 0x00: the delimiters let a reader that starts mid-stream, or loses bytes to a short write, find the
// next frame, and the CRC drops a damaged one. Bytes outside a valid frame (the ROM's boot banner
// after a reset) are dropped. firmware/include/communication/serial_frame.h implements the same, and
// platform_shared/serial_frame_vectors.json pins both.

export const SERIAL_FRAME_VERSION = 1
export const SERIAL_MAX_MESSAGE = 4096

const DELIMITER = 0
const MAX_BODY = 1 + SERIAL_MAX_MESSAGE + 2
// A COBS code byte for every 254 body bytes and one more, as serial_frame.h counts them.
const MAX_ENCODED = MAX_BODY + Math.floor(MAX_BODY / 254) + 1

// CRC-16/CCITT-FALSE: polynomial 0x1021, initial 0xFFFF, not reflected.
const crc16 = (data: Uint8Array) => {
  let crc = 0xffff
  for (const byte of data) {
    crc ^= byte << 8
    for (let bit = 0; bit < 8; bit++) {
      crc = crc & 0x8000 ? ((crc << 1) ^ 0x1021) & 0xffff : (crc << 1) & 0xffff
    }
  }
  return crc
}

// The bytes to write for one message, delimiters included.
export function encodeSerialFrame(message: Uint8Array): Uint8Array {
  if (message.length > SERIAL_MAX_MESSAGE) {
    throw new RangeError(
      `A serial frame carries at most ${SERIAL_MAX_MESSAGE} bytes, not ${message.length}`
    )
  }
  const body = new Uint8Array(message.length + 3)
  body[0] = SERIAL_FRAME_VERSION
  body.set(message, 1)
  const crc = crc16(body.subarray(0, body.length - 2))
  body[body.length - 2] = crc >> 8
  body[body.length - 1] = crc & 0xff

  const out = [DELIMITER]
  let codeAt = out.length
  out.push(1)
  for (const byte of body) {
    if (byte !== 0) {
      out.push(byte)
      out[codeAt]++
    }
    if (byte === 0 || out[codeAt] === 0xff) {
      codeAt = out.length
      out.push(1)
    }
  }
  out.push(DELIMITER)
  return Uint8Array.from(out)
}

const cobsDecode = (segment: number[]): Uint8Array | undefined => {
  const body: number[] = []
  for (let i = 0; i < segment.length; ) {
    const code = segment[i++]
    if (i + code - 1 > segment.length) return
    for (let end = i + code - 1; i < end; i++) body.push(segment[i])
    if (code !== 0xff && i < segment.length) body.push(0)
  }
  return Uint8Array.from(body)
}

const unframe = (segment: number[]): Uint8Array | undefined => {
  const body = cobsDecode(segment)
  if (!body || body.length < 3 || body[0] !== SERIAL_FRAME_VERSION) return
  if (body.length - 3 > SERIAL_MAX_MESSAGE) return
  const crc = (body[body.length - 2] << 8) | body[body.length - 1]
  if (crc16(body.subarray(0, body.length - 2)) !== crc) return
  return body.slice(1, body.length - 2)
}

// Splits a byte stream, fed in pieces of any size, into messages. A segment that is not a valid
// frame is dropped, and the 0x00 that ended it starts the next frame.
export class SerialFrameDecoder {
  private inFrame = false
  private segment: number[] = []

  constructor(private readonly onFrame: (message: Uint8Array) => void) {}

  feed(chunk: Uint8Array) {
    for (const byte of chunk) this.feedByte(byte)
  }

  reset() {
    this.inFrame = false
    this.segment = []
  }

  private feedByte(byte: number) {
    if (!this.inFrame) {
      if (byte === DELIMITER) {
        this.inFrame = true
        this.segment = []
      }
      return
    }
    if (byte !== DELIMITER) {
      this.segment.push(byte)
      if (this.segment.length > MAX_ENCODED) this.reset()
      return
    }
    if (this.segment.length === 0) return
    const message = unframe(this.segment)
    if (message) {
      this.onFrame(message)
      this.inFrame = false
    }
    this.segment = []
  }
}
