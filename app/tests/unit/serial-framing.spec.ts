import { describe, it, expect } from 'vitest'
import { readFileSync } from 'node:fs'
import { resolve } from 'node:path'
import {
  SERIAL_MAX_MESSAGE,
  SerialFrameDecoder,
  encodeSerialFrame
} from '../../src/lib/transport/serial-framing'

type Vector = { name: string; stream: string; frames: string[] }

// The same file the firmware's host test reads (firmware/test/test_serial_frame).
const vectors: Vector[] = JSON.parse(
  readFileSync(resolve(__dirname, '../../../platform_shared/serial_frame_vectors.json'), 'utf8')
).vectors

const fromHex = (hex: string) => Uint8Array.from(hex.match(/../g) ?? [], b => parseInt(b, 16))
const toHex = (bytes: Uint8Array) =>
  Array.from(bytes, b => b.toString(16).padStart(2, '0')).join('')

const decode = (stream: Uint8Array, chunk: number) => {
  const frames: string[] = []
  const decoder = new SerialFrameDecoder(message => frames.push(toHex(message)))
  for (let i = 0; i < stream.length; i += chunk) decoder.feed(stream.subarray(i, i + chunk))
  return frames
}

describe('serial framing', () => {
  it('reads every shared vector', () => {
    expect(vectors.length).toBe(7)
  })

  for (const v of vectors) {
    it(`decodes "${v.name}" alike in any chunking`, () => {
      const stream = fromHex(v.stream)
      for (const chunk of [stream.length, 1, 7]) expect(decode(stream, chunk)).toEqual(v.frames)
    })
  }

  it('encodes each single-frame vector to the bytes the firmware would write', () => {
    const single = vectors.filter(v => v.frames.length === 1)
    expect(single.length).toBeGreaterThan(3)
    for (const v of single)
      expect(v.stream).toContain(toHex(encodeSerialFrame(fromHex(v.frames[0]))))
  })

  it('finds the frames after the ROM banner a reset prints', () => {
    const a = Uint8Array.of(8, 1, 0, 0, 7)
    const b = Uint8Array.of(0x10, 0x20)
    const first = encodeSerialFrame(a)
    const banner = new TextEncoder().encode('ESP-ROM:esp32s3-20210327\r\n')
    const stream = new Uint8Array([
      ...first.subarray(first.length / 2),
      ...banner,
      ...encodeSerialFrame(b),
      ...first
    ])
    expect(decode(stream, 3)).toEqual([toHex(b), toHex(a)])
  })

  it('refuses a message over the limit', () => {
    expect(() => encodeSerialFrame(new Uint8Array(SERIAL_MAX_MESSAGE + 1))).toThrow(RangeError)
  })
})
