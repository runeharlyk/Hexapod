import { Message, type TickSample } from '$lib/platform_shared/message'

/**
 * The .hexrec format of simulation/src/sim/recording.py, so its loader and report_recording.py read what the app
 * records: b"HEXAREC1", uint32 length and the JSON of the robot's features, then records of uint64 host receive
 * time (us), uint32 length and one encoded socket_message.Message as it arrived; a record of length 0 marks a gap
 * where the link dropped. All integers are little-endian.
 */
const MAGIC = new TextEncoder().encode('HEXAREC1')
const LENGTH_BYTES = 4
const RECORD_HEADER_BYTES = 12

// Parts gather into a Blob once they reach this size, which the browser may keep out of the page's heap.
const CHUNK_BYTES = 1 << 20

/**
 * The text Python's json.dumps gives for JSON-shaped data: ", " and ": " separators, and non-ASCII escaped
 * (the only characters JSON.stringify leaves outside printable ASCII and DEL).
 */
const pythonJson = (value: unknown): string => {
  if (Array.isArray(value)) return `[${value.map(pythonJson).join(', ')}]`
  if (value !== null && typeof value === 'object') {
    const members = Object.entries(value).map(
      ([key, item]) => `${pythonJson(key)}: ${pythonJson(item)}`
    )
    return `{${members.join(', ')}}`
  }
  return JSON.stringify(value).replace(
    /[^\x20-\x7f]/g,
    char => `\\u${char.charCodeAt(0).toString(16).padStart(4, '0')}`
  )
}

const recordHeader = (hostRxUs: number, length: number) => {
  const header = new Uint8Array(RECORD_HEADER_BYTES)
  const view = new DataView(header.buffer)
  view.setBigUint64(0, BigInt(hostRxUs), true)
  view.setUint32(8, length, true)
  return header
}

/** Appends frames as they arrive; `size` counts every byte written, the header included. */
export class HexrecWriter {
  size = 0
  private chunks: Blob[] = []
  private parts: Uint8Array[] = []
  private partBytes = 0

  constructor(features: object) {
    const json = new TextEncoder().encode(pythonJson(features))
    const preamble = new Uint8Array(MAGIC.length + LENGTH_BYTES)
    preamble.set(MAGIC)
    new DataView(preamble.buffer).setUint32(MAGIC.length, json.length, true)
    this.push(preamble)
    this.push(json)
  }

  write(frame: Uint8Array, hostRxUs: number) {
    this.push(recordHeader(hostRxUs, frame.length))
    // A transport may reuse its receive buffer once the listeners return.
    this.push(frame.slice())
  }

  gap(hostRxUs: number) {
    this.push(recordHeader(hostRxUs, 0))
  }

  blob(): Blob {
    this.flush()
    return new Blob(this.chunks, { type: 'application/octet-stream' })
  }

  private push(part: Uint8Array) {
    this.parts.push(part)
    this.partBytes += part.length
    this.size += part.length
    if (this.partBytes >= CHUNK_BYTES) this.flush()
  }

  private flush() {
    if (this.parts.length === 0) return
    this.chunks.push(new Blob(this.parts as BlobPart[]))
    this.parts = []
    this.partBytes = 0
  }
}

export interface Recording {
  features: unknown
  ticks: TickSample[]
  linkGaps: number
}

export function readRecording(data: Uint8Array): Recording {
  if (data.length < MAGIC.length + LENGTH_BYTES || MAGIC.some((byte, i) => data[i] !== byte)) {
    throw new Error('not a hexapod telemetry recording')
  }
  const view = new DataView(data.buffer, data.byteOffset, data.byteLength)
  const featuresStart = MAGIC.length + LENGTH_BYTES
  const featuresEnd = featuresStart + view.getUint32(MAGIC.length, true)
  const features = JSON.parse(new TextDecoder().decode(data.subarray(featuresStart, featuresEnd)))
  const ticks: TickSample[] = []
  let linkGaps = 0
  let offset = featuresEnd
  while (offset + RECORD_HEADER_BYTES <= data.length) {
    const length = view.getUint32(offset + 8, true)
    const start = offset + RECORD_HEADER_BYTES
    if (start + length > data.length) break // the recorder stopped in the middle of this record
    offset = start + length
    if (length === 0) {
      linkGaps++
      continue
    }
    const batch = Message.decode(data.subarray(start, offset)).telemetryBatch
    if (batch) ticks.push(...batch.ticks)
  }
  return { features, ticks, linkGaps }
}

export const CONTROL_PERIOD_US = 5000
// A tick this much later than its slot is an overrun the firmware restarts its schedule after.
const OVERRUN_US = 2 * CONTROL_PERIOD_US

export type TimedTick = Pick<TickSample, 'seq' | 'periodUs' | 'computeUs'>

export interface TickTiming {
  ticks: number
  periodMeanUs: number
  periodP50Us: number
  periodP99Us: number
  overruns: number
  computeP99Us: number
}

/** NumPy's default percentile: linear interpolation between the closest ranks; NaN without values. */
const percentile = (values: number[], p: number) => {
  if (values.length === 0) return NaN
  const sorted = [...values].sort((a, b) => a - b)
  const rank = (p / 100) * (sorted.length - 1)
  const below = Math.floor(rank)
  const above = Math.min(below + 1, sorted.length - 1)
  return sorted[below] + (rank - below) * (sorted[above] - sorted[below])
}

/** simulation/src/sim/telemetry_stats.py's tick_timing, with the median beside the 99th percentile. */
export function tickTiming(ticks: TimedTick[]): TickTiming {
  // A tick after a hole in seq has its period measured across the missing ticks; leave it out.
  const periods = ticks
    .filter((tick, i) => i > 0 && tick.seq - ticks[i - 1].seq === 1)
    .map(tick => tick.periodUs)
  return {
    ticks: ticks.length,
    periodMeanUs:
      periods.length ? periods.reduce((sum, period) => sum + period, 0) / periods.length : NaN,
    periodP50Us: percentile(periods, 50),
    periodP99Us: percentile(periods, 99),
    overruns: periods.filter(period => period > OVERRUN_US).length,
    computeP99Us: percentile(
      ticks.map(tick => tick.computeUs),
      99
    )
  }
}
