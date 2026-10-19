// fixtures/recording.hexrec and recording.frames.json are written by simulation/scripts/make_hexrec_fixture.py
// (from simulation/: uv run python scripts/make_hexrec_fixture.py) with the writer record.py uses. The sidecar holds
// the features and every record's host receive time and frame, so these tests feed the writer and the recorder the
// exact frames protobuf-python encoded and check the framing alone. Its ticks are those `tick` builds below, in
// batch 7 [0, 1, 2], batch 8 [3, 4], a gap, and batch 10 [6, 7, 8]; the script prints the Python figures asserted
// here: TickTiming(ticks=8, period_mean_us=6166.666666666667, period_p99_us=11650.05, overruns=1,
// compute_p99_us=1279.3) and a median contiguous period of 5000.0.
import { readFileSync } from 'node:fs'
import { resolve } from 'node:path'
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { derived, get, writable, type Writable } from 'svelte/store'
import { DataBroker } from '$lib/transport/databroker'
import type { ITransport, LinkStatus } from '$lib/interfaces/transport.interface'
import {
  CorrelationResponse,
  FeaturesDataResponse,
  IMUData,
  Message,
  TickSample
} from '$lib/platform_shared/message'
import { HexrecWriter, readRecording, tickTiming } from '$lib/recording'
import { createTelemetryRecorder } from '$lib/stores/telemetry'

const fixture = new Uint8Array(readFileSync(resolve(__dirname, 'fixtures/recording.hexrec')))
const sidecar = JSON.parse(
  readFileSync(resolve(__dirname, 'fixtures/recording.frames.json'), 'utf8')
) as { features: object; records: { host_rx_us: number; frame: string }[] }
const records = sidecar.records.map(({ host_rx_us, frame }) => ({
  hostRxUs: host_rx_us,
  frame: Uint8Array.from(frame.match(/../g) ?? [], byte => parseInt(byte, 16))
}))
const [batch7, batch8, batch10] = records.filter(r => r.frame.length > 0).map(r => r.frame)

const FEATURES = FeaturesDataResponse.create({
  firmwareName: 'hexapod',
  firmwareVersion: '0.9.0',
  imu: true,
  servo: true,
  firmwareElfSha256: 'ab'.repeat(32)
})
const PERIODS = [5000, 5001, 4999, 5000, 12000, 5003, 4998, 5000, 5000]

const tick = (seq: number) =>
  TickSample.create({
    seq,
    tUs: 1_000_000 + seq * 5000,
    periodUs: PERIODS[seq],
    computeUs: 1200 + seq * 10,
    imuFresh: seq % 2 === 0,
    rpy: [0.5, -0.25, 1.0],
    gyro: [0.0, 0.125, -0.5],
    gravity: [0.0, 0.0, 1.0],
    angles: Array.from({ length: 18 }, (_, i) => i),
    command: [0.0, 0.5, 0.0, 0.0, 0.25, 0.0, 0.0, 0.75, -0.5],
    commandAgeMs: seq * 3,
    mode: 4,
    gait: 1,
    mag: [21.5, -4.25, 40.0],
    magFresh: seq % 3 === 0
  })

// jsdom's Blob has no arrayBuffer(); its FileReader reads one.
const bytesOf = (blob: Blob) =>
  new Promise<Uint8Array>((done, fail) => {
    const reader = new FileReader()
    reader.onload = () => done(new Uint8Array(reader.result as ArrayBuffer))
    reader.onerror = () => fail(reader.error)
    reader.readAsArrayBuffer(blob)
  })

describe('the .hexrec format', () => {
  it('writes the bytes record.py writes for the same features, frames and gap', async () => {
    const writer = new HexrecWriter(sidecar.features)
    for (const { hostRxUs, frame } of records) {
      if (frame.length) writer.write(frame, hostRxUs)
      else writer.gap(hostRxUs)
    }

    expect(writer.size).toBe(fixture.length)
    expect(await bytesOf(writer.blob())).toEqual(fixture)
  })

  it('escapes non-ASCII in the features as json.dumps does', async () => {
    const header = await bytesOf(
      new HexrecWriter({ firmwareName: `hex${String.fromCharCode(0xf8)}pod` }).blob()
    )
    expect(new TextDecoder().decode(header.subarray(12))).toBe('{"firmwareName": "hex\\u00f8pod"}')
  })

  it('reads back the features, every tick and the gap of a record.py file', () => {
    const recording = readRecording(fixture)

    expect(recording.features).toEqual(sidecar.features)
    expect(recording.ticks.map(t => t.seq)).toEqual([0, 1, 2, 3, 4, 6, 7, 8])
    expect(recording.ticks[5]).toEqual(tick(6))
    expect(FeaturesDataResponse.toJSON(FEATURES)).toEqual(sidecar.features)
    expect(recording.linkGaps).toBe(1)
  })

  it('stops at a record the recorder was cut off in', () => {
    const recording = readRecording(fixture.subarray(0, fixture.length - 1))
    expect(recording.ticks.map(t => t.seq)).toEqual([0, 1, 2, 3, 4])
  })

  it('rejects a file that is not a recording', () => {
    expect(() => readRecording(new TextEncoder().encode('LEIKAREC'))).toThrow(/not a hexapod/)
  })

  it('times the ticks as telemetry_stats.tick_timing does, stepping over holes in seq', () => {
    const timing = tickTiming(readRecording(fixture).ticks)

    expect(timing.ticks).toBe(8)
    expect(timing.periodMeanUs).toBeCloseTo(6166.666666666667, 9)
    expect(timing.periodP50Us).toBeCloseTo(5000, 9)
    expect(timing.periodP99Us).toBeCloseTo(11650.05, 9)
    expect(timing.overruns).toBe(1)
    expect(timing.computeP99Us).toBeCloseTo(1279.3, 9)
  })

  it('has no period statistics without two contiguous ticks', () => {
    const timing = tickTiming([tick(0)])
    expect(timing.ticks).toBe(1)
    expect(timing.periodP50Us).toBeNaN()
    expect(timing.overruns).toBe(0)
  })
})

const TELEMETRY_BATCH_TAG = 290

const createLink = (featuresStatus = 200) => {
  const status = writable<LinkStatus>('disconnected')
  const sent: Message[] = []
  const listeners: ((bytes: Uint8Array) => void)[] = []
  const receive = (bytes: Uint8Array) => listeners.forEach(listener => listener(bytes))
  const answerFeatures = (correlationId: number) =>
    receive(
      Message.encode(
        Message.create({
          correlationResponse: CorrelationResponse.create({
            correlationId,
            statusCode: featuresStatus,
            featuresDataResponse: FEATURES
          })
        })
      ).finish()
    )
  const transport: ITransport = {
    status,
    connected: derived(status, s => s === 'connected'),
    connect: async () => status.set('connected'),
    disconnect: async () => status.set('disconnected'),
    send: bytes => {
      const message = Message.decode(bytes)
      sent.push(message)
      const request = message.correlationRequest
      if (request?.featuresDataRequest) queueMicrotask(() => answerFeatures(request.correlationId))
    },
    trySend: bytes => {
      transport.send(bytes)
      return true
    },
    onData: listener => listeners.push(listener),
    onConnect: () => {},
    onDisconnect: () => {}
  }
  return { transport, sent, receive }
}

describe('the telemetry recorder', () => {
  let link: ReturnType<typeof createLink>
  let linked: Writable<boolean>
  let recorder: ReturnType<typeof createTelemetryRecorder>

  beforeEach(async () => {
    link = createLink()
    const broker = new DataBroker()
    broker.addTransport(link.transport)
    await link.transport.connect()
    linked = writable(true)
    const receiveTimes = records.map(r => r.hostRxUs)
    recorder = createTelemetryRecorder({ broker, linked, hostUs: () => receiveTimes.shift()! })
  })

  afterEach(() => vi.restoreAllMocks())

  const subscriptions = () =>
    link.sent.flatMap(m =>
      m.subNotif ? [`sub ${m.subNotif.tag}`]
      : m.unsubNotif ? [`unsub ${m.unsubNotif.tag}`]
      : []
    )

  it('does not start, and says why, when the robot refuses its features', async () => {
    const refusing = createLink(500)
    const broker = new DataBroker()
    broker.addTransport(refusing.transport)
    await refusing.transport.connect()
    const refused = createTelemetryRecorder({ broker, linked, hostUs: () => 0 })

    await refused.start()

    expect(get(refused)).toMatchObject({
      recording: false,
      starting: false,
      error: 'Recording not started: the robot refused the feature list'
    })
  })

  it('subscribes to the telemetry batches only while recording', async () => {
    expect(subscriptions()).toEqual([])
    await recorder.start()
    expect(get(recorder).recording).toBe(true)
    expect(subscriptions()).toEqual([`sub ${TELEMETRY_BATCH_TAG}`])

    recorder.stop()

    expect(subscriptions()).toEqual([`sub ${TELEMETRY_BATCH_TAG}`, `unsub ${TELEMETRY_BATCH_TAG}`])
    expect(get(recorder).recording).toBe(false)
  })

  it('writes what record.py writes: the frames as they arrived and a gap where the link came back', async () => {
    await recorder.start()
    link.receive(batch7)
    link.receive(batch8)
    linked.set(false)
    linked.set(true)
    link.receive(batch10)
    expect(get(recorder)).toMatchObject({ recording: true, frames: 3, gaps: 1 })

    recorder.stop()

    const finished = get(recorder).finished!
    expect(await bytesOf(finished.blob)).toEqual(fixture)
    expect(finished.filename).toMatch(/^recording-\d{8}-\d{6}\.hexrec$/)
  })

  it('summarises the control period and overruns on stop', async () => {
    await recorder.start()
    link.receive(batch7)
    link.receive(batch8)

    recorder.stop()

    const timing = get(recorder).finished!.timing
    expect(timing.ticks).toBe(5)
    expect(timing.periodP50Us).toBeCloseTo(5000.5, 9)
    expect(timing.overruns).toBe(1)
  })

  it('keeps other messages and frames after stop out of the file', async () => {
    await recorder.start()
    link.receive(Message.encode(Message.create({ imu: IMUData.create({ x: 1 }) })).finish())
    recorder.stop()
    link.receive(batch7)

    const finished = get(recorder).finished!
    expect(readRecording(await bytesOf(finished.blob)).ticks).toEqual([])
    expect(finished.timing.ticks).toBe(0)
  })

  it('downloads the finished recording under its file name', async () => {
    await recorder.start()
    link.receive(batch7)
    recorder.stop()
    const offered: Blob[] = []
    URL.createObjectURL = vi.fn((blob: Blob) => {
      offered.push(blob)
      return 'blob:recording'
    })
    URL.revokeObjectURL = vi.fn()
    const clicked: HTMLAnchorElement[] = []
    vi.spyOn(HTMLAnchorElement.prototype, 'click').mockImplementation(function (
      this: HTMLAnchorElement
    ) {
      clicked.push(this)
    })

    recorder.download()
    await new Promise(resolveSoon => setTimeout(resolveSoon, 0))

    const finished = get(recorder).finished!
    expect(clicked).toHaveLength(1)
    expect(clicked[0].download).toBe(finished.filename)
    expect(clicked[0].href).toBe('blob:recording')
    expect(offered).toEqual([finished.blob])
    expect(URL.revokeObjectURL).toHaveBeenCalledWith('blob:recording')
  })
})
