import type { DownloadOTA } from '$lib/types/models'
import { get, writable, type Readable } from 'svelte/store'
import { FeaturesDataResponse, TelemetryBatch } from '$lib/platform_shared/message'
import type { DataBroker } from '$lib/transport/databroker'
import { ask } from '$lib/transport/reply'
import { HexrecWriter, tickTiming, type TimedTick, type TickTiming } from '$lib/recording'

const telemetry_data = {
  rssi: {
    rssi: 0
  },
  download_ota: {
    status: 'none',
    progress: 0,
    error: ''
  }
}

function createTelemetry() {
  const { subscribe, update } = writable(telemetry_data)

  return {
    subscribe,
    setRSSI: (data: number) => {
      update(telemetry_data => ({
        ...telemetry_data,
        rssi: { rssi: data }
      }))
    },
    setDownloadOTA: (data: DownloadOTA) => {
      update(telemetry_data => ({
        ...telemetry_data,
        download_ota: { status: data.status, progress: data.progress, error: data.error }
      }))
    }
  }
}

export const telemetry = createTelemetry()

export interface FinishedRecording {
  blob: Blob
  filename: string
  timing: TickTiming
}

export interface RecorderState {
  recording: boolean
  starting: boolean
  bytes: number
  frames: number
  gaps: number
  finished?: FinishedRecording
  error?: string
}

const wallClockUs = () => Math.round((performance.timeOrigin + performance.now()) * 1000)

const stamp = (date: Date) => {
  const pad = (value: number) => String(value).padStart(2, '0')
  return (
    `${date.getFullYear()}${pad(date.getMonth() + 1)}${pad(date.getDate())}-` +
    `${pad(date.getHours())}${pad(date.getMinutes())}${pad(date.getSeconds())}`
  )
}

/**
 * Records the per-tick telemetry as simulation/record.py does: the robot's features in the header, every
 * telemetry batch as it arrived, and a gap wherever the link dropped and came back. The batches are subscribed
 * to only while recording, since the robot streams every control tick to a subscriber.
 */
export function createTelemetryRecorder({
  broker,
  linked,
  hostUs = wallClockUs
}: {
  broker: DataBroker
  linked: Readable<boolean>
  hostUs?: () => number
}) {
  const state = writable<RecorderState>({
    recording: false,
    starting: false,
    bytes: 0,
    frames: 0,
    gaps: 0
  })
  let writer: HexrecWriter | undefined
  let ticks: TimedTick[] = []
  let startedAt = new Date()
  let stopListening: (() => void)[] = []

  async function start() {
    if (writer || get(state).starting) return
    state.update(s => ({ ...s, starting: true, error: undefined }))
    let features: FeaturesDataResponse | undefined
    try {
      features = (await ask(broker, { featuresDataRequest: {} }, 'the feature list'))
        .featuresDataResponse
      if (!features) throw new Error('the robot sent no features')
    } catch (error) {
      state.update(s => ({
        ...s,
        starting: false,
        error: `Recording not started: ${error instanceof Error ? error.message : error}`
      }))
      return
    }
    const recording = new HexrecWriter(FeaturesDataResponse.toJSON(features) as object)
    writer = recording
    ticks = []
    startedAt = new Date()
    state.set({ recording: true, starting: false, bytes: recording.size, frames: 0, gaps: 0 })
    // record.py also counts 3 s without a batch as a dropped link; here only the link state marks a gap, and a
    // silent stretch shows in the report as missing batches.
    let wasLinked = true
    stopListening = [
      linked.subscribe(isLinked => {
        if (isLinked && !wasLinked) {
          recording.gap(hostUs())
          state.update(s => ({ ...s, bytes: recording.size, gaps: s.gaps + 1 }))
        }
        wasLinked = isLinked
      }),
      broker.on(TelemetryBatch, (batch, frame) => {
        recording.write(frame, hostUs())
        for (const { seq, periodUs, computeUs } of batch.ticks)
          ticks.push({ seq, periodUs, computeUs })
        state.update(s => ({ ...s, bytes: recording.size, frames: s.frames + 1 }))
      })
    ]
  }

  function stop() {
    if (!writer) return
    stopListening.forEach(stopOne => stopOne())
    stopListening = []
    const finished = {
      blob: writer.blob(),
      filename: `recording-${stamp(startedAt)}.hexrec`,
      timing: tickTiming(ticks)
    }
    writer = undefined
    ticks = []
    state.update(s => ({ ...s, recording: false, finished }))
  }

  function download() {
    const finished = get(state).finished
    if (!finished) return
    const url = URL.createObjectURL(finished.blob)
    const anchor = document.createElement('a')
    anchor.href = url
    anchor.download = finished.filename
    anchor.click()
    // Some browsers start reading the URL only after the click handler returns.
    setTimeout(() => URL.revokeObjectURL(url), 0)
  }

  return { subscribe: state.subscribe, start, stop, download }
}
