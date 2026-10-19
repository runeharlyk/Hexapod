import type { CoreDumpReport } from '$lib/platform_shared/message'
import type { FileChunk } from '$lib/platform_shared/api'

/**
 * The crash the robot kept (CoreDumpGet, CoreDumpRead and CoreDumpErase in message.proto). Every transport carries
 * these, so the download works over Bluetooth and USB as it does over WiFi; GET /api/coredump serves the same bytes
 * to curl.
 */

export type CoreDumpReply = {
  statusCode: number
  errorMessage: string
  coreDumpReport?: CoreDumpReport
  fileChunk?: FileChunk
}

/** What the last-crash card needs of the data broker (transport/databroker.ts). */
export interface CoreDumpLink {
  request(
    payload:
      | { coreDumpGet: object }
      | { coreDumpRead: { offset: number; length: number } }
      | { coreDumpErase: object }
  ): Promise<CoreDumpReply>
}

/** The most a CoreDumpRead returns, the FileChunk's room. */
export const CORE_DUMP_CHUNK = 512

export type LastCrash =
  | { kind: 'none' }
  | { kind: 'unreadable'; size: number }
  | {
      kind: 'crash'
      task: string
      pc: string
      backtrace: string
      panicReason: string
      elf: string
      /**
       * Whose ELF decodes the dump: the running build's, another build's whose hash begins `elf`, or, when the dump
       * names none, the build that was running at the crash.
       */
      build: 'running' | 'other' | 'unnamed'
    }

const hex = (address: number) => `0x${address.toString(16).padStart(8, '0')}`

/** What the card shows of a report: nothing stored, a dump the robot could not read, or the crash it holds. */
export const lastCrash = (report: CoreDumpReport): LastCrash => {
  if (!report.stored) return { kind: 'none' }
  if (!report.readable) return { kind: 'unreadable', size: report.size }
  const addresses = report.backtrace.map(hex).join(' ')
  return {
    kind: 'crash',
    task: report.task,
    pc: hex(report.pc),
    // As the IDF's panic handler prints a backtrace whose unwinding stopped.
    backtrace: report.backtraceCorrupted ? `${addresses} |<-CORRUPTED` : addresses,
    panicReason: report.panicReason,
    elf: report.appElfSha256,
    build:
      report.elfMatchesRunning ? 'running'
      : report.appElfSha256 ? 'other'
      : 'unnamed'
  }
}

const refusal = (reply: CoreDumpReply) =>
  reply.errorMessage || `the robot answered with status ${reply.statusCode}`

export const readCoreDump = async (link: CoreDumpLink): Promise<CoreDumpReport> => {
  const reply = await link.request({ coreDumpGet: {} })
  // The request has no fields to get wrong: a bare 400 is a firmware that does not know it.
  if (reply.statusCode === 400 && !reply.errorMessage)
    throw new Error(
      "the robot's firmware has no core-dump readout; update it to see the last crash"
    )
  if (reply.statusCode !== 200) throw new Error(refusal(reply))
  if (!reply.coreDumpReport) throw new Error('the robot answered without a core-dump report')
  return reply.coreDumpReport
}

/** The dump's bytes, header and checksum included, as `esp-coredump info_corefile -t raw` reads them. */
export const downloadCoreDump = async (
  link: CoreDumpLink,
  onProgress?: (done: number, total: number) => void
): Promise<Uint8Array<ArrayBuffer>> => {
  const parts: Uint8Array[] = []
  let offset = 0
  let total: number | undefined
  do {
    const reply = await link.request({ coreDumpRead: { offset, length: CORE_DUMP_CHUNK } })
    const chunk = reply.fileChunk
    if (reply.statusCode !== 200 || !chunk)
      throw new Error(`read at offset ${offset} failed with status ${reply.statusCode}`)
    if (total !== undefined && chunk.totalSize !== total)
      throw new Error('the core dump changed while it downloaded')
    total = chunk.totalSize
    if (chunk.content.length === 0 && offset < total)
      throw new Error(`read at offset ${offset} returned no data`)
    parts.push(chunk.content)
    offset += chunk.content.length
    onProgress?.(offset, total)
  } while (offset < total)

  const bytes = new Uint8Array(offset)
  let at = 0
  for (const part of parts) {
    bytes.set(part, at)
    at += part.length
  }
  return bytes
}

/** Erases the stored dump; the robot refuses unless it rests. Resolves to the report after the erase. */
export const eraseCoreDump = async (link: CoreDumpLink): Promise<CoreDumpReport> => {
  const reply = await link.request({ coreDumpErase: {} })
  if (reply.statusCode !== 200) throw new Error(refusal(reply))
  if (!reply.coreDumpReport) throw new Error('the robot answered without a core-dump report')
  return reply.coreDumpReport
}

/** Named after the build that crashed, whose ELF decodes it. */
export const coreDumpFilename = (report: CoreDumpReport) =>
  report.appElfSha256 ? `coredump-${report.appElfSha256.slice(0, 8)}.bin` : 'coredump.bin'
