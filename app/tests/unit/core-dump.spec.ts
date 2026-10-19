import { describe, expect, it } from 'vitest'
import type { CoreDumpReport } from '../../src/lib/platform_shared/message'
import {
  CORE_DUMP_CHUNK,
  coreDumpFilename,
  downloadCoreDump,
  eraseCoreDump,
  lastCrash,
  readCoreDump,
  type CoreDumpLink,
  type CoreDumpReply
} from '../../src/lib/stores/core-dump'

const RUNNING = '3f9a1c0be47d5a2961c8f0e3b7a4d6c2e9f1a8b5c3d7e0f2a4b6c8d0e1f3a5b7'

const report = (fields: Partial<CoreDumpReport>): CoreDumpReport => ({
  stored: false,
  size: 0,
  readable: false,
  task: '',
  pc: 0,
  backtrace: [],
  backtraceCorrupted: false,
  panicReason: '',
  appElfSha256: '',
  elfMatchesRunning: false,
  ...fields
})

const watchdogCrash = report({
  stored: true,
  size: 1300,
  readable: true,
  task: 'control',
  pc: 0x42012345,
  backtrace: [0x42012345, 0x42023456, 0x40378abc],
  panicReason: 'Task watchdog got triggered.',
  appElfSha256: RUNNING,
  elfMatchesRunning: true
})

const reply = (fields: Partial<CoreDumpReply>): CoreDumpReply => ({
  statusCode: 200,
  errorMessage: '',
  ...fields
})

// The robot's side of CoreDumpRead, as firmware/include/core_dump.h answers it, over a dump in a buffer.
const robotWith = (dump: Uint8Array) => {
  const requests: unknown[] = []
  const link: CoreDumpLink = {
    request: async payload => {
      requests.push(payload)
      if (!('coreDumpRead' in payload)) throw new Error('unexpected request')
      const { offset, length } = payload.coreDumpRead
      if (length > 512) return reply({ statusCode: 400 })
      const content = dump.slice(offset, offset + length)
      return reply({ fileChunk: { content, totalSize: dump.length } })
    }
  }
  return { link, requests }
}

const answering = (answer: CoreDumpReply): CoreDumpLink => ({ request: async () => answer })

describe('lastCrash', () => {
  it('shows no crash when no dump is stored', () => {
    expect(lastCrash(report({}))).toEqual({ kind: 'none' })
  })

  it('names the task, the PC, the backtrace and the build of a stored crash', () => {
    expect(lastCrash(watchdogCrash)).toEqual({
      kind: 'crash',
      task: 'control',
      pc: '0x42012345',
      backtrace: '0x42012345 0x42023456 0x40378abc',
      panicReason: 'Task watchdog got triggered.',
      elf: RUNNING,
      build: 'running'
    })
  })

  // The firmware compares the dump's hash with the running build's (core_dump::sameFirmware).
  it('says when the crash came from another build than the running one', () => {
    const view = lastCrash({
      ...watchdogCrash,
      appElfSha256: '0'.repeat(64),
      elfMatchesRunning: false
    })
    expect(view).toMatchObject({ kind: 'crash', build: 'other', elf: '0'.repeat(64) })
  })

  // A dump without its version note names no build, so no hash can be matched against a release's.
  it('says when the dump does not name the build that crashed', () => {
    const view = lastCrash({ ...watchdogCrash, appElfSha256: '', elfMatchesRunning: false })
    expect(view).toMatchObject({ kind: 'crash', build: 'unnamed', elf: '' })
  })

  it('marks a backtrace the unwinding could not trust to its end', () => {
    const view = lastCrash({ ...watchdogCrash, backtrace: [0x42012345], backtraceCorrupted: true })
    expect(view).toMatchObject({ backtrace: '0x42012345 |<-CORRUPTED' })
  })

  it('pads the addresses to eight digits', () => {
    expect(lastCrash({ ...watchdogCrash, pc: 0x1234, backtrace: [] })).toMatchObject({
      pc: '0x00001234',
      backtrace: ''
    })
  })

  it('shows a stored dump it could not read by its size alone', () => {
    expect(lastCrash(report({ stored: true, size: 4096 }))).toEqual({
      kind: 'unreadable',
      size: 4096
    })
  })
})

describe('readCoreDump', () => {
  it("returns the robot's report", async () => {
    expect(await readCoreDump(answering(reply({ coreDumpReport: watchdogCrash })))).toEqual(
      watchdogCrash
    )
  })

  // The request has no fields to get wrong: a bare 400 is a firmware that does not know it.
  it('names a firmware that predates the readout', async () => {
    await expect(readCoreDump(answering(reply({ statusCode: 400 })))).rejects.toThrow(
      "the robot's firmware does not support the core-dump readout; update it"
    )
  })

  it('refuses a reply without a report', async () => {
    await expect(readCoreDump(answering(reply({})))).rejects.toThrow(/without a core-dump report/)
  })
})

describe('downloadCoreDump', () => {
  const dump = Uint8Array.from({ length: 1300 }, (_, i) => (i * 7) & 0xff)

  it('reads the dump in chunks from its first byte to its last', async () => {
    const { link, requests } = robotWith(dump)
    const progress: number[] = []
    const bytes = await downloadCoreDump(link, (done, total) => progress.push(done / total))
    expect(bytes).toEqual(dump)
    expect(requests).toEqual([
      { coreDumpRead: { offset: 0, length: CORE_DUMP_CHUNK } },
      { coreDumpRead: { offset: 512, length: CORE_DUMP_CHUNK } },
      { coreDumpRead: { offset: 1024, length: CORE_DUMP_CHUNK } }
    ])
    expect(progress.at(-1)).toBe(1)
  })

  it('fails when the dump is erased while it downloads', async () => {
    let reads = 0
    const link: CoreDumpLink = {
      request: async () =>
        ++reads === 1 ?
          reply({ fileChunk: { content: dump.slice(0, 512), totalSize: dump.length } })
        : reply({ statusCode: 404 })
    }
    await expect(downloadCoreDump(link)).rejects.toThrow(
      'the robot refused the core-dump read at offset 512 (status 404)'
    )
  })

  // CoreDumpRead has fields, so a bare 400 is a read the robot refused, not a firmware without the readout.
  it('reports a bare 400 to a read as a refusal', async () => {
    await expect(downloadCoreDump(answering(reply({ statusCode: 400 })))).rejects.toThrow(
      'the robot refused the core-dump read at offset 0 (status 400)'
    )
  })

  it('fails on a chunk that brings nothing before the end', async () => {
    const link = answering(reply({ fileChunk: { content: new Uint8Array(), totalSize: 1300 } }))
    await expect(downloadCoreDump(link)).rejects.toThrow(/offset 0 returned no data/)
  })

  // A new crash cannot replace the dump without a restart, but a different size means other bytes.
  it('fails when the dump changes size partway', async () => {
    let reads = 0
    const link: CoreDumpLink = {
      request: async () =>
        reply({
          fileChunk: { content: dump.slice(0, 512), totalSize: ++reads === 1 ? 1300 : 2000 }
        })
    }
    await expect(downloadCoreDump(link)).rejects.toThrow(/changed while it downloaded/)
  })
})

describe('eraseCoreDump', () => {
  it('returns the report after the erase', async () => {
    const after = report({})
    expect(await eraseCoreDump(answering(reply({ coreDumpReport: after })))).toEqual(after)
  })

  it('passes on why the robot refused', async () => {
    const refusal = reply({
      statusCode: 409,
      errorMessage: 'Deactivate the robot or let it idle first',
      coreDumpReport: watchdogCrash
    })
    await expect(eraseCoreDump(answering(refusal))).rejects.toThrow(
      'Deactivate the robot or let it idle first'
    )
  })
})

describe('coreDumpFilename', () => {
  it('names the file after the build that crashed', () => {
    expect(coreDumpFilename(watchdogCrash)).toBe('coredump-3f9a1c0b.bin')
  })

  it('falls back to a plain name for a dump without a hash', () => {
    expect(coreDumpFilename(report({ stored: true }))).toBe('coredump.bin')
  })
})
