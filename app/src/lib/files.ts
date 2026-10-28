import { dataBroker } from '$lib/transport/databroker'
import { ask, type RequestLink, type RequestPayload } from '$lib/transport/reply'
import type { CorrelationResponse } from '$lib/platform_shared/message'
import type { FileEntry } from '$lib/platform_shared/api'

export type { FileEntry }

/** The most a chunk carries, either way (api.proto's FileWriteChunk and FileReadChunk). */
export const CHUNK = 512

/** What a transfer has moved of its total, in bytes, after each chunk. */
export type Progress = (done: number, total: number) => void

type Broker = RequestLink<RequestPayload, CorrelationResponse>

/** The path of `name` in the directory `parent`, which the root names as `/`. */
export const childPath = (parent: string, name: string) =>
  parent.endsWith('/') ? parent + name : `${parent}/${name}`

/**
 * Every entry of a directory, directories first, then each group by name, as the robot pages them. The directory may
 * change between two pages, which shifts an entry across the boundary: each name is kept once, and the next page is
 * asked for by the robot's own position, not by the entries kept.
 */
export const listDirectory = async (
  path: string,
  broker: Broker = dataBroker
): Promise<FileEntry[]> => {
  const entries = new Map<string, FileEntry>()
  let offset = 0
  let total = 0
  do {
    const res = await ask(broker, { fileList: { path, offset } }, `listing ${path}`)
    if (!res.fileList) throw new Error(`the listing of ${path} returned no entries`)
    total = res.fileList.total
    if (res.fileList.entries.length === 0) break
    for (const entry of res.fileList.entries)
      if (!entries.has(entry.name)) entries.set(entry.name, entry)
    offset += res.fileList.entries.length
  } while (offset < total)
  return [...entries.values()]
}

/** Whether the robot has the file, asked with a read of no bytes; a refusal other than "not there" rejects. */
export const fileExists = async (path: string, broker: Broker = dataBroker): Promise<boolean> => {
  const res = await ask(
    broker,
    { fileReadChunk: { path, offset: 0, length: 0 } },
    `looking for ${path}`,
    { accept: [404] }
  )
  return res.statusCode !== 404
}

export const readFile = async (
  path: string,
  onProgress?: Progress,
  broker: Broker = dataBroker
): Promise<Uint8Array> => {
  const parts: Uint8Array[] = []
  let offset = 0
  let total = 0
  do {
    const res = await ask(
      broker,
      { fileReadChunk: { path, offset, length: CHUNK } },
      `the read of ${path} at offset ${offset}`
    )
    if (!res.fileChunk) throw new Error(`the read of ${path} at offset ${offset} returned no chunk`)
    if (res.fileChunk.content.length === 0 && offset < res.fileChunk.totalSize)
      throw new Error(`read at offset ${offset} returned no data`)
    parts.push(res.fileChunk.content)
    offset += res.fileChunk.content.length
    total = res.fileChunk.totalSize
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

/** Writes the file from its first byte; an empty file is one empty chunk, which the robot takes as complete. */
export const writeFile = async (
  path: string,
  bytes: Uint8Array,
  onProgress?: Progress,
  broker: Broker = dataBroker
): Promise<void> => {
  const total = bytes.length
  let offset = 0
  do {
    const content = bytes.slice(offset, offset + CHUNK)
    await ask(
      broker,
      { fileWriteChunk: { path, offset, totalSize: total, content } },
      `the write of ${path} at offset ${offset}`
    )
    offset += content.length
    onProgress?.(offset, total)
  } while (offset < total)
}

/** Makes the directory with its parents; one that is there is no failure. */
export const makeDirectory = async (path: string, broker: Broker = dataBroker): Promise<void> => {
  await ask(broker, { fileMkdir: { path } }, `creating ${path}`)
}

export const deleteFile = async (path: string, broker: Broker = dataBroker): Promise<void> => {
  await ask(broker, { fileDelete: { path } }, `deleting ${path}`)
}
