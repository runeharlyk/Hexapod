import { dataBroker } from '$lib/transport/databroker'
import { Animation } from '$lib/platform_shared/animation'
import type { AnimationEntry, AnimationReport } from '$lib/platform_shared/message'
import { froundAnimation, validate } from './model'

export const CHUNK = 512

export const animationPath = (name: string) => '/animations/' + name + '.pb'

export type UploadResult = { ok: true; report: AnimationReport } | { ok: false; error: string }

const writeAll = async (
  path: string,
  bytes: Uint8Array,
  onProgress?: (sent: number, total: number) => void
): Promise<string | null> => {
  const total = bytes.length
  for (let offset = 0; offset < total; offset += CHUNK) {
    const content = bytes.slice(offset, offset + CHUNK)
    const res = await dataBroker.request({
      fileWriteChunk: { path, offset, totalSize: total, content }
    })
    if (res.statusCode !== 200)
      return `write at offset ${offset} failed with status ${res.statusCode}`
    onProgress?.(offset + content.length, total)
  }
  return null
}

const writeWithRetry = async (
  path: string,
  bytes: Uint8Array,
  onProgress?: (sent: number, total: number) => void
): Promise<string | null> => {
  const attempt = () => writeAll(path, bytes, onProgress).catch(e => `write failed: ${e}`)
  return (await attempt()) === null ? null : attempt()
}

export const validateAnimation = async (name: string): Promise<AnimationReport> => {
  const res = await dataBroker.request({ animationValidate: { name } })
  // The firmware answers a refused file with status 422 and the report, so the report decides.
  if (!res.animationReport) throw new Error(`validate ${name} failed with status ${res.statusCode}`)
  return res.animationReport
}

export const uploadAnimation = async (
  a: Animation,
  onProgress?: (sent: number, total: number) => void
): Promise<UploadResult> => {
  const failure = await writeWithRetry(
    animationPath(a.name),
    Animation.encode(a).finish(),
    onProgress
  )
  if (failure) return { ok: false, error: failure }
  const report = await validateAnimation(a.name)
  return report.ok ? { ok: true, report } : { ok: false, error: report.error }
}

export const downloadAnimation = async (name: string): Promise<Animation> => {
  const path = animationPath(name)
  const parts: Uint8Array[] = []
  let offset = 0
  let totalSize = 0
  do {
    const res = await dataBroker.request({ fileReadChunk: { path, offset, length: CHUNK } })
    if (res.statusCode !== 200 || !res.fileChunk)
      throw new Error(`read at offset ${offset} failed with status ${res.statusCode}`)
    if (res.fileChunk.content.length === 0 && offset < res.fileChunk.totalSize)
      throw new Error(`read at offset ${offset} returned no data`)
    parts.push(res.fileChunk.content)
    offset += res.fileChunk.content.length
    totalSize = res.fileChunk.totalSize
  } while (offset < totalSize)

  const bytes = new Uint8Array(offset)
  let at = 0
  for (const part of parts) {
    bytes.set(part, at)
    at += part.length
  }
  const animation = froundAnimation(Animation.decode(bytes))
  const error = validate(animation)
  if (error) throw new Error(`${name} is invalid: ${error}`)
  return animation
}

export const listAnimations = async (): Promise<AnimationEntry[]> => {
  const res = await dataBroker.request({ animationListRequest: {} })
  if (res.statusCode !== 200) throw new Error(`list failed with status ${res.statusCode}`)
  return res.animationList?.entries ?? []
}

export const deleteAnimation = async (name: string): Promise<void> => {
  const res = await dataBroker.request({ fileDelete: { path: animationPath(name) } })
  if (res.statusCode !== 200) throw new Error(`delete ${name} failed with status ${res.statusCode}`)
}
