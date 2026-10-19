import { dataBroker } from '$lib/transport/databroker'
import { ask } from '$lib/transport/reply'
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
): Promise<void> => {
  const total = bytes.length
  for (let offset = 0; offset < total; offset += CHUNK) {
    const content = bytes.slice(offset, offset + CHUNK)
    await ask(
      dataBroker,
      { fileWriteChunk: { path, offset, totalSize: total, content } },
      `the write at offset ${offset}`
    )
    onProgress?.(offset + content.length, total)
  }
}

const writeWithRetry = async (
  path: string,
  bytes: Uint8Array,
  onProgress?: (sent: number, total: number) => void
): Promise<string | null> => {
  const attempt = () =>
    writeAll(path, bytes, onProgress).then(
      () => null,
      (e: unknown) => (e instanceof Error ? e.message : `write failed: ${e}`)
    )
  return (await attempt()) === null ? null : attempt()
}

export const validateAnimation = async (name: string): Promise<AnimationReport> => {
  // The firmware answers a refused file with status 422 and the report, so the report decides.
  const res = await ask(dataBroker, { animationValidate: { name } }, `the validation of ${name}`, {
    accept: [422]
  })
  if (!res.animationReport) throw new Error(`the validation of ${name} returned no report`)
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
    const res = await ask(
      dataBroker,
      { fileReadChunk: { path, offset, length: CHUNK } },
      `the read of ${name} at offset ${offset}`
    )
    if (!res.fileChunk) throw new Error(`the read of ${name} at offset ${offset} returned no chunk`)
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
  const res = await ask(dataBroker, { animationListRequest: {} }, 'the animation library')
  return res.animationList?.entries ?? []
}

export const deleteAnimation = async (name: string): Promise<void> => {
  await ask(dataBroker, { fileDelete: { path: animationPath(name) } }, `deleting ${name}`)
}
