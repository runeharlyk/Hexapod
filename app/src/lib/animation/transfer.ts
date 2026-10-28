import { dataBroker } from '$lib/transport/databroker'
import { ask } from '$lib/transport/reply'
import { Animation } from '$lib/platform_shared/animation'
import type { AnimationEntry, AnimationReport } from '$lib/platform_shared/message'
import { deleteFile, readFile, writeFile, type Progress } from '$lib/files'
import { froundAnimation, validate } from './model'

export const animationPath = (name: string) => '/animations/' + name + '.pb'

export type UploadResult = { ok: true; report: AnimationReport } | { ok: false; error: string }

const writeWithRetry = async (
  path: string,
  bytes: Uint8Array,
  onProgress?: Progress
): Promise<string | null> => {
  const attempt = () =>
    writeFile(path, bytes, onProgress).then(
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
  onProgress?: Progress
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
  const bytes = await readFile(animationPath(name))
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
  await deleteFile(animationPath(name))
}
