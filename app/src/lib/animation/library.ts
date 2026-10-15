import { derived, get, writable } from 'svelte/store'
import { Animation } from '$lib/platform_shared/animation'
import type { AnimationEntry } from '$lib/platform_shared/message'
import { loadAnimationJson } from './model'
import { listAnimations } from './transfer'

const files = import.meta.glob('../../../../animations/*.json', { eager: true, import: 'default' })

const stem = (path: string) => path.slice(path.lastIndexOf('/') + 1, -'.json'.length)

// A broken bundled file throws here, at import. The build does not run this module (ssr = false),
// so the editor unit test, which imports it, is what fails on one.
export const builtIn: Animation[] = Object.entries(files)
  .map(([path, json]) => {
    const loaded = loadAnimationJson(JSON.stringify(json))
    if ('error' in loaded) throw new Error(`animations/${stem(path)}.json: ${loaded.error}`)
    if (loaded.animation.name !== stem(path))
      throw new Error(`animations/${stem(path)}.json is named ${loaded.animation.name}`)
    return loaded.animation
  })
  .sort((a, b) => a.name.localeCompare(b.name))

const DRAFTS_KEY = 'animation_drafts'

type Drafts = Record<string, unknown>

// A draft as this tab last saw it; the fingerprint tells whether it has changed since.
export interface DraftRef {
  name: string
  fingerprint: string
}

const storedDrafts = (): Drafts | null => {
  try {
    const text = localStorage.getItem(DRAFTS_KEY)
    return text === null ? {} : (JSON.parse(text) as Drafts)
  } catch {
    return null
  }
}

// A per-browser convenience only: storage may be blocked, full or cleared at any time. The list
// holds only what storage accepted, so a refused write changes nothing and reports null.
const drafts = writable<Drafts>(storedDrafts() ?? {})

// Every tab shares the storage, so reads go to it and the list follows other tabs' writes.
const currentDrafts = () => storedDrafts() ?? get(drafts)

export const reloadDrafts = () => drafts.set(currentDrafts())

if (typeof window !== 'undefined')
  window.addEventListener('storage', e => {
    if (e.key === DRAFTS_KEY || e.key === null) reloadDrafts()
  })

const fingerprintOf = (json: unknown) => JSON.stringify(json)

const storeDrafts = (next: Drafts): boolean => {
  try {
    localStorage.setItem(DRAFTS_KEY, JSON.stringify(next))
  } catch {
    return false
  }
  drafts.set(next)
  return true
}

export const saveDraft = (a: Animation): DraftRef | null => {
  const json = Animation.toJSON(a)
  if (!storeDrafts({ ...currentDrafts(), [a.name]: json })) return null
  return { name: a.name, fingerprint: fingerprintOf(json) }
}

export const deleteDraft = (name: string): boolean =>
  storeDrafts(Object.fromEntries(Object.entries(currentDrafts()).filter(([key]) => key !== name)))

export const draftRef = (name: string): DraftRef | null => {
  const json = currentDrafts()[name]
  return json === undefined ? null : { name, fingerprint: fingerprintOf(json) }
}

// Saving replaces a stored draft of that name without asking only when it is still the draft this
// document was opened from or last saved as, unchanged by another tab since.
export const overwritesDraft = (name: string, savedAs: DraftRef | null): boolean => {
  const stored = draftRef(name)
  return stored !== null && (savedAs?.name !== name || savedAs.fingerprint !== stored.fingerprint)
}

const parseDraft = (name: string, json: unknown) => {
  try {
    return loadAnimationJson(JSON.stringify(json))
  } catch (e) {
    return { error: `draft ${name} is unreadable: ${(e as Error).message}` }
  }
}

// The animation and the reference come from one read, so the reference is the draft opened.
export const loadDraft = (
  name: string
): { animation: Animation; draft: DraftRef } | { error: string } => {
  const json = currentDrafts()[name]
  if (json === undefined) return { error: `no draft named ${name}` }
  const loaded = parseDraft(name, json)
  if ('error' in loaded) return loaded
  return { animation: loaded.animation, draft: { name, fingerprint: fingerprintOf(json) } }
}

// Drafts that still parse; a draft that no longer validates is left out rather than shown broken.
export const draftAnimations = derived(drafts, d =>
  Object.keys(d)
    .sort()
    .map(name => parseDraft(name, d[name]))
    .flatMap(loaded => ('animation' in loaded ? [loaded.animation] : []))
)

export const robotAnimations = writable<AnimationEntry[] | null>(null)

export const refreshRobotList = async () => {
  try {
    robotAnimations.set(await listAnimations())
  } catch {
    robotAnimations.set(null)
  }
}
