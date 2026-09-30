import { derived, get, writable } from 'svelte/store'
import { Animation } from '$lib/platform_shared/animation'
import type { AnimationEntry } from '$lib/platform_shared/message'
import { persistentStore } from '$lib/utilities/svelte-utilities'
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

// A per-browser convenience only: storage may be blocked, full or cleared at any time.
export const drafts = persistentStore('animation_drafts', {} as Record<string, unknown>)

export const saveDraft = (a: Animation): boolean => {
  try {
    drafts.update(d => ({ ...d, [a.name]: Animation.toJSON(a) }))
    return true
  } catch {
    return false
  }
}

export const deleteDraft = (name: string) => {
  try {
    drafts.update(d => Object.fromEntries(Object.entries(d).filter(([key]) => key !== name)))
  } catch {
    // The draft stays in memory until the page reloads.
  }
}

export const loadDraft = (name: string): { animation: Animation } | { error: string } => {
  try {
    const json = get(drafts)[name]
    if (json === undefined) return { error: `no draft named ${name}` }
    return loadAnimationJson(JSON.stringify(json))
  } catch (e) {
    return { error: `draft ${name} is unreadable: ${(e as Error).message}` }
  }
}

// Drafts that still parse; a draft that no longer validates is left out rather than shown broken.
export const draftAnimations = derived(drafts, d =>
  Object.keys(d)
    .sort()
    .map(loadDraft)
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
