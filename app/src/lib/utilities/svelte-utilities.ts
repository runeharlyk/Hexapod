import { writable } from 'svelte/store'
import { browser } from '$app/environment'

const load = <T>(key: string, initialValue: T): T => {
  if (!browser) return initialValue
  try {
    const savedValue = localStorage.getItem(key)
    return savedValue !== null ? (JSON.parse(savedValue) as T) : initialValue
  } catch {
    return initialValue
  }
}

export const persistentStore = <T>(key: string, initialValue: T) => {
  const store = writable<T>(load(key, initialValue))

  store.subscribe(value => {
    if (!browser) return
    try {
      localStorage.setItem(key, JSON.stringify(value))
    } catch {
      // Storage full or blocked: the value still lives in memory for this session.
    }
  })

  return store
}
