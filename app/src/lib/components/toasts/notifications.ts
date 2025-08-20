import { writable } from 'svelte/store'

type StateType = 'info' | 'success' | 'warning' | 'error'

type State = {
  id: string
  type: StateType
  message: string
}

function createNotificationStore() {
  const { subscribe, update, set } = writable<State[]>([])

  function send(message: string, type: StateType = 'info', timeout: number) {
    const id = generateId()
    update(s => [...s, { id, type, message }])
    if (timeout > 0) setTimeout(() => update(s => s.filter(n => n.id !== id)), timeout)
    return id
  }

  return {
    subscribe,
    send,
    error: (msg: string, timeout: number = 4000) => send(msg, 'error', timeout),
    warning: (msg: string, timeout: number = 4000) => send(msg, 'warning', timeout),
    info: (msg: string, timeout: number = 4000) => send(msg, 'info', timeout),
    success: (msg: string, timeout: number = 4000) => send(msg, 'success', timeout)
  }
}

const generateId = () =>
  typeof crypto !== 'undefined' && 'randomUUID' in crypto ?
    crypto.randomUUID()
  : Math.random().toString(36).slice(2, 11)

export const notifications = createNotificationStore()
