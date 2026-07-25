import { persistentStore } from '$lib/utilities'
import { get } from 'svelte/store'

export interface Robot {
  name: string
  address: string
  lastSeenAt: number | null
}

export const robots = persistentStore<Robot[]>('robots', [])

export const subnetPrefix = persistentStore<string>('subnet_prefix', '')

export const addRobot = (address: string, name = address) => {
  robots.update(list =>
    list.some(r => r.address === address) ? list : [...list, { name, address, lastSeenAt: null }]
  )
}

export const forgetRobot = (address: string) => {
  robots.update(list => list.filter(r => r.address !== address))
}

export const renameRobot = (address: string, name: string) => {
  robots.update(list => list.map(r => (r.address === address ? { ...r, name } : r)))
}

export const markSeen = (address: string) => {
  robots.update(list =>
    list.map(r => (r.address === address ? { ...r, lastSeenAt: Date.now() } : r))
  )
}

export const savedAddresses = () => get(robots).map(r => r.address)
