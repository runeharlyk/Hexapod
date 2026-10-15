import { persistentStore } from '$lib/utilities'
import { writable } from 'svelte/store'
import appEnv from 'app-env'

export const location = appEnv.VITE_USE_HOST_NAME ? writable('') : persistentStore('location', '')

// The firmware has no TLS, so it answers only http:// and ws://. From an https page, Chromium 147+
// exempts private IPs and .local names from mixed-content blocking once the user grants local network
// access; Firefox and Safari block them, leaving Bluetooth and USB. The hosted https app has no robot at
// its own host, so it needs a saved address to reach one.
export const canReachRobot = (pageUrl: URL, address: string): boolean =>
  pageUrl.protocol !== 'https:' || address !== ''
