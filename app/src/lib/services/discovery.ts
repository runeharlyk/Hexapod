import { savedAddresses } from '$lib/stores/robots'

const PROBE_TIMEOUT_MS = 1500
const SWEEP_TIMEOUT_MS = 1200
const SWEEP_CONCURRENCY = 24
const SWEEP_FIRST_HOST = 1
const SWEEP_LAST_HOST = 254

// Default mDNS hostname is "esp32" (firmware/include/settings/mdns_settings.h); "hexapod"
// covers a renamed device. 192.168.4.1 is the SoftAP address when the robot hosts its own network.
const WELL_KNOWN_HOSTS = ['esp32.local', 'hexapod.local', '192.168.4.1']

/**
 * Probes by opening the WebSocket the app actually uses. Unlike fetch this is not subject
 * to CORS, and a successful open proves the robot's protocol endpoint is live rather than
 * just that something answers on the address.
 */
export const probeAddress = (
  address: string,
  timeoutMs = PROBE_TIMEOUT_MS,
  signal?: AbortSignal
): Promise<boolean> =>
  new Promise(resolve => {
    if (signal?.aborted) {
      resolve(false)
      return
    }

    let socket: WebSocket
    try {
      socket = new WebSocket(`ws://${address}/api/ws`)
    } catch {
      resolve(false)
      return
    }

    const settle = (found: boolean) => {
      clearTimeout(timer)
      signal?.removeEventListener('abort', onAbort)
      socket.onopen = null
      socket.onerror = null
      socket.onclose = null
      if (socket.readyState === WebSocket.CONNECTING || socket.readyState === WebSocket.OPEN) {
        socket.close()
      }
      resolve(found)
    }

    const onAbort = () => settle(false)
    const timer = setTimeout(() => settle(false), timeoutMs)
    signal?.addEventListener('abort', onAbort, { once: true })

    socket.onopen = () => settle(true)
    socket.onerror = () => settle(false)
    socket.onclose = () => settle(false)
  })

export const candidateAddresses = () => [...new Set([...WELL_KNOWN_HOSTS, ...savedAddresses()])]

export interface CandidateStatus {
  address: string
  state: 'probing' | 'found' | 'absent'
}

export const probeCandidates = async (
  onUpdate: (statuses: CandidateStatus[]) => void,
  signal?: AbortSignal
) => {
  const statuses: CandidateStatus[] = candidateAddresses().map(address => ({
    address,
    state: 'probing'
  }))
  onUpdate([...statuses])

  await Promise.all(
    statuses.map(async status => {
      const found = await probeAddress(status.address, PROBE_TIMEOUT_MS, signal)
      status.state = found ? 'found' : 'absent'
      onUpdate([...statuses])
    })
  )

  return statuses.filter(s => s.state === 'found').map(s => s.address)
}

/** Accepts "192.168.1", "192.168.1.", "192.168.1.42" or "192.168.1.0/24" -> "192.168.1." */
export const normalizeSubnetPrefix = (value: string) => {
  const parts = value
    .trim()
    .replace(/\/\d+$/, '')
    .split('.')
    .filter(Boolean)
  if (parts.length < 3) return null
  if (!parts.slice(0, 3).every(part => /^\d{1,3}$/.test(part) && Number(part) <= 255)) return null
  return `${parts.slice(0, 3).join('.')}.`
}

export interface SweepHandlers {
  onProgress?: (done: number, total: number) => void
  onFound?: (address: string) => void
  signal?: AbortSignal
}

export const sweepSubnet = async (prefix: string, handlers: SweepHandlers = {}) => {
  const { onProgress, onFound, signal } = handlers
  const hosts = Array.from(
    { length: SWEEP_LAST_HOST - SWEEP_FIRST_HOST + 1 },
    (_, i) => `${prefix}${SWEEP_FIRST_HOST + i}`
  )

  const found: string[] = []
  let cursor = 0
  let done = 0

  const worker = async () => {
    while (cursor < hosts.length && !signal?.aborted) {
      const address = hosts[cursor++]
      const reachable = await probeAddress(address, SWEEP_TIMEOUT_MS, signal)
      done++
      onProgress?.(done, hosts.length)
      if (reachable) {
        found.push(address)
        onFound?.(address)
      }
    }
  }

  await Promise.all(Array.from({ length: SWEEP_CONCURRENCY }, worker))
  return found
}
