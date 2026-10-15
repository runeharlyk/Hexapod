import { get } from 'svelte/store'
import { location } from '$lib/stores/location-store'

// Points robot-relative paths at the saved robot address, over http even from an https page, as the
// robot serves no TLS; absolute URLs pass through.
export function resolveUrl(url: string): string {
  if (url.startsWith('http') || !get(location)) return url
  return `http://${get(location)}${url.startsWith('/') ? '' : '/'}${url}`
}

// IPv4 <-> uint32 matching the firmware IPAddress packing: little-endian, byte 0 is the first octet.
export const ipToString = (v: number): string =>
  [v & 0xff, (v >> 8) & 0xff, (v >> 16) & 0xff, (v >> 24) & 0xff].join('.')

export const ipToU32 = (s: string): number => {
  const p = s.split('.').map(n => Number(n) & 0xff)
  return ((p[0] ?? 0) | ((p[1] ?? 0) << 8) | ((p[2] ?? 0) << 16) | ((p[3] ?? 0) << 24)) >>> 0
}
