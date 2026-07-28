import { get } from 'svelte/store'
import { location } from '$lib/stores'
import { Request, Response } from '$lib/platform_shared/api'

export function resolveUrl(url: string): string {
  if (url.startsWith('http') || !get(location)) return url
  const protocol = typeof window !== 'undefined' ? window.location.protocol : 'http:'
  return `${protocol}//${get(location)}${url.startsWith('/') ? '' : '/'}${url}`
}

export async function protoGet(url: string): Promise<Response | null> {
  try {
    const res = await fetch(resolveUrl(url), { method: 'GET' })
    if (!res.ok) return null
    return Response.decode(new Uint8Array(await res.arrayBuffer()))
  } catch {
    return null
  }
}

export async function protoPost(url: string, request: Request): Promise<Response | null> {
  try {
    const res = await fetch(resolveUrl(url), {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-protobuf' },
      body: Request.encode(request).finish()
    })
    if (!res.ok) return null
    return Response.decode(new Uint8Array(await res.arrayBuffer()))
  } catch {
    return null
  }
}

// IPv4 <-> uint32 matching the firmware IPAddress packing: little-endian, byte 0 is the first octet.
export const ipToString = (v: number): string =>
  [v & 0xff, (v >> 8) & 0xff, (v >> 16) & 0xff, (v >> 24) & 0xff].join('.')

export const ipToU32 = (s: string): number => {
  const p = s.split('.').map(n => Number(n) & 0xff)
  return ((p[0] ?? 0) | ((p[1] ?? 0) << 8) | ((p[2] ?? 0) << 16) | ((p[3] ?? 0) << 24)) >>> 0
}
