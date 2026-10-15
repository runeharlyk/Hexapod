/// <reference no-default-lib="true"/>
/// <reference lib="esnext" />
/// <reference lib="webworker" />
/// <reference types="@sveltejs/kit" />

import { base, build, files, version } from '$service-worker'

const worker = self as unknown as ServiceWorkerGlobalScope

const CACHE = `shell-${version}`
const SHELL = `${base}/`
const PRECACHED = new Set([...build, ...files, SHELL])
const NAVIGATION_TIMEOUT_MS = 3000

worker.addEventListener('install', event => {
  event.waitUntil(
    caches
      .open(CACHE)
      .then(cache => cache.addAll([...PRECACHED]))
      .then(() => worker.skipWaiting())
  )
})

worker.addEventListener('activate', event => {
  event.waitUntil(
    caches
      .keys()
      .then(keys => Promise.all(keys.filter(key => key !== CACHE).map(key => caches.delete(key))))
      .then(() => worker.clients.claim())
  )
})

const isRobotEndpoint = (pathname: string) =>
  pathname.startsWith('/api/') || pathname.startsWith(`${base}/api/`)

const cachedShell = async () => (await caches.match(SHELL)) ?? Response.error()

// A phone on a hotspot without upstream can hang on DNS, so a slow network loses to the shell.
const networkFirstNavigation = async (request: Request) => {
  const timeout = new Promise<never>((_, reject) =>
    setTimeout(() => reject(new Error('navigation timed out')), NAVIGATION_TIMEOUT_MS)
  )
  try {
    return await Promise.race([fetch(request), timeout])
  } catch {
    return cachedShell()
  }
}

worker.addEventListener('fetch', event => {
  const { request } = event
  if (request.method !== 'GET') return

  const url = new URL(request.url)
  if (url.origin !== worker.location.origin || isRobotEndpoint(url.pathname)) return

  if (PRECACHED.has(url.pathname) && url.pathname !== SHELL) {
    event.respondWith(caches.match(url.pathname).then(cached => cached ?? fetch(request)))
  } else if (request.mode === 'navigate' && url.pathname.startsWith(SHELL)) {
    event.respondWith(networkFirstNavigation(request))
  }
})
