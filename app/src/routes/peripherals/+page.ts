import type { PageLoad } from './$types'
import { goto } from '$app/navigation'
import { resolve } from '$app/paths'

const base = resolve('/')

export const load = (async () => {
  goto(base)
  return
}) satisfies PageLoad
