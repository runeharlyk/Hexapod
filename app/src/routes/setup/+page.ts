import type { PageLoad } from './$types'

export const load = (async () => {
  return {
    title: 'Set up over USB'
  }
}) satisfies PageLoad
