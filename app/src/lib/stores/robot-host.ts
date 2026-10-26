import { derived, writable, type Readable } from 'svelte/store'
import { location } from './location-store'

/**
 * The IP the socket tries in place of a saved robot's name while the name does not open, null while it is the name
 * that is tried. It belongs to the address it was found for: another location forgets it.
 */
export const fallbackHost = writable<string | null>(null)

location.subscribe(() => fallbackHost.set(null))

/** The host the socket opens and the robot's HTTP paths resolve against. */
export const robotHost: Readable<string> = derived(
  [fallbackHost, location],
  ([$fallbackHost, $location]) => $fallbackHost ?? ($location || window.location.host)
)
