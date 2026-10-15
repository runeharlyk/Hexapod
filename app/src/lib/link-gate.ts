import type { LinkState } from '$lib/stores/link'

export type GateState = 'show' | 'connecting' | 'offline'

// A settings page asks the robot for its state on mount, and without a link that request would
// only wait out its 15 s timeout. Once shown, the page stays shown through a dropped link, so a
// half-edited form is not thrown away; its next request fails visibly instead.
export const gateState = (status: LinkState['status'], shownBefore: boolean): GateState => {
  if (status === 'connected' || shownBefore) return 'show'
  return status === 'connecting' ? 'connecting' : 'offline'
}
