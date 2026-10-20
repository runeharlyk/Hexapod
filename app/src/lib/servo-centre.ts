import { dataBroker } from '$lib/transport/databroker'
import { ask, type Reply, type RequestLink } from '$lib/transport/reply'

export type CentreResult = { ok: true } | { ok: false; error: string }

/**
 * Every servo to its calibrated centre pulse, for lining the joints up with their marks on the calibration page. The
 * robot does it at its next tick and only in IDLE; otherwise it refuses with its reason (firmware/src/main.cpp).
 */
export const centreServos = async (
  link: RequestLink<{ servoCentre: object }, Reply> = dataBroker
): Promise<CentreResult> => {
  try {
    await ask(link, { servoCentre: {} }, 'centring the servos')
    return { ok: true }
  } catch (error) {
    return { ok: false, error: error instanceof Error ? error.message : String(error) }
  }
}
