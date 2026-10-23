import { get } from 'svelte/store'
import { probeAddress } from '$lib/services/discovery'
import { readFirmwareReport } from '$lib/services/firmware-report'
import { markSeen, recordFirmware, robots } from '$lib/stores/robots'

export interface Refreshed {
  online: boolean
  /** Whether the robot reported its firmware, which is recorded. */
  reported: boolean
}

/**
 * Asks whether a saved robot answers, at its address and else at the IP it last reported, and records what it says of
 * itself. Another robot may answer at that IP once the router handed it on, so the IP counts only when the robot
 * there reports the id the saved robot has.
 */
export const refreshSavedRobot = async (
  address: string,
  signal?: AbortSignal
): Promise<Refreshed> => {
  if (await probeAddress(address)) {
    markSeen(address)
    const report = await readFirmwareReport(address, signal)
    if (report) recordFirmware(address, report)
    return { online: true, reported: report !== null }
  }

  const { robotId, lastIp } = get(robots).find(r => r.address === address) ?? {}
  if (!robotId || !lastIp || !(await probeAddress(lastIp)))
    return { online: false, reported: false }
  const report = await readFirmwareReport(lastIp, signal)
  if (report?.robotId !== robotId) return { online: false, reported: false }
  markSeen(address)
  recordFirmware(address, report)
  return { online: true, reported: true }
}
