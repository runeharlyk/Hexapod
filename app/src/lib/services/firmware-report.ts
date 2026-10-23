import type { FirmwareReport } from '$lib/stores/robots'

const REPORT_TIMEOUT_MS = 3000

const isObject = (value: unknown): value is Record<string, unknown> =>
  typeof value === 'object' && value !== null && !Array.isArray(value)

/** The report in an /api/features answer; null when the answer is not one. */
export const parseFirmwareReport = (features: unknown): FirmwareReport | null => {
  if (!isObject(features)) return null
  const { firmware_version, firmware_elf_sha256, servos_calibrated, robot_id } = features
  if (typeof firmware_version !== 'string' || !firmware_version) return null
  return {
    version: firmware_version,
    build: typeof firmware_elf_sha256 === 'string' ? firmware_elf_sha256 : '',
    servosCalibrated: typeof servos_calibrated === 'boolean' ? servos_calibrated : null,
    robotId: typeof robot_id === 'string' && robot_id ? robot_id : null
  }
}

/**
 * Asks the robot at the address what it runs; null when it does not answer in time or answers
 * something else. Meant for a robot that just answered the socket probe: its local network access
 * prompt is then answered, so the timeout cannot run out behind it, and nothing is asked of an
 * address that may not be a robot.
 */
export const readFirmwareReport = async (
  address: string,
  signal?: AbortSignal
): Promise<FirmwareReport | null> => {
  if (signal?.aborted) return null
  const controller = new AbortController()
  const abort = () => controller.abort()
  signal?.addEventListener('abort', abort, { once: true })
  const timer = setTimeout(abort, REPORT_TIMEOUT_MS)
  try {
    const response = await fetch(`http://${address}/api/features`, {
      cache: 'no-store',
      signal: controller.signal
    })
    return response.ok ? parseFirmwareReport(await response.json()) : null
  } catch {
    return null
  } finally {
    clearTimeout(timer)
    signal?.removeEventListener('abort', abort)
  }
}
