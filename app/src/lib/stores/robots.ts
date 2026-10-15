import { persistentStore } from '$lib/utilities'
import { get } from 'svelte/store'

/** What a robot says it runs, from its /api/features. */
export interface FirmwareReport {
  version: string
  /** SHA-256 of the running image's ELF; tells two builds of one version apart. */
  build: string
  /** Null when its firmware predates reporting it: unknown, not uncalibrated. */
  servosCalibrated: boolean | null
}

export interface Robot {
  name: string
  address: string
  lastSeenAt: number | null
  /** The last report, kept to show while the robot cannot be asked; absent until one arrives. */
  firmware?: FirmwareReport & { reportedAt: number }
}

export const robots = persistentStore<Robot[]>('robots', [])

export const subnetPrefix = persistentStore<string>('subnet_prefix', '')

export const addRobot = (address: string, name = address) => {
  robots.update(list =>
    list.some(r => r.address === address) ? list : [...list, { name, address, lastSeenAt: null }]
  )
}

export const forgetRobot = (address: string) => {
  robots.update(list => list.filter(r => r.address !== address))
}

export const renameRobot = (address: string, name: string) => {
  robots.update(list => list.map(r => (r.address === address ? { ...r, name } : r)))
}

export const markSeen = (address: string) => {
  robots.update(list =>
    list.map(r => (r.address === address ? { ...r, lastSeenAt: Date.now() } : r))
  )
}

// A report replaces the previous one whole: a value the new report lacks is unknown now, not the
// value an earlier firmware gave.
export const recordFirmware = (address: string, report: FirmwareReport) => {
  robots.update(list =>
    list.map(r =>
      r.address === address ? { ...r, firmware: { ...report, reportedAt: Date.now() } } : r
    )
  )
}

export const savedAddresses = () => get(robots).map(r => r.address)

export const agoLabel = (timestamp: number, now = Date.now()) => {
  const minutes = Math.round((now - timestamp) / 60000)
  if (minutes < 60) return `${Math.max(1, minutes)} min ago`
  return `${Math.round(minutes / 60)} h ago`
}

const calibrationLabel = (servosCalibrated: boolean | null) => {
  if (servosCalibrated === null) return 'calibration unknown'
  return servosCalibrated ? 'servos calibrated' : 'servos not calibrated'
}

/**
 * The robot's firmware and servo calibration. `live` says the robot reported it during this visit;
 * otherwise the stored report is shown as the last known one, with its age.
 */
export const firmwareLabel = ({ firmware }: Robot, live: boolean, now = Date.now()) => {
  if (!firmware) return 'Firmware unknown'
  const summary = (lead: string) =>
    [
      `${lead} ${firmware.version}`,
      firmware.build && `build ${firmware.build.slice(0, 8)}`,
      calibrationLabel(firmware.servosCalibrated)
    ]
      .filter(Boolean)
      .join(' · ')
  if (live) return summary('Firmware')
  return `Last known ${agoLabel(firmware.reportedAt, now)}: ${summary('firmware')}`
}
