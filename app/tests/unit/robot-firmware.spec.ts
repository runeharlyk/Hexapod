// @vitest-environment-options { "url": "https://runeharlyk.github.io/Hexapod/" }
import { describe, it, expect, afterEach, beforeEach, vi } from 'vitest'
import { get } from 'svelte/store'
import { parseFirmwareReport, readFirmwareReport } from '../../src/lib/services/firmware-report'

// What the robot at 192.168.1.151 answered on /api/features on 2026-10-15 (firmware 0.1.1, which
// predates servos_calibrated).
const answerOf011 = {
  camera: true,
  imu: true,
  mag: true,
  bmp: false,
  mdns: true,
  servo: true,
  analytics: true,
  sleep: true,
  ota: true,
  download_firmware: true,
  upload_firmware: true,
  firmware_version: '0.1.1',
  firmware_name: 'Hexapod',
  firmware_built_target: 'esp32-wroom-camera',
  firmware_elf_sha256: '9a30bade0dff2470a706c39740850ef936e184216ae01a408a887d156990cd09'
}

const loadRobots = async () => {
  vi.resetModules()
  return import('../../src/lib/stores/robots')
}

describe('parseFirmwareReport', () => {
  it('reads the version, build and calibration a robot reports', () => {
    expect(parseFirmwareReport({ ...answerOf011, servos_calibrated: false })).toEqual({
      version: '0.1.1',
      build: answerOf011.firmware_elf_sha256,
      servosCalibrated: false
    })
  })

  it('leaves the calibration unknown, not false, for firmware that does not report it', () => {
    expect(parseFirmwareReport(answerOf011)?.servosCalibrated).toBeNull()
  })

  it('refuses an answer that names no firmware', () => {
    expect(parseFirmwareReport({ ...answerOf011, firmware_version: '' })).toBeNull()
    expect(parseFirmwareReport({ camera: true })).toBeNull()
    expect(parseFirmwareReport('<html>printer</html>')).toBeNull()
    expect(parseFirmwareReport(null)).toBeNull()
  })

  it('does not take a calibration it cannot read as a boolean', () => {
    expect(
      parseFirmwareReport({ ...answerOf011, servos_calibrated: 'false' })?.servosCalibrated
    ).toBeNull()
  })
})

describe('readFirmwareReport', () => {
  beforeEach(() => vi.useFakeTimers())

  afterEach(() => {
    vi.useRealTimers()
    vi.unstubAllGlobals()
  })

  it("asks the robot's own webserver, bypassing the browser cache", async () => {
    const fetchMock = vi.fn(async () => Response.json({ ...answerOf011, servos_calibrated: true }))
    vi.stubGlobal('fetch', fetchMock)

    const report = await readFirmwareReport('192.168.1.151')

    expect(report?.servosCalibrated).toBe(true)
    expect(fetchMock).toHaveBeenCalledWith(
      'http://192.168.1.151/api/features',
      expect.objectContaining({ cache: 'no-store' })
    )
  })

  it('gives up on a robot that never answers instead of keeping the page waiting', async () => {
    vi.stubGlobal(
      'fetch',
      (_url: string, init: RequestInit) =>
        new Promise((_, reject) =>
          init.signal!.addEventListener('abort', () => reject(new DOMException('', 'AbortError')))
        )
    )
    const settled = vi.fn()
    const report = readFirmwareReport('192.168.1.151').then(value => {
      settled(value)
      return value
    })

    await vi.advanceTimersByTimeAsync(2999)
    expect(settled).not.toHaveBeenCalled()
    await vi.advanceTimersByTimeAsync(1)

    await expect(report).resolves.toBeNull()
  })

  it('stops when the page that asked is left', async () => {
    vi.stubGlobal(
      'fetch',
      (_url: string, init: RequestInit) =>
        new Promise((_, reject) =>
          init.signal!.addEventListener('abort', () => reject(new DOMException('', 'AbortError')))
        )
    )
    const leaving = new AbortController()
    const report = readFirmwareReport('192.168.1.151', leaving.signal)

    leaving.abort()

    await expect(report).resolves.toBeNull()
  })

  it('reports nothing for an error status or an answer that is not JSON', async () => {
    vi.stubGlobal('fetch', async () => new Response('Not found', { status: 404 }))
    await expect(readFirmwareReport('192.168.1.151')).resolves.toBeNull()

    vi.stubGlobal('fetch', async () => new Response('<html></html>', { status: 200 }))
    await expect(readFirmwareReport('192.168.1.151')).resolves.toBeNull()

    vi.stubGlobal('fetch', async () => {
      throw new TypeError('Failed to fetch')
    })
    await expect(readFirmwareReport('192.168.1.151')).resolves.toBeNull()
  })
})

describe('the saved robots', () => {
  const report = {
    version: '0.1.1',
    build: answerOf011.firmware_elf_sha256,
    servosCalibrated: true
  }

  beforeEach(() => {
    vi.useFakeTimers()
    vi.setSystemTime(new Date('2026-10-15T12:00:00Z'))
  })

  afterEach(() => vi.useRealTimers())

  it('says the firmware is unknown for a robot that never reported it', async () => {
    const { robots, addRobot, firmwareLabel } = await loadRobots()
    addRobot('192.168.1.151')

    expect(firmwareLabel(get(robots)[0], false)).toBe('Firmware unknown')
  })

  it('shows a report from this visit as current', async () => {
    const { robots, addRobot, recordFirmware, firmwareLabel } = await loadRobots()
    addRobot('192.168.1.151')
    recordFirmware('192.168.1.151', report)

    expect(firmwareLabel(get(robots)[0], true)).toBe(
      'Firmware 0.1.1 · build 9a30bade · servos calibrated'
    )
  })

  it('labels a stored report as the last known one, with its age', async () => {
    const { robots, addRobot, recordFirmware, firmwareLabel } = await loadRobots()
    addRobot('192.168.1.151')
    recordFirmware('192.168.1.151', { ...report, servosCalibrated: false })

    vi.setSystemTime(new Date('2026-10-15T15:00:00Z'))

    expect(firmwareLabel(get(robots)[0], false)).toBe(
      'Last known 3 h ago: firmware 0.1.1 · build 9a30bade · servos not calibrated'
    )
  })

  it('replaces the whole report, so a value the new firmware leaves out becomes unknown', async () => {
    const { robots, addRobot, recordFirmware, firmwareLabel } = await loadRobots()
    addRobot('192.168.1.151')
    recordFirmware('192.168.1.151', { version: '0.2.0', build: 'aa', servosCalibrated: true })

    recordFirmware('192.168.1.151', { ...report, servosCalibrated: null })

    expect(firmwareLabel(get(robots)[0], true)).toBe(
      'Firmware 0.1.1 · build 9a30bade · calibration unknown'
    )
  })

  it('records a report only on the robot that sent it', async () => {
    const { robots, addRobot, recordFirmware } = await loadRobots()
    addRobot('192.168.1.151')
    addRobot('192.168.1.152')

    recordFirmware('192.168.1.152', report)

    expect(get(robots).map(r => r.firmware?.version)).toEqual([undefined, '0.1.1'])
  })
})
