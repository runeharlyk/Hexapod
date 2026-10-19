import { dataBroker } from '$lib/transport/databroker'
import { PeripheralSettings, ServoSettings } from '$lib/platform_shared/api'
import { ask, RequestFailed, type RequestPayload } from '$lib/transport/reply'

const FORMAT = 'hexapod-robot-config'
const VERSION = 1

/**
 * What makes a robot this robot, saved to a file: its servo calibration and its peripheral
 * settings. Network settings are left out: the robot never sends its passwords.
 */
export interface RobotConfig {
  format: typeof FORMAT
  version: number
  exportedAt: string
  servo: ServoSettings
  peripherals: PeripheralSettings
}

const NOT_A_CONFIG = 'This file is not a hexapod configuration'

const isObject = (value: unknown): value is Record<string, unknown> =>
  typeof value === 'object' && value !== null && !Array.isArray(value)

// The reply's reason after this module's own sentence; a refusal without a reason is the sentence alone.
async function askOr(payload: RequestPayload, feature: string, refusal: string) {
  try {
    return await ask(dataBroker, payload, feature)
  } catch (error) {
    if (
      error instanceof RequestFailed &&
      error.failure.kind === 'refused' &&
      !error.failure.message
    )
      throw new Error(refusal)
    throw new Error(`${refusal}: ${error instanceof Error ? error.message : error}`)
  }
}

async function robotServos(): Promise<ServoSettings> {
  const { servoSettings } = await askOr(
    { servoSettingsGet: {} },
    'the servo calibration',
    'The robot did not send its servo calibration'
  )
  if (!servoSettings) throw new Error('The robot did not send its servo calibration')
  return servoSettings
}

export async function exportConfig(now = new Date()): Promise<RobotConfig> {
  const servo = await robotServos()
  const { peripheralSettings } = await askOr(
    { peripheralSettingsGet: {} },
    'the peripheral settings',
    'The robot did not send its peripheral settings'
  )
  if (!peripheralSettings) throw new Error('The robot did not send its peripheral settings')
  return {
    format: FORMAT,
    version: VERSION,
    exportedAt: now.toISOString(),
    servo,
    peripherals: peripheralSettings
  }
}

export function serializeConfig(config: RobotConfig): string {
  return JSON.stringify(
    {
      ...config,
      servo: ServoSettings.toJSON(config.servo),
      peripherals: PeripheralSettings.toJSON(config.peripherals)
    },
    null,
    2
  )
}

export function parseConfig(text: string): RobotConfig {
  let file: Record<string, unknown> | null
  try {
    file = JSON.parse(text)
  } catch {
    throw new Error(NOT_A_CONFIG)
  }
  if (file?.format !== FORMAT) throw new Error(NOT_A_CONFIG)
  if (file.version !== VERSION)
    throw new Error(
      `This file is configuration version ${file.version}; this app reads version ${VERSION}`
    )
  if (!isObject(file.servo) || !Array.isArray(file.servo.servos) || !isObject(file.peripherals))
    throw new Error(NOT_A_CONFIG)
  return {
    format: FORMAT,
    version: VERSION,
    exportedAt: typeof file.exportedAt === 'string' ? file.exportedAt : '',
    servo: ServoSettings.fromJSON(file.servo),
    peripherals: PeripheralSettings.fromJSON(file.peripherals)
  }
}

// Compared on the wire encoding, so float32 rounding on the robot does not read as a refusal.
const sameServos = (a: ServoSettings, b: ServoSettings) =>
  ServoSettings.encode(a).finish().join() === ServoSettings.encode(b).finish().join()

// The firmware's own bounds (settings/peripherals_settings.h): any S3 GPIO, two distinct pins, and
// the I2C standard to fast-plus range.
const validI2c = ({ sda, scl, frequency }: PeripheralSettings) =>
  [sda, scl].every(pin => pin >= 0 && pin <= 48) &&
  sda !== scl &&
  frequency >= 100000 &&
  frequency <= 1000000

/**
 * Writes the calibration, then the peripheral settings. A calibration is only meaningful for the
 * servo layout it was made on, so a file with another servo count, or with I2C settings the robot
 * would refuse, is refused before anything is written. The robot answers a refused calibration with the one it kept, so the echo is checked.
 */
export async function applyConfig(config: RobotConfig) {
  const current = await robotServos()
  if (config.servo.servos.length !== current.servos.length)
    throw new Error(
      `This file holds ${config.servo.servos.length} servos; the robot has ${current.servos.length}`
    )
  if (!validI2c(config.peripherals)) throw new Error('The I2C settings in this file are invalid')

  const servoRefusal = 'The robot rejected the servo calibration'
  const { servoSettings } = await askOr(
    { servoSettingsUpdate: config.servo },
    'the servo calibration',
    servoRefusal
  )
  if (!servoSettings || !sameServos(servoSettings, config.servo)) throw new Error(servoRefusal)

  await askOr(
    { peripheralSettingsUpdate: await keepingCompassOffset(config.peripherals) },
    'the peripheral settings',
    'The robot rejected the peripheral settings; the servo calibration was already applied'
  )
}

// A file from before the compass offset existed, or from an uncalibrated robot, holds no offset (a zero is not
// written to JSON), and restoring it must not wipe the offset the robot has fitted since.
async function keepingCompassOffset(saved: PeripheralSettings): Promise<PeripheralSettings> {
  if (saved.magOffsetX || saved.magOffsetY || saved.magOffsetZ) return saved
  const { peripheralSettings } = await askOr(
    { peripheralSettingsGet: {} },
    'the peripheral settings',
    'The robot did not send its peripheral settings'
  )
  if (!peripheralSettings) throw new Error('The robot did not send its peripheral settings')
  const { magOffsetX, magOffsetY, magOffsetZ } = peripheralSettings
  return { ...saved, magOffsetX, magOffsetY, magOffsetZ }
}
