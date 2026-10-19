import { createScan } from '$lib/scan'

export type I2CRow = { address: number; part: string | null; name: string }

type Part = Omit<I2CRow, 'address'>

// The chips the robot carries and the parts it was built to take; the servo boards as docs/firmware.md assigns them.
const PARTS = new Map<number, Part>([
  [0x1e, { part: 'HMC5883', name: '3-axis magnetometer' }],
  [0x40, { part: 'PCA9685', name: '16-channel PWM driver, left legs' }],
  [0x41, { part: 'PCA9685', name: '16-channel PWM driver, right legs' }],
  [0x48, { part: 'ADS1115', name: '4-channel 16-bit ADC' }],
  [0x68, { part: 'MPU6050', name: '6-axis gyroscope and accelerometer' }],
  [0x77, { part: 'BMP085', name: 'Temperature and barometric pressure' }]
])

const UNKNOWN: Part = { part: null, name: 'Unknown device' }

export const hexAddress = (address: number) =>
  `0x${address.toString(16).toUpperCase().padStart(2, '0')}`

export const describeI2C = (addresses: number[]): I2CRow[] =>
  addresses.map(address => ({ address, ...(PARTS.get(address) ?? UNKNOWN) }))

export const i2cScan = createScan<I2CRow[]>()
