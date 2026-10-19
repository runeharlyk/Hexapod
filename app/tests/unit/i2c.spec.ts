import { describe, expect, it } from 'vitest'
import { describeI2C, hexAddress } from '../../src/lib/i2c'

describe('describeI2C', () => {
  it('names the robot own chips', () => {
    const rows = describeI2C([0x1e, 0x40, 0x41, 0x68])
    expect(rows.map(r => r.part)).toEqual(['HMC5883', 'PCA9685', 'PCA9685', 'MPU6050'])
    expect(rows[1].name).toMatch(/left legs/)
    expect(rows[2].name).toMatch(/right legs/)
  })

  it('keeps an address it does not know, without a part', () => {
    expect(describeI2C([0x23])).toEqual([{ address: 0x23, part: null, name: 'Unknown device' }])
  })

  // The robot reply used to stop at 16 addresses, so a bus with more never showed its real devices.
  it('lists every address of a reply longer than 16, in order', () => {
    const addresses = Array.from({ length: 20 }, (_, i) => 0x10 + i).concat([0x40, 0x68])
    const rows = describeI2C(addresses)
    expect(rows.map(r => r.address)).toEqual(addresses)
    expect(rows.at(-1)?.part).toBe('MPU6050')
  })
})

describe('hexAddress', () => {
  it('writes a 7-bit address as two hex digits', () => {
    expect(hexAddress(0x08)).toBe('0x08')
    expect(hexAddress(0x1e)).toBe('0x1E')
    expect(hexAddress(0x77)).toBe('0x77')
  })
})
