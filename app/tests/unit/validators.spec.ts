import { describe, expect, it } from 'vitest'
import { IPV4_PATTERN } from '$lib/components/layout/validators'

// As the browser applies an input's pattern attribute: anchored to the whole value, with the v flag.
const accepts = (value: string) => new RegExp(`^(?:${IPV4_PATTERN})$`, 'v').test(value)

describe('IPV4_PATTERN', () => {
  it('accepts dotted addresses whose octets are 0 to 255', () => {
    for (const address of ['192.168.4.1', '0.0.0.0', '255.255.255.0', '10.0.0.255', '001.02.3.4'])
      expect(accepts(address), address).toBe(true)
  })

  it('rejects an octet over 255, a missing or extra octet, and anything around the address', () => {
    for (const value of [
      '256.1.1.1',
      '1.2.3.260',
      '1.2.3',
      '1.2.3.4.5',
      '1.2.3.',
      ' 1.2.3.4',
      '1.2.3.4 ',
      'a1.2.3.4',
      '1..2.3',
      '1a2b3c4',
      '10-0-0-1',
      ''
    ])
      expect(accepts(value), JSON.stringify(value)).toBe(false)
  })
})
