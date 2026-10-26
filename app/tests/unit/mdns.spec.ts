import { describe, it, expect } from 'vitest'
import { readFileSync } from 'node:fs'
import { resolve } from 'node:path'
import {
  advertisedLabel,
  MDNS_LIMITS,
  localName,
  scanTypes,
  serviceTypeLabel,
  sortScanResults,
  txtLabel
} from '../../src/lib/mdns'

describe('mDNS labels', () => {
  it('names the host as other devices resolve it', () => {
    expect(localName('hexapod')).toBe('hexapod.local')
  })

  it('writes a service the way DNS-SD browsers list it', () => {
    expect(advertisedLabel({ service: '_ws', protocol: '_tcp', port: 80, txtRecords: [] })).toBe(
      '_ws._tcp on port 80'
    )
  })

  it('shows a TXT record as key=value, and a bare key alone', () => {
    expect(txtLabel({ key: 'Firmware Version', value: '0.1.1' })).toBe('Firmware Version=0.1.1')
    expect(txtLabel({ key: 'flag', value: '' })).toBe('flag')
  })
})

describe('scanTypes', () => {
  it('offers _http._tcp even when the robot advertises nothing', () => {
    expect(scanTypes([]).map(serviceTypeLabel)).toEqual(['_http._tcp'])
  })

  it('adds the robot own types once each, after _http._tcp', () => {
    const advertised = [
      { service: '_ws', protocol: '_tcp' },
      { service: '_http', protocol: '_tcp' },
      { service: '_ws', protocol: '_tcp' }
    ]
    expect(scanTypes(advertised).map(serviceTypeLabel)).toEqual(['_http._tcp', '_ws._tcp'])
  })
})

describe('sortScanResults', () => {
  it('orders by address numerically, not as text', () => {
    const sorted = sortScanResults([
      { name: 'b', ip: '192.168.1.151', port: 80 },
      { name: 'a', ip: '192.168.1.20', port: 80 }
    ])
    expect(sorted.map(r => r.ip)).toEqual(['192.168.1.20', '192.168.1.151'])
  })

  it('puts a host without an address last and breaks ties by name', () => {
    const sorted = sortScanResults([
      { name: 'printer', ip: '', port: 631 },
      { name: 'hexapod', ip: '10.0.0.5', port: 80 },
      { name: 'b', ip: '10.0.0.2', port: 80 },
      { name: 'a', ip: '10.0.0.2', port: 8080 }
    ])
    expect(sorted.map(r => r.name)).toEqual(['a', 'b', 'hexapod', 'printer'])
  })

  it('leaves the response it was given untouched', () => {
    const results = [
      { name: 'z', ip: '10.0.0.9', port: 80 },
      { name: 'y', ip: '10.0.0.1', port: 80 }
    ]
    sortScanResults(results)
    expect(results[0].name).toBe('z')
  })
})

// The editor's caps are the wire's: a string's max_size less its terminator, a list's max_count.
describe('MDNS_LIMITS', () => {
  const options = readFileSync(resolve(__dirname, '../../../platform_shared/api.options'), 'utf8')
  const limit = (field: string, kind: 'max_size' | 'max_count') => {
    const escaped = field.replaceAll('.', String.raw`\.`)
    const found = options.match(new RegExp(String.raw`^api\.${escaped}\s+${kind}:(\d+)`, 'm'))
    if (!found) throw new Error(`api.options has no ${kind} for ${field}`)
    return Number(found[1])
  }

  it('match the sizes platform_shared/api.options gives the mDNS settings', () => {
    expect(MDNS_LIMITS).toEqual({
      hostname: limit('MDNSSettings.hostname', 'max_size') - 1,
      instance: limit('MDNSSettings.instance', 'max_size') - 1,
      serviceType: limit('MDNSServiceDef.service', 'max_size') - 1,
      protocol: limit('MDNSServiceDef.protocol', 'max_size') - 1,
      txtKey: limit('MDNSTxtRecord.key', 'max_size') - 1,
      txtValue: limit('MDNSTxtRecord.value', 'max_size') - 1,
      services: limit('MDNSSettings.services', 'max_count'),
      txtPerService: limit('MDNSServiceDef.txt_records', 'max_count'),
      globalTxt: limit('MDNSSettings.global_txt_records', 'max_count')
    })
  })
})
