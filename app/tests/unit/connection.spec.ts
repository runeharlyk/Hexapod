import { describe, it, expect } from 'vitest'
import { wifiAction } from '../../src/lib/connection'
import { socketUrl } from '../../src/lib/transport/websocket-adapter'

describe('wifiAction', () => {
  it('offers to leave the robot the socket is on', () => {
    expect(wifiAction('connected', '192.168.1.151', '192.168.1.151')).toBe('disconnect')
  })

  it('offers to stop retrying while the socket cannot reach its robot', () => {
    expect(wifiAction('connecting', '192.0.2.1', '192.0.2.1')).toBe('stop')
  })

  it('switches robots when the field names another address, even while connected', () => {
    expect(wifiAction('connected', '192.168.1.151', '192.168.1.39')).toBe('connect')
    expect(wifiAction('connecting', '192.168.1.39', '192.168.1.151')).toBe('connect')
  })

  it('ignores whitespace around the typed address', () => {
    expect(wifiAction('connected', 'hexapod.local', ' hexapod.local ')).toBe('disconnect')
  })

  it('connects when nothing is up', () => {
    expect(wifiAction('disconnected', '192.168.1.151', '192.168.1.151')).toBe('connect')
  })
})

describe('socketUrl', () => {
  it('points at the firmware socket endpoint on the given host', () => {
    expect(socketUrl('192.168.1.151')).toBe('ws://192.168.1.151/api/ws')
    expect(socketUrl('localhost:5173')).toBe('ws://localhost:5173/api/ws')
  })
})
