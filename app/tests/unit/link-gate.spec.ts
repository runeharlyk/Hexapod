import { describe, it, expect } from 'vitest'
import { gateState } from '../../src/lib/link-gate'

describe('gateState', () => {
  it('shows the page only once a link is up', () => {
    expect(gateState('connected', false)).toBe('show')
    expect(gateState('connecting', false)).toBe('connecting')
    expect(gateState('disconnected', false)).toBe('offline')
  })

  it('keeps a shown page through a dropped or reconnecting link', () => {
    expect(gateState('disconnected', true)).toBe('show')
    expect(gateState('connecting', true)).toBe('show')
  })
})
