import { describe, it, expect, beforeEach, afterEach, vitest } from 'vitest'
import { throttler } from '../../src/lib/utilities/buffer-utilities'

describe('throttler', () => {
  let throttle: throttler
  let sent: string[]
  const send = (value: string) => () => sent.push(value)

  beforeEach(() => {
    vitest.useFakeTimers()
    throttle = new throttler()
    sent = []
  })

  afterEach(() => {
    vitest.useRealTimers()
  })

  it('runs nothing before the window closes', () => {
    throttle.throttle(send('a'), 40)
    vitest.advanceTimersByTime(39)
    expect(sent).toEqual([])
  })

  it('runs only the latest callback of a window', () => {
    throttle.throttle(send('a'), 40)
    vitest.advanceTimersByTime(10)
    throttle.throttle(send('b'), 40)
    vitest.advanceTimersByTime(10)
    throttle.throttle(send('c'), 40)
    vitest.advanceTimersByTime(20)
    expect(sent).toEqual(['c'])
  })

  it('does not extend the window when called again', () => {
    throttle.throttle(send('a'), 40)
    vitest.advanceTimersByTime(30)
    throttle.throttle(send('b'), 40)
    vitest.advanceTimersByTime(10)
    expect(sent).toEqual(['b'])
  })

  it('starts a new window after one fires', () => {
    throttle.throttle(send('a'), 40)
    vitest.advanceTimersByTime(40)
    throttle.throttle(send('b'), 40)
    vitest.advanceTimersByTime(40)
    expect(sent).toEqual(['a', 'b'])
  })

  it('drops the pending callback when cancelled, so a stop sent meanwhile stays last', () => {
    throttle.throttle(send('move'), 40)
    throttle.cancel()
    sent.push('stop')
    vitest.advanceTimersByTime(100)
    expect(sent).toEqual(['stop'])
  })

  it('accepts new callbacks after a cancel', () => {
    throttle.throttle(send('a'), 40)
    throttle.cancel()
    throttle.throttle(send('b'), 40)
    vitest.advanceTimersByTime(40)
    expect(sent).toEqual(['b'])
  })
})
