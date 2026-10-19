import { describe, expect, it } from 'vitest'
import { get } from 'svelte/store'
import { answered, createScan } from '../../src/lib/scan'
import type { CorrelationResponse } from '../../src/lib/platform_shared/message'

// A robot request the test settles: each call is one request on the wire.
const fakeRequest = <R>() => {
  const calls: { resolve: (r: R) => void; reject: (e: Error) => void }[] = []
  const request = () => new Promise<R>((resolve, reject) => calls.push({ resolve, reject }))
  return { request, calls }
}

const reply = (fields: Partial<CorrelationResponse>) =>
  ({ correlationId: 1, statusCode: 200, errorMessage: '', ...fields }) as CorrelationResponse

describe('createScan', () => {
  it('shows the scan running, then its result', async () => {
    const scan = createScan<number[]>()
    const robot = fakeRequest<number[]>()

    const done = scan.run(robot.request)
    expect(get(scan)).toEqual({ state: 'scanning' })
    robot.calls[0].resolve([0x40, 0x41])
    await done

    expect(get(scan)).toEqual({ state: 'done', result: [0x40, 0x41] })
  })

  // A hot reload or a reconnect mounts the page again while its first scan is out: both mounts share that one.
  it('runs one scan at a time: a second start while one runs joins it', async () => {
    const scan = createScan<number[]>()
    const robot = fakeRequest<number[]>()

    const first = scan.run(robot.request)
    const second = scan.run(robot.request)
    expect(robot.calls).toHaveLength(1)
    robot.calls[0].resolve([0x68])
    await Promise.all([first, second])

    expect(get(scan)).toEqual({ state: 'done', result: [0x68] })
  })

  it('scans again once the last scan has ended, as opening the page again does', async () => {
    const scan = createScan<number[]>()
    const robot = fakeRequest<number[]>()

    const first = scan.run(robot.request)
    robot.calls[0].resolve([])
    await first
    const second = scan.run(robot.request)

    expect(robot.calls).toHaveLength(2)
    expect(get(scan)).toEqual({ state: 'scanning' })
    robot.calls[1].resolve([0x1e])
    await second
    expect(get(scan)).toEqual({ state: 'done', result: [0x1e] })
  })

  // The link dropping mid-scan rejects the request; the page mounted again after the reconnect scans afresh.
  it('reports a failed request and lets the next start scan anew', async () => {
    const scan = createScan<number[]>()
    const robot = fakeRequest<number[]>()

    const first = scan.run(robot.request)
    robot.calls[0].reject(new Error('disconnected'))
    await first
    expect(get(scan)).toEqual({ state: 'failed', message: 'disconnected' })

    const second = scan.run(robot.request)
    expect(robot.calls).toHaveLength(2)
    robot.calls[1].resolve([0x40])
    await second
    expect(get(scan)).toEqual({ state: 'done', result: [0x40] })
  })
})

// The data broker resolves every reply, refusals included: a 503 must not read as "no devices".
describe('answered', () => {
  it('passes a 200 reply through', () => {
    const ok = reply({ statusCode: 200 })
    expect(answered(ok)).toBe(ok)
  })

  it('turns a refusal into an error with the robot reason', () => {
    expect(() => answered(reply({ statusCode: 503, errorMessage: 'The I2C bus is busy' }))).toThrow(
      'The I2C bus is busy'
    )
  })

  it('names the status when the robot gave no reason', () => {
    expect(() => answered(reply({ statusCode: 400 }))).toThrow('status 400')
  })
})
