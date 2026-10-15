import { describe, it, expect, beforeEach } from 'vitest'
import {
  PENDING_UPDATE_TTL_MS,
  forgetUpdate,
  judgeUpdate,
  pendingUpdate,
  rememberUpdate
} from '../../src/lib/firmware/update-outcome'

const OLD = 'a'.repeat(64)
const NEW = 'b'.repeat(64)

describe('judgeUpdate', () => {
  it('reports an uploaded image that runs after the restart', () => {
    const update = { expectedSha256: NEW, previousSha256: OLD, requestedAt: 0 }
    expect(judgeUpdate(update, NEW, true)).toBe('updated')
  })

  it('reports a rollback when the previous image runs after the restart', () => {
    const update = { expectedSha256: NEW, previousSha256: OLD, requestedAt: 0 }
    expect(judgeUpdate(update, OLD, true)).toBe('rolled-back')
  })

  it('judges a download the robot fetched by whether the hash changed', () => {
    const update = { previousSha256: OLD, requestedAt: 0 }
    expect(judgeUpdate(update, NEW, true)).toBe('updated')
    expect(judgeUpdate(update, OLD, true)).toBe('rolled-back')
  })

  it('calls nothing a rollback before the robot restarted', () => {
    const update = { expectedSha256: NEW, previousSha256: OLD, requestedAt: 0 }
    expect(judgeUpdate(update, OLD, false)).toBeNull()
    expect(judgeUpdate(update, NEW, false)).toBe('updated')
  })

  it('stays undecided without a hash to compare', () => {
    expect(judgeUpdate({ previousSha256: '', requestedAt: 0 }, NEW, true)).toBeNull()
    expect(judgeUpdate({ previousSha256: OLD, requestedAt: 0 }, '', true)).toBeNull()
  })
})

describe('pendingUpdate', () => {
  beforeEach(() => forgetUpdate())

  it('survives a page reload through storage', () => {
    rememberUpdate({ previousSha256: OLD, requestedAt: 1000 })
    expect(pendingUpdate(2000)).toEqual({ previousSha256: OLD, requestedAt: 1000 })
  })

  it('drops an update that never finished', () => {
    rememberUpdate({ previousSha256: OLD, requestedAt: 0 })
    expect(pendingUpdate(PENDING_UPDATE_TTL_MS + 1)).toBeNull()
    expect(pendingUpdate(0)).toBeNull()
  })
})
