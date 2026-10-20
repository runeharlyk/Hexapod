import { describe, expect, it } from 'vitest'
import { reorder } from 'svelte-dnd-list'
import { afterDelete, afterMove, NONE } from '../../src/routes/wifi/sta/selection'

const NETWORKS = ['home', 'garage', 'lab', 'office']

describe('the selected network', () => {
  it('stays on its network when one before it is deleted', () => {
    const remaining = NETWORKS.filter((_, index) => index !== 0)
    expect(remaining[afterDelete(2, 0)]).toBe('lab')
  })

  it('stays put when one after it is deleted', () => {
    expect(afterDelete(1, 3)).toBe(1)
  })

  it('is none once its own network is deleted', () => {
    expect(afterDelete(1, 1)).toBe(NONE)
  })

  it('follows its network through every move of the list', () => {
    for (let selected = 0; selected < NETWORKS.length; selected++)
      for (let from = 0; from < NETWORKS.length; from++)
        for (let to = 0; to < NETWORKS.length; to++) {
          const moved = reorder(NETWORKS, from, to)
          expect(moved[afterMove(selected, from, to)], `${selected}: ${from} -> ${to}`).toBe(
            NETWORKS[selected]
          )
        }
  })
})
