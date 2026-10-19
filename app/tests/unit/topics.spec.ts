// @vitest-environment node
// The app's state topics against platform_shared/topics.json, which the firmware's table is checked against too
// (firmware/test/test_topic_snapshot), so the app can never watch, as a state, a topic the firmware streams.
import { describe, expect, it } from 'vitest'
import { readFileSync } from 'node:fs'
import { resolve } from 'node:path'
import { STATE_TOPICS } from '../../src/lib/transport/topics'
import { keyOf, tagOf } from '../../src/lib/transport/message-codec'
import type { MessageFns } from '../../src/lib/platform_shared/message'

const shared: { topics: { name: string; tag: number; kind: 'state' | 'stream' }[] } = JSON.parse(
  readFileSync(resolve(__dirname, '../../../platform_shared/topics.json'), 'utf8')
)

const camel = (name: string) => name.replace(/_(\w)/g, (_, c: string) => c.toUpperCase())

describe('the state topics', () => {
  it('are the shared table state topics, each with its tag', () => {
    const app = STATE_TOPICS.map(topic => {
      const fns = topic as MessageFns<unknown>
      return [keyOf(fns), tagOf(fns)]
    })
    const expected = shared.topics
      .filter(({ kind }) => kind === 'state')
      .map(({ name, tag }) => [camel(name), tag])
    expect(app.sort()).toEqual(expected.sort())
  })
})
