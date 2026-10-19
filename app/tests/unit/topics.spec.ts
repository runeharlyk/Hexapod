// @vitest-environment node
// The app's topic table against platform_shared/topics.json, which the firmware's table is checked against too
// (firmware/test/test_topic_snapshot), so neither side can call a topic state that the other streams.
import { describe, expect, it } from 'vitest'
import { readFileSync } from 'node:fs'
import { resolve } from 'node:path'
import { STATE_TOPICS, STREAM_TOPICS, kindOf } from '../../src/lib/transport/topics'
import { keyOf, tagOf } from '../../src/lib/transport/message-codec'
import type { MessageFns } from '../../src/lib/platform_shared/message'

const shared: { topics: { name: string; tag: number; kind: 'state' | 'stream' }[] } = JSON.parse(
  readFileSync(resolve(__dirname, '../../../platform_shared/topics.json'), 'utf8')
)

const camel = (name: string) => name.replace(/_(\w)/g, (_, c: string) => c.toUpperCase())

describe('the topic table', () => {
  it('names every shared topic with its tag and kind', () => {
    for (const { name, tag, kind } of shared.topics)
      expect([name, kindOf(tag)]).toEqual([name, kind])
  })

  it('lists no topic the shared table does not', () => {
    const app = [...STATE_TOPICS, ...STREAM_TOPICS].map(topic => {
      const fns = topic as MessageFns<unknown>
      return [keyOf(fns), tagOf(fns)]
    })
    const expected = shared.topics.map(({ name, tag }) => [camel(name), tag])
    expect(app.sort()).toEqual(expected.sort())
  })
})
