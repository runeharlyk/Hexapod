import { readable, type Readable } from 'svelte/store'
import type { MessageFns } from '$lib/platform_shared/message'
import { dataBroker, type DataBroker } from '$lib/transport/databroker'
import type { StateTopic, ValueOf, Watched } from '$lib/transport/topics'

const NOTHING_YET: Watched<never> = {
  value: undefined,
  status: 'subscribing',
  stale: false,
  failure: null
}

/**
 * A state topic of the robot as a store: its value, from the snapshot the robot answers the subscribe with and every
 * push after, and whether that value is live, stale or failed (transport/topics.ts). The robot is
 * subscribed to while the store has a subscriber, and every watch of a topic shares one subscription. A stream has no
 * current value and is followed with the data broker's `on`.
 */
export const watch = <F extends StateTopic>(
  topic: F,
  broker: DataBroker = dataBroker
): Readable<Watched<ValueOf<F>>> =>
  readable<Watched<ValueOf<F>>>(NOTHING_YET, set =>
    broker.follow(topic as MessageFns<ValueOf<F>>, set)
  )
