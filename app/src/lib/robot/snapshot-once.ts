import type { MessageFns } from '$lib/platform_shared/message'
import { dataBroker, REQUEST_TIMEOUT_MS, type DataBroker } from '$lib/transport/databroker'
import { RequestFailed } from '$lib/transport/reply'
import type { StateTopic, ValueOf } from '$lib/transport/topics'

/**
 * A state topic's current value, read once: follows the topic until the robot's snapshot is in and live, resolves with
 * it, and stops following. Rejects with the failure the topic ends in when the robot never sends a value. As a request
 * of its own would, it fails at once as disconnected while no link is up, and as timed out when no snapshot is in
 * within a read's timeout, as when the link drops and none comes back. For a reader that wants the value as of now and
 * no pushes after, such as the configuration export.
 */
export const snapshotOnce = <F extends StateTopic>(
  topic: F,
  broker: DataBroker = dataBroker
): Promise<ValueOf<F>> => {
  if (!broker.linked()) return Promise.reject(new RequestFailed({ kind: 'disconnected' }))
  let unfollow!: () => void
  let timer: ReturnType<typeof setTimeout> | undefined
  const read = new Promise<ValueOf<F>>((resolve, reject) => {
    timer = setTimeout(() => reject(new RequestFailed({ kind: 'timeout' })), REQUEST_TIMEOUT_MS)
    unfollow = broker.follow(topic as MessageFns<ValueOf<F>>, state => {
      if (state.status === 'live' && state.value !== undefined) resolve(state.value)
      else if (state.status === 'error') reject(new RequestFailed(state.failure!))
    })
  })
  return read.finally(() => {
    clearTimeout(timer)
    unfollow()
  })
}
