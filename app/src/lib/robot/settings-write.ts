import type {
  CorrelationRequest,
  CorrelationResponse,
  MessageFns
} from '$lib/platform_shared/message'
import { dataBroker, type DataBroker } from '$lib/transport/databroker'
import { ask } from '$lib/transport/reply'
import type { SettingsTopic } from '$lib/transport/topics'

/**
 * Sends a settings update and counts it into the topic's `pending` from the send to the reply that settles it:
 * answered, refused, timed out or dropped. The value a page shows never comes from the write; it comes from the
 * snapshot and the pushes (robot/watch.ts), so a refused or unchanged update leaves it as it was. Rejects as `ask` does.
 */
export const writeSettings = async (
  topic: SettingsTopic,
  payload: Omit<CorrelationRequest, 'correlationId'>,
  what: string,
  broker: DataBroker = dataBroker
): Promise<CorrelationResponse> => {
  const end = broker.writing(topic as MessageFns<unknown>)
  try {
    return await ask(broker, payload, what)
  } finally {
    end()
  }
}
