import type { Page } from '@playwright/test'
import {
  CorrelationResponse,
  Message,
  type CorrelationRequest
} from '../../src/lib/platform_shared/message'

type Answer = Partial<Omit<CorrelationResponse, 'correlationId'>>

/**
 * Stands in for the robot behind the WebSocket the layout opens by itself on an http origin: every request goes to
 * `answer`, whose reply, when it settles, is sent back with the request's correlation id and status 200 unless it
 * names another. Pings get pongs. A subscribe gets nothing, as from firmware older than the snapshots.
 */
export const fakeRobot = (
  page: Page,
  answer: (request: CorrelationRequest) => Answer | Promise<Answer>
) =>
  page.routeWebSocket('**/api/ws', socket => {
    socket.onMessage(async frame => {
      if (typeof frame === 'string') return
      const message = Message.decode(new Uint8Array(frame))
      if (message.pingmsg) socket.send(Buffer.from(Message.encode({ pongmsg: {} }).finish()))
      const request = message.correlationRequest
      if (!request) return
      const response = CorrelationResponse.create({
        correlationId: request.correlationId,
        statusCode: 200,
        ...(await answer(request))
      })
      socket.send(Buffer.from(Message.encode({ correlationResponse: response }).finish()))
    })
  })

/** What a firmware that does not know the request answers: the 501 of main.cpp's `default:`. */
export const tooOld = (): Answer => ({ statusCode: 501 })

/** What firmware up to v0.1.1 answers to a request it does not know: a bare 400, as to a malformed one. */
export const tooOldUpToV011 = (): Answer => ({ statusCode: 400 })
