import type { Page } from '@playwright/test'
import {
  CorrelationResponse,
  Message,
  type CorrelationRequest
} from '../../src/lib/platform_shared/message'

type Answer = Partial<Omit<CorrelationResponse, 'correlationId'>>

/**
 * Stands in for the robot behind the WebSocket the layout opens by itself on an http origin: every request goes to
 * `answer`, whose reply is sent back with the request's correlation id and status 200 unless it names another.
 */
export const fakeRobot = (page: Page, answer: (request: CorrelationRequest) => Answer) =>
  page.routeWebSocket('**/api/ws', socket => {
    socket.onMessage(frame => {
      if (typeof frame === 'string') return
      const request = Message.decode(new Uint8Array(frame)).correlationRequest
      if (!request) return
      const response = CorrelationResponse.create({
        correlationId: request.correlationId,
        statusCode: 200,
        ...answer(request)
      })
      socket.send(Buffer.from(Message.encode({ correlationResponse: response }).finish()))
    })
  })

/** What a firmware from before every request answers: the bare 400 of main.cpp's `default:`. */
export const tooOld = (): Answer => ({ statusCode: 400 })
