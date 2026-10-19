import type { Page } from '@playwright/test'
import { FakeRobot, serveRobot } from '../fake-robot/playwright'
import type { Answer, Responder } from '../fake-robot/robot'

/**
 * A robot with firmware from before the snapshots behind the WebSocket the layout opens by itself on an http origin:
 * every request goes to `answer`, whose reply, when it settles, is sent back with the request's correlation id and
 * status 200 unless it names another, pings get pongs, and a subscribe gets nothing.
 */
export const fakeRobot = (page: Page, answer: Responder) => {
  const robot = new FakeRobot()
  robot.ackless()
  robot.respondOtherwise(answer)
  return serveRobot(page, robot)
}

/** What a firmware that does not know the request answers: the 501 of main.cpp's `default:`. */
export const tooOld = (): Answer => ({ statusCode: 501 })

/** What firmware up to v0.1.1 answers to a request it does not know: a bare 400, as to a malformed one. */
export const tooOldUpToV011 = (): Answer => ({ statusCode: 400 })
