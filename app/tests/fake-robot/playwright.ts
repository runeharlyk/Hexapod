import type { Page } from '@playwright/test'
import { FakeRobot, type Client } from './robot'

/** Puts the fake robot behind the WebSocket the layout opens by itself on an http origin. */
export const serveRobot = (page: Page, robot: FakeRobot) =>
  page.routeWebSocket('**/api/ws', socket => {
    const client: Client = { deliver: bytes => socket.send(Buffer.from(bytes)) }
    const hangUp = robot.connect(client)
    socket.onMessage(frame => {
      if (typeof frame !== 'string') robot.receive(client, new Uint8Array(frame))
    })
    socket.onClose(() => hangUp())
  })

export { FakeRobot }
