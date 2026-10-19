import type { Page, WebSocketRoute } from '@playwright/test'
import { FakeRobot, type Client } from './robot'

/**
 * Puts the fake robot behind the WebSocket the layout opens by itself on an http origin. `drop()` closes the link and
 * refuses every later one, as a robot that went out of reach.
 */
export const serveRobot = async (page: Page, robot: FakeRobot) => {
  const open = new Set<WebSocketRoute>()
  let reachable = true
  await page.routeWebSocket('**/api/ws', socket => {
    if (!reachable) return void socket.close()
    const client: Client = { deliver: bytes => socket.send(Buffer.from(bytes)) }
    const hangUp = robot.connect(client)
    open.add(socket)
    socket.onMessage(frame => {
      if (typeof frame !== 'string') robot.receive(client, new Uint8Array(frame))
    })
    socket.onClose(() => {
      open.delete(socket)
      hangUp()
    })
  })
  return {
    drop: () => {
      reachable = false
      open.forEach(socket => socket.close())
    }
  }
}

export { FakeRobot }
