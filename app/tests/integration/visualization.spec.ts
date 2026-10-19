import { expect, test } from '@playwright/test'
import { Message } from '../../src/lib/platform_shared/message'
import { MotionModes } from '../../src/lib/motion'

const base = process.env.BASE_PATH ?? ''

const modeIndex = (mode: MotionModes) => Object.values(MotionModes).indexOf(mode)

// The preview origin is http, so the layout opens ws://<host>/api/ws by itself; the route stands in for the robot and
// keeps every mode the app sends it.
test("the 3D view's panel leaves the robot's mode to the controller", async ({ page }) => {
  const sentModes: number[] = []
  await page.routeWebSocket('**/api/ws', socket => {
    socket.onMessage(frame => {
      if (typeof frame === 'string') return
      const mode = Message.decode(new Uint8Array(frame)).mode
      if (mode) sentModes.push(mode.mode)
    })
  })
  await page.addInitScript(() =>
    localStorage.setItem('selected_view', JSON.stringify('3D representation'))
  )

  await page.goto(`${base}/controller`)
  const panel = page.locator('#three-gui-panel')
  await panel.getByText('Controls', { exact: true }).click()
  await panel.getByRole('button', { name: 'center', exact: true }).click()
  // The controller's own mode button, sent after the panel's: the first mode the robot hears must be this one.
  await page.getByRole('button', { name: 'Stand', exact: true }).click()

  await expect.poll(() => sentModes).toEqual([modeIndex(MotionModes.STAND)])
})
