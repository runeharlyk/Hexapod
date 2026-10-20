import { expect, test, type Page } from '@playwright/test'
import { Message } from '../../src/lib/platform_shared/message'
import { MotionModes, modeToWire } from '../../src/lib/motion'

const base = process.env.BASE_PATH ?? ''

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

  await expect.poll(() => sentModes).toEqual([modeToWire(MotionModes.STAND)])
})

type Counted = { glDraws: number }

// Counts the WebGL draw calls of every context on the page in window.glDraws.
const countDraws = (page: Page) =>
  page.addInitScript(() => {
    const counted = window as unknown as Counted
    counted.glDraws = 0
    const methods = ['drawArrays', 'drawElements', 'drawArraysInstanced', 'drawElementsInstanced']
    for (const context of [WebGLRenderingContext, WebGL2RenderingContext]) {
      const prototype = context.prototype as unknown as Record<string, (...args: unknown[]) => void>
      for (const method of methods) {
        const draw = prototype[method]
        if (!draw) continue
        prototype[method] = function (this: unknown, ...args: unknown[]) {
          counted.glDraws++
          return draw.apply(this, args)
        }
      }
    }
  })

const draws = (page: Page) => page.evaluate(() => (window as unknown as Counted).glDraws)

const drawsWithin = (page: Page, ms: number) =>
  page.evaluate(async ms => {
    const counted = window as unknown as Counted
    const before = counted.glDraws
    await new Promise(resolve => setTimeout(resolve, ms))
    return counted.glDraws - before
  }, ms)

// A software-rendered frame of this scene holds the page's main thread for hundreds of milliseconds, so a view that
// redraws an unchanged scene every frame leaves the page too busy to answer a click.
test('the 3D view draws only while what it shows changes', async ({ page }) => {
  await page.routeWebSocket('**/api/ws', () => {})
  await countDraws(page)
  await page.addInitScript(() =>
    localStorage.setItem('selected_view', JSON.stringify('3D representation'))
  )

  await page.goto(`${base}/controller`)
  const panel = page.locator('#three-gui-panel')
  await panel.getByText('Controls', { exact: true }).click()
  await expect.poll(() => drawsWithin(page, 1000), { timeout: 15_000 }).toBe(0)

  // Straightening the legs moves the model, so the view draws it, and settles again once the pose holds.
  const settled = await draws(page)
  await panel.getByRole('button', { name: 'center', exact: true }).click()
  await expect.poll(() => draws(page)).toBeGreaterThan(settled)
  await expect.poll(() => drawsWithin(page, 1000), { timeout: 15_000 }).toBe(0)
})
