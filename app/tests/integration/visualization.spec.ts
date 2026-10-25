import { expect, test, type Page } from '@playwright/test'
import { AnglesData, Message, ModeData, ModesEnum } from '../../src/lib/platform_shared/message'
import { MotionModes, modeToWire } from '../../src/lib/motion'
import { FakeRobot, serveRobot } from '../fake-robot/playwright'

const base = process.env.BASE_PATH ?? ''

// The preview origin is http, so the layout opens ws://<host>/api/ws by itself; the route stands in for the robot and
// keeps every mode the app sends it.
test('the 3D view has no GUI panel and mode changes use the controller', async ({ page }) => {
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
  await expect(page.locator('canvas').first()).toBeVisible()
  await expect(page.locator('.lil-gui')).toHaveCount(0)
  await page.getByRole('combobox', { name: 'Mode', exact: true }).selectOption('stand')

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
  const robot = new FakeRobot()
  robot.set(ModeData, ModeData.create({ mode: ModesEnum.STAND }))
  robot.set(AnglesData, AnglesData.create({ angles: Array(18).fill(0) }))
  await serveRobot(page, robot)
  await countDraws(page)
  await page.addInitScript(() =>
    localStorage.setItem('selected_view', JSON.stringify('3D representation'))
  )

  await page.goto(`${base}/controller`)
  await expect.poll(() => draws(page)).toBeGreaterThan(0)
  await expect.poll(() => drawsWithin(page, 1000), { timeout: 15_000 }).toBe(0)

  // A new reported pose moves the model, which draws again and then settles.
  const settled = await draws(page)
  robot.set(AnglesData, AnglesData.create({ angles: Array(18).fill(30) }))
  await expect.poll(() => draws(page)).toBeGreaterThan(settled)
  await expect.poll(() => drawsWithin(page, 1000), { timeout: 15_000 }).toBe(0)
})
