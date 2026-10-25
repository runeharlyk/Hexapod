import { expect, test } from '@playwright/test'
import { ModeData, ModesEnum } from '../../src/lib/platform_shared/message'
import { FakeRobot, serveRobot } from '../fake-robot/playwright'

const base = process.env.BASE_PATH ?? ''

test.beforeEach(async ({ page }) => {
  await page.route('**/mujoco.wasm', () => {})
})

test('selecting Simulation takes the controls before its view module loads', async ({ page }) => {
  await page.setViewportSize({ width: 320, height: 480 })
  await page.routeWebSocket('**/api/ws', socket => socket.close())
  await page.goto(`${base}/controller`)
  await expect(page.getByText('Preview', { exact: true })).toBeVisible()
  await page.route('**/*.js', () => {})
  await page.getByRole('combobox', { name: 'View', exact: true }).selectOption('Simulation')
  await expect(page.getByText('Simulation only', { exact: true })).toBeVisible()
  await expect(page.getByRole('button', { name: 'Deactivate', exact: true })).toHaveCount(0)
  const mode = page.getByRole('combobox', { name: 'Mode', exact: true })
  await mode.selectOption('walk')
  await expect(mode).toHaveValue('walk')
  await page.getByRole('combobox', { name: 'Gait', exact: true }).selectOption('Wave')
  await expect(page.getByRole('combobox', { name: 'Gait', exact: true })).toHaveValue('Wave')
  const badge = page.getByText('Simulation only', { exact: true })
  await expect(badge).toBeInViewport({ ratio: 1 })
  await expect(mode).toBeInViewport({ ratio: 1 })
  const modeBox = await mode.boundingBox()
  const badgeBox = await badge.boundingBox()
  expect(badgeBox!.y + badgeBox!.height / 2).toBe(modeBox!.y + modeBox!.height / 2)
})

test('simulation mode changes work without a connected robot while physics loads', async ({
  page
}) => {
  await page.routeWebSocket('**/api/ws', socket => socket.close())
  await page.addInitScript(() =>
    localStorage.setItem('selected_view', JSON.stringify('Simulation'))
  )
  await page.goto(`${base}/controller`)
  await expect(page.getByText('Simulation only', { exact: true })).toBeVisible()
  const mode = page.getByRole('combobox', { name: 'Mode', exact: true })
  for (const next of ['stand', 'walk', 'deactivated']) {
    await mode.selectOption(next)
    await expect(mode).toHaveValue(next)
  }
  await expect(page.getByText(/Mode not changed/)).toHaveCount(0)
})

test('simulation keeps its mode separate from the connected robot across view changes', async ({
  page
}) => {
  const robot = new FakeRobot()
  robot.set(ModeData, ModeData.create({ mode: ModesEnum.STAND }))
  await serveRobot(page, robot)
  await page.goto(`${base}/controller`)
  const view = page.getByRole('combobox', { name: 'View', exact: true })
  const mode = page.getByRole('combobox', { name: 'Mode', exact: true })
  await expect(mode).toHaveValue('stand')
  await view.selectOption('Simulation')
  await expect(page.getByText('Simulation only', { exact: true })).toBeVisible()
  await mode.selectOption('walk')
  await expect(mode).toHaveValue('walk')
  await page.getByRole('combobox', { name: 'Gait', exact: true }).selectOption('Wave')
  await expect(page.getByRole('combobox', { name: 'Gait', exact: true })).toHaveValue('Wave')
  robot.set(ModeData, ModeData.create({ mode: ModesEnum.IDLE }))
  await expect(mode).toHaveValue('walk')
  await view.selectOption('Stream')
  await expect(mode).toHaveValue('idle')
  await view.selectOption('Simulation')
  await expect(mode).toHaveValue('walk')
  await expect(page.getByRole('combobox', { name: 'Gait', exact: true })).toHaveValue('Wave')
  expect(robot.received.filter(frame => frame.mode)).toHaveLength(0)
})

test('offline Simulation plays and stops bundled animations', async ({ page }) => {
  await page.unroute('**/mujoco.wasm')
  await page.setViewportSize({ width: 390, height: 700 })
  await page.routeWebSocket('**/api/ws', socket => socket.close())
  await page.addInitScript(() =>
    localStorage.setItem('selected_view', JSON.stringify('Simulation'))
  )
  const errors: string[] = []
  page.on('pageerror', error => errors.push(error.message))
  await page.goto(`${base}/controller`)
  await page.getByRole('combobox', { name: 'Mode', exact: true }).selectOption('animate')
  await page.getByRole('combobox', { name: 'Animation', exact: true }).selectOption('wave')
  const play = page.getByRole('button', { name: 'Play', exact: true })
  await expect(play).toBeEnabled({ timeout: 15000 })
  await play.click()
  await expect(page.getByText('Playing wave', { exact: true })).toBeVisible()
  await page.getByRole('button', { name: 'Stop', exact: true }).click()
  await expect(page.getByText('Playing wave', { exact: true })).toHaveCount(0)
  await expect(page.getByText(/not connected/)).toHaveCount(0)
  expect(errors).toEqual([])
})
