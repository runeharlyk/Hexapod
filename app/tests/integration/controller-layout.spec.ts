import { expect, test, type Page } from '@playwright/test'
import { MotionModes, modeToWire } from '../../src/lib/motion'
import { AnglesData, ModeData } from '../../src/lib/platform_shared/message'
import { FakeRobot, serveRobot } from '../fake-robot/playwright'

const base = process.env.BASE_PATH ?? ''

const expectControlsInViewport = async (page: Page) => {
  await expect(page.getByRole('combobox', { name: 'Mode', exact: true })).toBeInViewport({
    ratio: 1
  })
  await expect(page.getByRole('button', { name: 'Deactivate', exact: true })).toBeInViewport({
    ratio: 1
  })
  await expect(page.locator('.navbar')).toBeInViewport({ ratio: 1 })
  const adjustments = page.getByRole('button', { name: 'Adjustments', exact: true })
  if (await adjustments.count()) {
    await expect(adjustments).toBeInViewport({ ratio: 1 })
    const deactivateBox = await page
      .getByRole('button', { name: 'Deactivate', exact: true })
      .boundingBox()
    const adjustmentsBox = await adjustments.boundingBox()
    expect(adjustmentsBox!.x).toBeGreaterThanOrEqual(deactivateBox!.x + deactivateBox!.width)
    expect(adjustmentsBox!.y).toBe(deactivateBox!.y)
  }
  expect(
    await page.evaluate(() => ({
      width: document.documentElement.scrollWidth - document.documentElement.clientWidth,
      height: document.documentElement.scrollHeight - document.documentElement.clientHeight
    }))
  ).toEqual({ width: 0, height: 0 })
}

for (const viewport of [
  { width: 390, height: 700 },
  { width: 320, height: 480 },
  { width: 700, height: 320 }
]) {
  test(`phone controls stay visible at ${viewport.width}x${viewport.height} through mode changes`, async ({
    page
  }) => {
    const robot = new FakeRobot()
    robot.set(AnglesData, AnglesData.create({ angles: Array(18).fill(0) }))
    await serveRobot(page, robot)
    await page.setViewportSize({ width: 390, height: 700 })
    await page.goto(`${base}/controller`)
    await expect(page.getByRole('combobox', { name: 'Mode', exact: true })).toBeVisible()
    await page.setViewportSize(viewport)

    for (const mode of Object.values(MotionModes)) {
      robot.set(ModeData, ModeData.create({ mode: modeToWire(mode) }))
      await expect(page.getByRole('combobox', { name: 'Mode', exact: true })).toHaveValue(mode)
      await expectControlsInViewport(page)
      if (mode === MotionModes.WALK || mode === MotionModes.STAND) {
        await page.getByRole('button', { name: 'Adjustments', exact: true }).click()
        await expect(page.getByLabel('Body height', { exact: true })).toBeInViewport({ ratio: 1 })
        await expectControlsInViewport(page)
        await page.getByRole('button', { name: 'Adjustments', exact: true }).click()
      }
    }
  })
}

test('walking keeps speed and gait available while other sliders expand on demand', async ({
  page
}) => {
  const robot = new FakeRobot()
  robot.set(ModeData, ModeData.create({ mode: modeToWire(MotionModes.WALK) }))
  await serveRobot(page, robot)
  await page.goto(`${base}/controller`)

  await expect(page.getByLabel('Speed', { exact: true })).toBeVisible()
  await expect(page.getByRole('combobox', { name: 'Gait', exact: true })).toBeVisible()
  await expect(page.getByLabel('Body height', { exact: true })).toBeHidden()
  await expect(page.getByLabel('Step height', { exact: true })).toBeHidden()
  await page.getByText('Adjustments', { exact: true }).click()
  await expect(page.getByLabel('Body height', { exact: true })).toBeVisible()
  await expect(page.getByLabel('Step height', { exact: true })).toBeVisible()
})

test('mode selection sends a request and displays the confirmed state; deactivate remains direct', async ({
  page
}) => {
  const robot = new FakeRobot()
  robot.set(ModeData, ModeData.create({ mode: modeToWire(MotionModes.WALK) }))
  await serveRobot(page, robot)
  await page.goto(`${base}/controller`)
  const mode = page.getByRole('combobox', { name: 'Mode', exact: true })
  await expect(mode).toHaveValue(MotionModes.WALK)

  await mode.selectOption(MotionModes.STAND)
  await expect
    .poll(() => robot.received.flatMap(frame => (frame.mode ? [frame.mode.mode] : [])))
    .toEqual([modeToWire(MotionModes.STAND)])
  await expect(mode).toHaveValue(MotionModes.WALK)
  robot.set(ModeData, ModeData.create({ mode: modeToWire(MotionModes.STAND) }))
  await expect(mode).toHaveValue(MotionModes.STAND)

  await page.getByRole('button', { name: 'Deactivate', exact: true }).click()
  await expect
    .poll(() => robot.received.flatMap(frame => (frame.mode ? [frame.mode.mode] : [])))
    .toEqual([modeToWire(MotionModes.STAND), modeToWire(MotionModes.DEACTIVATED)])
})

test('labelled joystick rings remain centred and send a movement followed by a stop', async ({
  page
}, testInfo) => {
  const robot = new FakeRobot()
  robot.set(ModeData, ModeData.create({ mode: modeToWire(MotionModes.STAND) }))
  robot.set(AnglesData, AnglesData.create({ angles: Array(18).fill(0) }))
  await serveRobot(page, robot)
  await page.setViewportSize({ width: 390, height: 700 })
  await page.goto(`${base}/controller`)
  await expect(page.locator('#left .joystick-label')).toHaveText('Shift')
  await expect(page.locator('#right .joystick-label')).toHaveText('Tilt')
  for (const side of ['left', 'right']) {
    const ring = page.locator(`#${side} .front`)
    await expect(ring).toBeInViewport({ ratio: 1 })
    const box = await ring.boundingBox()
    expect(box!.y + box!.height / 2).toBe(350)
  }
  for (const theme of ['light', 'dark'] as const) {
    await page.emulateMedia({ colorScheme: theme })
    await page.screenshot({ path: testInfo.outputPath(`joystick-rings-${theme}.png`) })
  }
  robot.set(ModeData, ModeData.create({ mode: modeToWire(MotionModes.WALK) }))
  await expect(page.locator('#left .joystick-label')).toHaveText('Move')
  await expect(page.locator('#right .joystick-label')).toHaveText('Turn')
  const box = await page.locator('#left .front').boundingBox()
  const x = box!.x + box!.width / 2
  const y = box!.y + box!.height / 2
  await page.mouse.move(x, y)
  await page.mouse.down()
  await page.mouse.move(x, y - 45, { steps: 2 })
  await expect
    .poll(() => robot.received.some(frame => (frame.input?.left?.y ?? 0) > 0.5))
    .toBe(true)
  await page.mouse.up()
  await expect
    .poll(() => robot.received.filter(frame => frame.input).at(-1)?.input?.left?.y)
    .toBe(0)
})
