import { expect, test, type Page } from '@playwright/test'
import { PeripheralSettings } from '../../src/lib/platform_shared/api'
import { serveStockRobot, stockRobot, STOCK } from '../fake-robot/stock'

// The shared settings form (lib/components/layout), as the I2C page uses it against the stock robot.

const base = process.env.BASE_PATH ?? ''

const form = (page: Page) =>
  page.locator('form').filter({ has: page.getByRole('heading', { name: 'Bus configuration' }) })
const save = (page: Page) => form(page).getByRole('button', { name: 'Save', exact: true })
const reset = (page: Page) => form(page).getByRole('button', { name: 'Reset', exact: true })
const frequency = (page: Page) => page.getByLabel('Frequency')

test('the form shows a skeleton while the robot reads the settings, then the fields', async ({
  page
}) => {
  const robot = stockRobot()
  let answer: (settings: PeripheralSettings) => void = () => {}
  robot.respond(
    'peripheralSettingsGet',
    () => new Promise(resolve => (answer = settings => resolve({ peripheralSettings: settings })))
  )
  await serveStockRobot(page, robot)
  await page.goto(`${base}/peripherals/i2c`)

  const loading = form(page).getByRole('status', { name: 'Reading the I2C settings' })
  await expect(loading.locator('.skeleton').first()).toBeVisible()
  await expect(frequency(page)).toHaveCount(0)
  await expect(save(page)).toHaveCount(0)

  answer(STOCK.peripheralSettings)

  await expect(frequency(page)).toHaveValue('400000')
  await expect(loading).toHaveCount(0)
})

test('save and reset stay disabled until a field changes, and reset restores the read value', async ({
  page
}) => {
  await serveStockRobot(page)
  await page.goto(`${base}/peripherals/i2c`)
  await expect(frequency(page)).toHaveValue('400000')
  await expect(save(page)).toBeDisabled()
  await expect(reset(page)).toBeDisabled()

  await frequency(page).fill('100000')
  await expect(save(page)).toBeEnabled()
  await expect(reset(page)).toBeEnabled()

  // Typing the read value back is no change.
  await frequency(page).fill('400000')
  await expect(save(page)).toBeDisabled()

  await frequency(page).fill('100000')
  await reset(page).click()
  await expect(frequency(page)).toHaveValue('400000')
  await expect(save(page)).toBeDisabled()
  await expect(reset(page)).toBeDisabled()
})

test('a saved value is sent once confirmed and becomes the one reset returns to', async ({
  page
}) => {
  const { robot } = await serveStockRobot(page)
  await page.goto(`${base}/peripherals/i2c`)
  await frequency(page).fill('100000')
  await save(page).click()
  await page.getByRole('button', { name: 'Confirm' }).click()

  await expect(
    page.getByText('I2C configuration saved - restart the robot to apply it.')
  ).toBeVisible()
  expect(robot.requests('peripheralSettingsUpdate').map(r => r.peripheralSettingsUpdate)).toEqual([
    PeripheralSettings.create({ ...STOCK.peripheralSettings, frequency: 100000, pins: [] })
  ])
  await expect(save(page)).toBeDisabled()
  await expect(frequency(page)).toHaveValue('100000')

  await frequency(page).fill('200000')
  await reset(page).click()
  await expect(frequency(page)).toHaveValue('100000')
})

const errorColour = (page: Page) =>
  page.evaluate(() => {
    const probe = document.createElement('span')
    probe.style.color = 'var(--color-error)'
    document.body.append(probe)
    const colour = getComputedStyle(probe).color
    probe.remove()
    return colour
  })

test('a value outside its constraints turns its hint to the error colour', async ({ page }) => {
  await serveStockRobot(page)
  await page.goto(`${base}/peripherals/i2c`)
  const hint = form(page).getByText('100000 to 400000 Hz', { exact: true })
  await expect(hint).toBeVisible()
  const red = await errorColour(page)
  await expect(hint).not.toHaveCSS('color', red)

  await frequency(page).fill('50')
  await frequency(page).blur()

  await expect(hint).toHaveCSS('color', red)
})

test('a save with an invalid folded field opens its section and sends nothing', async ({
  page
}) => {
  const { robot } = await serveStockRobot(page)
  await page.goto(`${base}/peripherals/i2c`)
  await expect(frequency(page)).toHaveValue('400000')
  const pins = form(page).locator('details')
  await expect(pins).not.toHaveAttribute('open')

  await pins.getByText('Pins', { exact: true }).click()
  await page.getByLabel('SDA').fill('99')
  await pins.getByText('Pins', { exact: true }).click()
  await expect(pins).not.toHaveAttribute('open')
  await save(page).click()

  await expect(pins).toHaveAttribute('open')
  await expect(page.getByLabel('SDA')).toBeFocused()
  await expect(page.getByRole('button', { name: 'Confirm' })).toHaveCount(0)
  expect(robot.requests('peripheralSettingsUpdate')).toEqual([])
})

test('a failed read is shown in the form and read again on retry', async ({ page }) => {
  const robot = stockRobot()
  // The firmware never refuses this read; one that does not know it answers 501.
  robot.refuse('peripheralSettingsGet', 501)
  await serveStockRobot(page, robot)
  await page.goto(`${base}/peripherals/i2c`)

  const failure = form(page).getByRole('alert')
  await expect(failure).toContainText(
    "Could not read the I2C settings: the robot's firmware does not support the I2C settings; update it"
  )
  robot.respond('peripheralSettingsGet', () => ({ peripheralSettings: STOCK.peripheralSettings }))
  await failure.getByRole('button', { name: 'Retry' }).click()

  await expect(frequency(page)).toHaveValue('400000')
  await expect(failure).toHaveCount(0)
})
