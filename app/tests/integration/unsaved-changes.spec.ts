import { expect, test, type Page } from '@playwright/test'
import { serveStockRobot } from '../fake-robot/stock'

// Leaving a settings page with changes that are not saved asks first: the app's own dialog for a move inside the app,
// the browser's prompt for a reload or a closed tab.

const base = process.env.BASE_PATH ?? ''

const save = (page: Page) => page.getByRole('button', { name: 'Save', exact: true })
// The way a person moves on: through the menu, as a link the app routes itself.
const openConnectionPage = async (page: Page) => {
  await page.locator('label.drawer-button').click()
  await page.getByRole('link', { name: 'Connection', exact: true }).click()
}
const prompt = (page: Page) => page.getByRole('dialog').filter({ hasText: 'Unsaved changes' })

const hostName = (page: Page) => page.getByLabel('Host name')

test('a dirty form asks before the app moves on, and staying keeps the edit', async ({ page }) => {
  await serveStockRobot(page)
  await page.goto(`${base}/wifi/sta`)
  await hostName(page).fill('hexapod-lab')

  await openConnectionPage(page)

  await expect(prompt(page)).toBeVisible()
  await prompt(page).getByRole('button', { name: 'Stay' }).click()
  await expect(prompt(page)).toHaveCount(0)
  await expect(page).toHaveURL(new RegExp(`${base}/wifi/sta$`))
  await expect(hostName(page)).toHaveValue('hexapod-lab')
  await expect(save(page)).toBeEnabled()
})

test('leaving a dirty form drops the edit', async ({ page }) => {
  await serveStockRobot(page)
  await page.goto(`${base}/wifi/sta`)
  await hostName(page).fill('hexapod-lab')

  await openConnectionPage(page)
  await prompt(page).getByRole('button', { name: 'Leave' }).click()

  await expect(page).toHaveURL(new RegExp(`${base}/connection$`))
  await page.goBack()
  await expect(hostName(page)).toHaveValue('hexapod')
})

test('a network added in its dialog but not saved holds the page', async ({ page }) => {
  await serveStockRobot(page)
  await page.goto(`${base}/wifi/sta`)
  await page.getByRole('button', { name: 'Add', exact: true }).click()
  const dialog = page.getByRole('dialog')
  await dialog.getByLabel('SSID').fill('garage')
  await dialog.getByRole('button', { name: 'Add network' }).click()
  await expect(page.getByText('garage', { exact: true })).toBeVisible()

  await openConnectionPage(page)

  await expect(prompt(page)).toBeVisible()
  await prompt(page).getByRole('button', { name: 'Stay' }).click()
  await expect(page.getByText('garage', { exact: true })).toBeVisible()
})

test('the back and forward buttons ask too, and leaving goes on', async ({ page }) => {
  await serveStockRobot(page)
  await page.goto(`${base}/peripherals/i2c`)
  await expect(page.getByLabel('Frequency')).toHaveValue('400000')
  await openConnectionPage(page)
  await expect(page).toHaveURL(new RegExp(`${base}/connection$`))
  await page.goBack()
  await page.getByLabel('Frequency').fill('100000')

  await page.goForward()

  await expect(prompt(page)).toBeVisible()
  await expect(page).toHaveURL(new RegExp(`${base}/peripherals/i2c$`))
  await expect(page.getByLabel('Frequency')).toHaveValue('100000')
  await prompt(page).getByRole('button', { name: 'Leave' }).click()
  await expect(page).toHaveURL(new RegExp(`${base}/connection$`))
})

test('a clean form lets the app move on without asking', async ({ page }) => {
  await serveStockRobot(page)
  await page.goto(`${base}/wifi/sta`)
  await expect(hostName(page)).toHaveValue('hexapod')

  await openConnectionPage(page)

  await expect(page).toHaveURL(new RegExp(`${base}/connection$`))
  await expect(prompt(page)).toHaveCount(0)
})

test('a saved form is clean again', async ({ page }) => {
  await serveStockRobot(page)
  await page.goto(`${base}/wifi/sta`)
  await hostName(page).fill('hexapod-lab')
  await save(page).click()
  await expect(page.getByText('Wi-Fi settings updated.')).toBeVisible()

  await openConnectionPage(page)

  await expect(page).toHaveURL(new RegExp(`${base}/connection$`))
  await expect(prompt(page)).toHaveCount(0)
})

test('closing the tab with a dirty form leaves the asking to the browser', async ({ page }) => {
  await serveStockRobot(page)
  await page.goto(`${base}/peripherals/i2c`)
  await page.getByLabel('Frequency').fill('100000')

  const asked = page.waitForEvent('dialog')
  await page.close({ runBeforeUnload: true })

  const dialog = await asked
  expect(dialog.type()).toBe('beforeunload')
  await dialog.dismiss()
})

test('closing the tab with a clean form does not ask', async ({ page }) => {
  await serveStockRobot(page)
  await page.goto(`${base}/peripherals/i2c`)
  await expect(page.getByLabel('Frequency')).toHaveValue('400000')
  let asked = false
  page.on('dialog', () => (asked = true))

  await page.close({ runBeforeUnload: true })

  await expect.poll(() => page.isClosed()).toBe(true)
  expect(asked).toBe(false)
})
