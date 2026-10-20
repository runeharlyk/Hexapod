import { expect, test, type Page } from '@playwright/test'
import { serveStockRobot } from '../fake-robot/stock'

// The Wi-Fi station page's network editor is a modal dialog: it takes focus, keeps it from the page behind, closes on
// Escape without applying, and gives focus back to the button that opened it.

const base = process.env.BASE_PATH ?? ''

const dialog = (page: Page) => page.getByRole('dialog')
const addButton = (page: Page) => page.getByRole('button', { name: 'Add', exact: true })

// Where focus is: in the dialog, on the page behind it, or on neither. A modal <dialog> lets Tab past its last control
// leave the page for the browser's own controls, which a headless browser reports as the body; it never reaches the
// page.
const focus = (page: Page) =>
  page.evaluate(() => {
    const focused = document.activeElement
    if (focused?.closest('dialog, [role="dialog"]')) return 'dialog'
    return !focused || focused === document.body ? 'browser' : 'page'
  })

const openAdd = async (page: Page) => {
  await serveStockRobot(page)
  await page.goto(`${base}/wifi/sta`)
  await addButton(page).click()
  await expect(dialog(page)).toBeVisible()
}

test('the dialog takes focus on its first field', async ({ page }) => {
  await openAdd(page)
  await expect(dialog(page).getByLabel('SSID')).toBeFocused()
})

test('Escape closes the dialog without applying it', async ({ page }) => {
  await openAdd(page)
  await dialog(page).getByLabel('SSID').fill('garage')

  await page.keyboard.press('Escape')

  await expect(dialog(page)).toHaveCount(0)
  await expect(page.getByText('garage', { exact: true })).toHaveCount(0)
  await expect(page.getByRole('button', { name: 'Save', exact: true })).toBeDisabled()
})

test('Shift+Tab from the first field never reaches the page behind the dialog', async ({
  page
}) => {
  await openAdd(page)
  await dialog(page).getByLabel('SSID').click()

  const visited = new Set<string>()
  for (let press = 0; press < 12; press++) {
    await page.keyboard.press('Shift+Tab')
    visited.add(await focus(page))
  }
  expect(visited).not.toContain('page')
  expect(visited).toContain('dialog')
})

test('Tab never reaches the page or the backdrop behind the dialog', async ({ page }) => {
  await openAdd(page)

  const visited = new Set<string>()
  for (let press = 0; press < 12; press++) {
    await page.keyboard.press('Tab')
    visited.add(await focus(page))
  }
  expect(visited).not.toContain('page')
  expect(visited).toContain('dialog')
})

test('closing the dialog gives focus back to the button that opened it', async ({ page }) => {
  await openAdd(page)
  await page.keyboard.press('Escape')
  await expect(dialog(page)).toHaveCount(0)
  await expect(addButton(page)).toBeFocused()

  await page.getByRole('button', { name: 'Edit home-network' }).click()
  await dialog(page).getByRole('button', { name: 'Cancel' }).click()
  await expect(dialog(page)).toHaveCount(0)
  await expect(page.getByRole('button', { name: 'Edit home-network' })).toBeFocused()
})
