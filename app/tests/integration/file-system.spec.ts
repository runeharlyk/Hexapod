import { expect, test } from '@playwright/test'
import { serveStockRobot } from '../fake-robot/stock'

const base = process.env.BASE_PATH ?? ''

test('creating a file keeps the open folders open', async ({ page }) => {
  await serveStockRobot(page)
  const tree: Record<string, unknown> = {
    config: { 'wifiSettings.pb': 112 },
    animations: { 'wave.pb': 2210 }
  }
  await page.route('**/api/files', route => route.fulfill({ json: { root: tree } }))
  await page.route('**/api/files/edit', route => {
    const { path } = route.request().postDataJSON() as { path: string }
    tree[path.slice(1)] = 0
    return route.fulfill({ json: {} })
  })
  await page.route('**/api/files/content?**', route => route.fulfill({ body: '' }))
  await page.goto(`${base}/system/filesystem`)

  await page.getByRole('button', { name: 'config', exact: true }).click()
  await expect(page.getByText('wifiSettings.pb')).toBeVisible()

  await page.getByRole('button', { name: 'New file' }).click()
  await page.getByPlaceholder('File name').fill('notes.txt')
  await page.getByRole('button', { name: 'Create' }).click()

  await expect(page.getByText('notes.txt').first()).toBeVisible()
  await expect(page.getByText('wifiSettings.pb')).toBeVisible()
})
