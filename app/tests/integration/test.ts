import { expect, test } from '@playwright/test'

// The Pages build serves the app under BASE_PATH (/Hexapod); local builds serve it at the root.
const base = process.env.BASE_PATH ?? ''

test('landing page offers to add a robot', async ({ page }) => {
  await page.goto(`${base}/`)
  await expect(page).toHaveTitle('Add robot')
  await expect(page.getByRole('button', { name: 'Add robot' }).first()).toBeVisible()
})

test('controller page renders its mode controls without a robot', async ({ page }) => {
  const errors: string[] = []
  page.on('pageerror', error => errors.push(error.message))

  await page.goto(`${base}/controller`)
  await expect(page).toHaveTitle('Controller')
  await expect(page.getByRole('button', { name: 'Deactivated' })).toBeVisible()
  await expect(page.getByText('Preview')).toBeVisible()
  expect(errors).toEqual([])
})
