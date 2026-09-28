import { expect, test } from '@playwright/test'

test('landing page offers to add a robot', async ({ page }) => {
  await page.goto('/')
  await expect(page).toHaveTitle('Add robot')
  await expect(page.getByRole('button', { name: 'Add robot' }).first()).toBeVisible()
})

test('controller page renders its mode controls without a robot', async ({ page }) => {
  const errors: string[] = []
  page.on('pageerror', error => errors.push(error.message))

  await page.goto('/controller')
  await expect(page).toHaveTitle('Controller')
  await expect(page.getByRole('button', { name: 'Deactivated' })).toBeVisible()
  await expect(page.getByText('Preview')).toBeVisible()
  expect(errors).toEqual([])
})
