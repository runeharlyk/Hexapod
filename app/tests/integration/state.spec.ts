import { expect, test } from '@playwright/test'
import { fakeRobot, tooOld } from './fake-robot'

const base = process.env.BASE_PATH ?? ''

// Firmware from before the snapshots answers pings and requests but never acks a subscribe, so no state the app follows
// ever goes live; the layout says why, in the words a request such firmware lacks gets, rather than failing in silence.
test('a firmware without snapshots is named as too old for the live state', async ({ page }) => {
  await fakeRobot(page, tooOld)
  await page.goto(`${base}/controller`)

  await expect(
    page.getByText("Robot state: the robot's firmware does not support live state; update it", {
      exact: true
    })
  ).toBeVisible({ timeout: 15000 })
})
