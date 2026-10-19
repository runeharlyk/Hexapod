import { expect, test } from '@playwright/test'
import { fakeRobot, tooOld } from './fake-robot'
import { FeaturesDataResponse } from '../../src/lib/platform_shared/message'

const base = process.env.BASE_PATH ?? ''

const features = FeaturesDataResponse.create({ firmwareName: 'hexapod', imu: true, servo: true })

// A robot a WiFi or Bluetooth hop away, whose features arrive after the page has started.
const FEATURES_DELAY_MS = 1000

let featuresSent: Promise<void>

// A reload or a link straight to a feature's page starts with the flags unknown, which reads as every feature off
// until the robot's list arrives; the page has to wait for the list rather than decide on that default.
test.beforeEach(async ({ page }) => {
  let sent: () => void
  featuresSent = new Promise(resolve => (sent = resolve))
  await fakeRobot(page, async request => {
    if (!request.featuresDataRequest) return tooOld()
    await new Promise(resolve => setTimeout(resolve, FEATURES_DELAY_MS))
    sent()
    return { featuresDataResponse: features }
  })
})

test('a link straight to the metrics page stays there', async ({ page }) => {
  await page.goto(`${base}/system/metrics`)
  await featuresSent
  await expect(page.getByText('Telemetry recording', { exact: true })).toBeVisible()
  expect(new URL(page.url()).pathname).toBe(`${base}/system/metrics`)
})

test('a link straight to the IMU page draws its chart', async ({ page }) => {
  await page.goto(`${base}/peripherals/imu`)
  // Chart.js sizes the canvas it draws on; a chart made before its canvas existed leaves the canvas as the browser
  // made it.
  await expect(page.locator('canvas').first()).toHaveAttribute('style', /display: block/)
})
