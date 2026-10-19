import { expect, test } from '@playwright/test'
import { fakeRobot, tooOld } from './fake-robot'
import { FeaturesDataResponse } from '../../src/lib/platform_shared/message'

const base = process.env.BASE_PATH ?? ''

const features = FeaturesDataResponse.create({ firmwareName: 'hexapod', imu: true, servo: true })

// A robot a WiFi or Bluetooth hop away, whose features arrive after the page has started.
const FEATURES_DELAY_MS = 1000

// A reload or a link straight to a feature's page starts with the flags unknown, which reads as every feature off
// until the robot's list arrives; the page has to wait for the list rather than decide on that default.
test.describe('with the features late', () => {
  let featuresSent: Promise<void>

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
})

// A read that fails must not leave the page waiting for good: it says why and offers another.
test('a failed read of the features says why and can be retried', async ({ page }) => {
  // The robot's webserver, which the app asks when the link fails, is not there either.
  await page.route('**/api/features', route => route.fulfill({ status: 404 }))
  let featureRequests = 0
  await fakeRobot(page, request => {
    if (!request.featuresDataRequest) return tooOld()
    return ++featureRequests === 1 ?
        { statusCode: 503, errorMessage: 'Busy' }
      : { featuresDataResponse: features }
  })

  await page.goto(`${base}/system/metrics`)
  await expect(
    page.getByText("The robot's features could not be read: Busy", { exact: true })
  ).toBeVisible()
  await page.getByRole('button', { name: 'Retry' }).click()
  await expect(page.getByText('Telemetry recording', { exact: true })).toBeVisible()
})
