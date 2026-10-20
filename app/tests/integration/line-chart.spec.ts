import { expect, test, type Locator } from '@playwright/test'
import { AnalyticsData } from '../../src/lib/platform_shared/message'
import { analyticsAt, serveStockRobot } from '../fake-robot/stock'

const base = process.env.BASE_PATH ?? ''

// The mean brightness of the legend's text: the opaque grey pixels across the top of the chart, which the coloured
// legend boxes and the faint grid do not count towards.
const legendBrightness = (chart: Locator) =>
  chart.evaluate(canvas => {
    const { width } = canvas as HTMLCanvasElement
    const strip = (canvas as HTMLCanvasElement).getContext('2d')!.getImageData(0, 0, width, 30)
    let sum = 0
    let count = 0
    for (let i = 0; i < strip.data.length; i += 4) {
      const [r, g, b, a] = strip.data.slice(i, i + 4)
      if (a < 200 || Math.max(r, g, b) - Math.min(r, g, b) > 40) continue
      sum += (r + g + b) / 3
      count++
    }
    return count ? sum / count : NaN
  })

// The charts draw their text in the theme's colours, read again when the menu's switch changes the theme.
test('a chart takes the new colours when the theme is switched', async ({ page }) => {
  await page.emulateMedia({ colorScheme: 'dark' })
  const { robot } = await serveStockRobot(page)
  await page.goto(`${base}/system/metrics`)
  for (let tick = 0; tick < 5; tick++) robot.push(AnalyticsData, analyticsAt(tick))
  const chart = page.locator('canvas').first()
  await expect.poll(() => legendBrightness(chart)).toBeGreaterThan(160)

  await page.locator('label.drawer-button').click()
  await page.locator('label', { has: page.locator('.theme-controller') }).click()

  await expect.poll(() => legendBrightness(chart)).toBeLessThan(100)
})
