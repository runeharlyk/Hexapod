import { test } from '@playwright/test'
import path from 'path'
import { AnalyticsData, IMUData } from '../../src/lib/platform_shared/message'
import { analyticsAt, imuAt, serveStockRobot } from '../fake-robot/stock'

// SHOT_DIR is where the PNGs go, SHOT_LABEL the prefix that tells one capture from another (before, after).
const dir = process.env.SHOT_DIR ?? 'test-results/screenshots'
const label = process.env.SHOT_LABEL ?? 'shot'
const only = process.env.SHOT_PAGES?.split(',')
// The app follows the system's colour scheme; SHOT_THEMES lists the ones to capture.
const themes = (process.env.SHOT_THEMES ?? 'light').split(',') as ('light' | 'dark')[]

const base = process.env.BASE_PATH ?? ''

const VIEWPORTS = {
  phone: { width: 390, height: 844 },
  tablet: { width: 820, height: 1180 },
  desktop: { width: 1440, height: 900 }
}

// Each settings-style page, by the name its file takes, with the stream frames its charts need.
const PAGES: { name: string; path: string; streams?: boolean }[] = [
  { name: 'servo', path: '/peripherals/servo' },
  { name: 'imu', path: '/peripherals/imu', streams: true },
  { name: 'camera', path: '/peripherals/camera' },
  { name: 'i2c', path: '/peripherals/i2c' },
  { name: 'wifi-sta', path: '/wifi/sta' },
  { name: 'wifi-ap', path: '/wifi/ap' },
  { name: 'wifi-mdns', path: '/wifi/mdns' },
  { name: 'system-status', path: '/system/status' },
  { name: 'system-filesystem', path: '/system/filesystem' },
  { name: 'system-configuration', path: '/system/configuration' },
  { name: 'system-metrics', path: '/system/metrics', streams: true },
  { name: 'system-update', path: '/system/update' },
  { name: 'bluetooth', path: '/bluetooth' },
  { name: 'connection', path: '/connection' },
  { name: 'setup', path: '/setup' }
]

for (const shown of PAGES.filter(p => !only || only.includes(p.name)))
  for (const [width, viewport] of Object.entries(VIEWPORTS))
    for (const theme of themes)
      test(`${shown.name} at ${width} in ${theme}`, async ({ page }) => {
        await page.setViewportSize(viewport)
        await page.emulateMedia({ colorScheme: theme })
        const { robot } = await serveStockRobot(page)
        await page.goto(`${base}${shown.path}`)
        await page.waitForLoadState('networkidle')
        if (shown.streams)
          for (let tick = 0; tick < 40; tick++) {
            robot.push(AnalyticsData, analyticsAt(tick))
            robot.push(IMUData, imuAt(tick))
          }
        // The pages slide their sections in and Chart.js eases its lines; both settle well within this.
        await page.waitForTimeout(1500)
        await page.screenshot({
          path: path.join(dir, `${label}-${shown.name}-${width}-${theme}.png`),
          fullPage: true
        })
      })
