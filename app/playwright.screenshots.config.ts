import type { PlaywrightTestConfig } from '@playwright/test'
import base from './playwright.config'

// Captures every settings-style page against the stock fake robot (tests/screenshots); not part of `pnpm test`.
const config: PlaywrightTestConfig = {
  ...base,
  testDir: 'tests/screenshots',
  testMatch: /.+\.shots\.ts/
}

export default config
