import { expect, test, type Page } from '@playwright/test'

const base = process.env.BASE_PATH ?? ''

const pngSize = (bytes: Buffer) => `${bytes.readUInt32BE(16)}x${bytes.readUInt32BE(20)}`

const waitForControllingWorker = async (page: Page) => {
  await page.goto(`${base}/`)
  await page.evaluate(async () => {
    await navigator.serviceWorker.ready
    if (navigator.serviceWorker.controller) return
    await new Promise(resolve =>
      navigator.serviceWorker.addEventListener('controllerchange', resolve, { once: true })
    )
  })
}

test('manifest makes the app installable from its base path', async ({ page, request }) => {
  await page.goto(`${base}/`)
  const href = await page.locator('link[rel="manifest"]').getAttribute('href')
  expect(href).not.toBeNull()
  const manifestUrl = new URL(href!, page.url())

  const response = await request.get(manifestUrl.href)
  expect(response.ok()).toBe(true)
  const manifest = await response.json()

  expect(manifest.name).toBeTruthy()
  expect(manifest.short_name).toBeTruthy()
  expect(['fullscreen', 'standalone', 'minimal-ui']).toContain(manifest.display)
  expect(manifest.prefer_related_applications ?? false).toBe(false)

  const appRoot = new URL(`${base}/`, page.url()).href
  expect(new URL(manifest.start_url, manifestUrl).href).toBe(appRoot)
  expect(new URL(manifest.scope, manifestUrl).href).toBe(appRoot)

  const declaredSizes: string[] = []
  for (const icon of manifest.icons) {
    expect(icon.type).toBe('image/png')
    const image = await request.get(new URL(icon.src, manifestUrl).href)
    expect(image.ok()).toBe(true)
    expect(pngSize(await image.body())).toBe(icon.sizes)
    declaredSizes.push(icon.sizes)
  }
  expect(declaredSizes).toEqual(expect.arrayContaining(['192x192', '512x512']))
})

test('service worker registers for the app scope', async ({ page }) => {
  await waitForControllingWorker(page)
  const registration = await page.evaluate(async () => {
    const ready = await navigator.serviceWorker.ready
    return { scope: ready.scope, script: ready.active?.scriptURL }
  })
  expect(registration.scope).toBe(new URL(`${base}/`, page.url()).href)
  expect(registration.script).toBe(new URL(`${base}/service-worker.js`, page.url()).href)
})

test('cached shell opens every route with the network blocked', async ({ page, context }) => {
  await waitForControllingWorker(page)

  await page.route('**/*', route => route.abort('internetdisconnected'))
  await context.setOffline(true)

  const apiReachedNetwork = await page.evaluate(
    url =>
      fetch(url).then(
        () => true,
        () => false
      ),
    `${base}/api/features`
  )
  expect(apiReachedNetwork).toBe(false)

  const errors: string[] = []
  page.on('pageerror', error => errors.push(error.message))

  const controller = await page.goto(`${base}/controller`)
  expect(controller?.fromServiceWorker()).toBe(true)
  await expect(page).toHaveTitle('Controller')
  await expect(page.getByRole('button', { name: 'Deactivated' })).toBeVisible()

  const reloaded = await page.reload()
  expect(reloaded?.fromServiceWorker()).toBe(true)
  await expect(page).toHaveTitle('Controller')
  expect(errors).toEqual([])
})
