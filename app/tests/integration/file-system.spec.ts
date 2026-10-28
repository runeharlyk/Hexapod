import { expect, test, type Page } from '@playwright/test'
import { serveStockRobot } from '../fake-robot/stock'

const base = process.env.BASE_PATH ?? ''

const folder = (page: Page, name: string) => page.getByRole('button', { name, exact: true })

const file = folder

const listed = (robot: Awaited<ReturnType<typeof serveStockRobot>>['robot']) =>
  robot.requests('fileList').map(r => r.fileList!.path)

const create = async (page: Page, button: 'New file' | 'New folder', name: string) => {
  await page.getByRole('button', { name: button }).click()
  await page.getByPlaceholder(button === 'New file' ? 'File name' : 'Folder name').fill(name)
  await page.getByRole('button', { name: 'Create' }).click()
  await expect(page.getByRole('dialog')).toHaveCount(0)
}

test('the browser opens a folder only when expanded', async ({ page }) => {
  const { robot } = await serveStockRobot(page)
  await page.goto(`${base}/system/filesystem`)

  await expect(folder(page, 'animations')).toBeVisible()
  expect(listed(robot)).toEqual(['/'])

  await folder(page, 'animations').click()
  await expect(page.getByText('wave.pb')).toBeVisible()
  expect(listed(robot)).toEqual(['/', '/animations'])
})

test('a file is opened, edited and saved', async ({ page }) => {
  const { robot } = await serveStockRobot(page)
  robot.files.put('/notes.txt', 'first line')
  await page.goto(`${base}/system/filesystem`)

  await page.getByRole('button', { name: 'notes.txt', exact: true }).click()
  await expect(page.locator('pre')).toHaveText('first line')

  await page.getByRole('button', { name: 'Edit', exact: true }).click()
  await page.getByRole('textbox').fill('second line, with \u00f6')
  await page.getByRole('button', { name: 'Save', exact: true }).click()

  await expect(page.locator('pre')).toHaveText('second line, with \u00f6')
  expect(robot.files.text('/notes.txt')).toBe('second line, with \u00f6')
})

test('a file over a few chunks shows its progress while it is read', async ({ page }) => {
  const { robot } = await serveStockRobot(page)
  const text = 'abcdefghij'.repeat(400)
  robot.files.put('/long.txt', text)
  let release = () => {}
  const held = new Promise<void>(resolve => (release = resolve))
  robot.respond('fileReadChunk', async ({ fileReadChunk: r }) => {
    if (r!.offset === 2048) await held
    return robot.files.read(r!.path, r!.offset, r!.length)
  })
  await page.goto(`${base}/system/filesystem`)

  await page.getByRole('button', { name: 'long.txt', exact: true }).click()
  const progress = page.getByRole('progressbar')
  await expect(progress).toHaveJSProperty('value', 2048)
  await expect(progress).toHaveJSProperty('max', 4000)

  release()
  await expect(page.locator('pre')).toHaveText(text)
  await expect(progress).toHaveCount(0)
})

test('a read that ends after another file was opened leaves that file its progress', async ({
  page
}) => {
  const { robot } = await serveStockRobot(page)
  robot.files.put('/a.txt', 'a'.repeat(4000))
  robot.files.put('/b.txt', 'b'.repeat(5000))
  let releaseA = () => {}
  let releaseB = () => {}
  const heldA = new Promise<void>(resolve => (releaseA = resolve))
  const heldB = new Promise<void>(resolve => (releaseB = resolve))
  robot.respond('fileReadChunk', async ({ fileReadChunk: r }) => {
    if (r!.offset === 2048) await (r!.path === '/a.txt' ? heldA : heldB)
    return robot.files.read(r!.path, r!.offset, r!.length)
  })
  const readOf = (path: string, offset: number) =>
    robot
      .requests('fileReadChunk')
      .some(r => r.fileReadChunk!.path === path && r.fileReadChunk!.offset === offset)
  await page.goto(`${base}/system/filesystem`)
  const progress = page.getByRole('progressbar')

  await file(page, 'a.txt').click()
  await expect(progress).toHaveJSProperty('max', 4000)
  await file(page, 'b.txt').click()
  await expect(progress).toHaveJSProperty('max', 5000)
  await expect(progress).toHaveJSProperty('value', 2048)

  releaseA()
  await expect.poll(() => readOf('/a.txt', 3584)).toBe(true)
  await expect(progress).toHaveJSProperty('max', 5000)
  await expect(progress).toHaveJSProperty('value', 2048)

  releaseB()
  await expect(page.locator('pre')).toHaveText('b'.repeat(5000))
  await expect(progress).toHaveCount(0)
})

test('a file that is not UTF-8 text shows its size and cannot be edited', async ({ page }) => {
  const { robot } = await serveStockRobot(page)
  robot.files.put('/blob.bin', Uint8Array.from([0x41, 0xff, 0x00, 0x80, 0x42]))
  await page.goto(`${base}/system/filesystem`)

  await file(page, 'blob.bin').click()

  await expect(
    page.getByText('5 bytes that are not UTF-8 text, which this page cannot show or edit')
  ).toBeVisible()
  await expect(page.getByRole('button', { name: 'Edit', exact: true })).toHaveCount(0)
  await expect(page.getByRole('button', { name: 'Delete', exact: true })).toBeVisible()
})

test('Save waits for a change, and a saved file keeps its byte order mark', async ({ page }) => {
  const { robot } = await serveStockRobot(page)
  const bom = [0xef, 0xbb, 0xbf]
  robot.files.put('/bom.txt', Uint8Array.from([...bom, ...new TextEncoder().encode('alpha')]))
  await page.goto(`${base}/system/filesystem`)
  const save = page.getByRole('button', { name: 'Save', exact: true })
  const editor = page.getByRole('textbox')

  await file(page, 'bom.txt').click()
  await page.getByRole('button', { name: 'Edit', exact: true }).click()
  await expect(save).toBeDisabled()
  await editor.press('Control+End')
  await editor.pressSequentially('!')
  await expect(save).toBeEnabled()
  await editor.press('Backspace')
  await expect(save).toBeDisabled()
  await editor.pressSequentially('?')
  await save.click()

  await expect(editor).toHaveCount(0)
  expect([...robot.files.bytes('/bom.txt')]).toEqual([
    ...bom,
    ...new TextEncoder().encode('alpha?')
  ])
})

test('a folder is created and a file deleted', async ({ page }) => {
  const { robot } = await serveStockRobot(page)
  page.on('dialog', dialog => dialog.accept())
  await page.goto(`${base}/system/filesystem`)

  await create(page, 'New folder', 'logs')
  await expect(folder(page, 'logs')).toBeVisible()
  expect(robot.files.isDirectory('/logs')).toBe(true)

  await folder(page, 'animations').click()
  await expect(page.getByText('wave.pb')).toBeVisible()
  await page.getByRole('button', { name: 'Delete wave.pb' }).click()

  await expect(page.getByText('wave.pb')).toHaveCount(0)
  await expect(page.getByText('stretch.pb')).toBeVisible()
  expect(robot.files.has('/animations/wave.pb')).toBe(false)
})

test('a file that is slow or fails to read cannot be edited, so it never takes the previous text', async ({
  page
}) => {
  const { robot } = await serveStockRobot(page)
  robot.files.put('/a.txt', 'alpha')
  robot.files.put('/b.txt', 'beta')
  robot.files.put('/c.txt', 'gamma')
  let release = () => {}
  const held = new Promise<void>(resolve => (release = resolve))
  robot.respond('fileReadChunk', async ({ fileReadChunk: r }) => {
    if (r!.path === '/b.txt') await held
    if (r!.path === '/c.txt') return { statusCode: 500, errorMessage: 'flash read failed' }
    return robot.files.read(r!.path, r!.offset, r!.length)
  })
  await page.goto(`${base}/system/filesystem`)
  const edit = page.getByRole('button', { name: 'Edit', exact: true })

  await file(page, 'a.txt').click()
  await expect(page.locator('pre')).toHaveText('alpha')
  await file(page, 'b.txt').click()
  await expect(page.getByRole('status', { name: 'Reading /b.txt' })).toBeVisible()
  await expect(edit).toHaveCount(0)
  release()
  await expect(page.locator('pre')).toHaveText('beta')
  await expect(edit).toBeVisible()

  await file(page, 'c.txt').click()
  await expect(page.getByRole('alert')).toContainText('Could not read /c.txt: flash read failed')
  await expect(edit).toHaveCount(0)
  await expect(page.getByRole('button', { name: 'Delete', exact: true })).toBeVisible()
  expect(robot.requests('fileWriteChunk')).toHaveLength(0)
})

test('a new file does not overwrite one that is there', async ({ page }) => {
  const { robot } = await serveStockRobot(page)
  robot.files.put('/notes.txt', 'keep me')
  await page.goto(`${base}/system/filesystem`)

  await create(page, 'New file', 'notes.txt')
  await expect(page.getByText('notes.txt already exists')).toBeVisible()
  await create(page, 'New file', 'animations/wave.pb')
  await expect(page.getByText('animations/wave.pb already exists')).toBeVisible()

  expect(robot.requests('fileWriteChunk')).toHaveLength(0)
  expect(robot.files.text('/notes.txt')).toBe('keep me')
  expect(robot.files.bytes('/animations/wave.pb')).toHaveLength(2210)
})

test('a new file is listed in the folder it was made in', async ({ page }) => {
  const { robot } = await serveStockRobot(page)
  await page.goto(`${base}/system/filesystem`)
  await folder(page, 'animations').click()
  await expect(file(page, 'wave.pb')).toBeVisible()

  await create(page, 'New file', 'animations/notes.txt')

  await expect(file(page, 'notes.txt')).toBeVisible()
  await expect(file(page, 'wave.pb')).toBeVisible()
  expect(robot.files.text('/animations/notes.txt')).toBe('')
})

test('switching file or cancelling asks before it drops an edit', async ({ page }) => {
  const { robot } = await serveStockRobot(page)
  robot.files.put('/a.txt', 'alpha')
  robot.files.put('/b.txt', 'beta')
  await page.goto(`${base}/system/filesystem`)
  const edit = page.getByRole('button', { name: 'Edit', exact: true })
  const cancel = page.getByRole('button', { name: 'Cancel', exact: true })
  const prompt = page.getByRole('dialog').filter({ hasText: 'Discard changes' })

  await file(page, 'a.txt').click()
  await edit.click()
  await cancel.click()
  await expect(page.locator('pre')).toHaveText('alpha')

  await edit.click()
  await page.getByRole('textbox').fill('changed')
  await file(page, 'b.txt').click()
  await expect(prompt).toBeVisible()
  await prompt.getByRole('button', { name: 'Keep' }).click()
  await expect(prompt).toHaveCount(0)
  await expect(page.getByRole('textbox')).toHaveValue('changed')

  await cancel.click()
  await prompt.getByRole('button', { name: 'Keep' }).click()
  await expect(prompt).toHaveCount(0)
  await expect(page.getByRole('textbox')).toHaveValue('changed')

  await cancel.click()
  await prompt.getByRole('button', { name: 'Discard' }).click()
  await expect(prompt).toHaveCount(0)
  await expect(page.locator('pre')).toHaveText('alpha')

  await edit.click()
  await page.getByRole('textbox').fill('changed again')
  await file(page, 'b.txt').click()
  await prompt.getByRole('button', { name: 'Discard' }).click()
  await expect(prompt).toHaveCount(0)
  await expect(page.locator('pre')).toHaveText('beta')
  expect(robot.files.text('/a.txt')).toBe('alpha')
})

test('leaving the page asks while an edit is unsaved', async ({ page }) => {
  const { robot } = await serveStockRobot(page)
  robot.files.put('/a.txt', 'alpha')
  await page.goto(`${base}/system/filesystem`)
  const prompt = page.getByRole('dialog').filter({ hasText: 'Unsaved changes' })
  const openConnectionPage = async () => {
    await page.locator('label.drawer-button').click()
    await page.getByRole('link', { name: 'Connection', exact: true }).click()
  }
  await file(page, 'a.txt').click()
  await page.getByRole('button', { name: 'Edit', exact: true }).click()

  await page.getByRole('textbox').fill('changed')
  await openConnectionPage()

  await expect(prompt).toBeVisible()
  await prompt.getByRole('button', { name: 'Stay' }).click()
  await expect(page).toHaveURL(new RegExp(`${base}/system/filesystem$`))
  await expect(page.getByRole('textbox')).toHaveValue('changed')

  await page.getByRole('textbox').fill('alpha')
  await openConnectionPage()
  await expect(page).toHaveURL(new RegExp(`${base}/connection$`))
})

test('a save that ends after another file was opened leaves that file alone', async ({ page }) => {
  const { robot } = await serveStockRobot(page)
  robot.files.put('/a.txt', 'alpha')
  robot.files.put('/b.txt', 'beta')
  let release = () => {}
  const held = new Promise<void>(resolve => (release = resolve))
  robot.respond('fileWriteChunk', async ({ fileWriteChunk: w }) => {
    await held
    return robot.files.write(w!.path, w!.offset, w!.totalSize, w!.content)
  })
  await page.goto(`${base}/system/filesystem`)
  const prompt = page.getByRole('dialog').filter({ hasText: 'Discard changes' })

  await file(page, 'a.txt').click()
  await page.getByRole('button', { name: 'Edit', exact: true }).click()
  await page.getByRole('textbox').fill('changed')
  await page.getByRole('button', { name: 'Save', exact: true }).click()
  await file(page, 'b.txt').click()
  await prompt.getByRole('button', { name: 'Discard' }).click()
  await expect(prompt).toHaveCount(0)
  await expect(page.locator('pre')).toHaveText('beta')
  await page.getByRole('button', { name: 'Edit', exact: true }).click()

  release()

  await expect.poll(() => robot.files.text('/a.txt')).toBe('changed')
  await expect(page.getByRole('textbox')).toHaveValue('beta')
})

test('creating a file keeps the open folders open', async ({ page }) => {
  const { robot } = await serveStockRobot(page)
  await page.goto(`${base}/system/filesystem`)

  await folder(page, 'animations').click()
  await expect(page.getByText('wave.pb')).toBeVisible()

  await create(page, 'New file', 'notes.txt')

  await expect(page.getByText('notes.txt').first()).toBeVisible()
  await expect(page.getByText('wave.pb')).toBeVisible()
  expect(robot.files.text('/notes.txt')).toBe('')
})

test('config is refused', async ({ page }) => {
  const { robot } = await serveStockRobot(page)
  await page.goto(`${base}/system/filesystem`)

  await expect(folder(page, 'animations')).toBeVisible()
  await expect(folder(page, 'config')).toHaveCount(0)

  await create(page, 'New file', 'config/wifiSettings.pb')

  await expect(page.getByText(/Could not create config\/wifiSettings\.pb: .*refused/)).toBeVisible()
  expect(robot.requests('fileWriteChunk')).toHaveLength(0)
  expect(robot.files.bytes('/config/wifiSettings.pb')).toHaveLength(112)
})

test('the browser works with no HTTP route served', async ({ page }) => {
  const http: string[] = []
  page.on('request', request => {
    if (new URL(request.url()).pathname.includes('/api/')) http.push(request.url())
  })
  const { robot } = await serveStockRobot(page)
  robot.files.put('/animations/readme.txt', 'over the socket')
  await page.goto(`${base}/system/filesystem`)

  await folder(page, 'animations').click()
  await page.getByRole('button', { name: 'readme.txt', exact: true }).click()
  await expect(page.locator('pre')).toHaveText('over the socket')

  expect(http).toEqual([])
})
