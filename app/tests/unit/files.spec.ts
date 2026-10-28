// The file client against the stock robot's file tree, which answers as the firmware does (file_transfer.h).
import { beforeEach, describe, expect, it } from 'vitest'
import { DataBroker } from '$lib/transport/databroker'
import { RequestFailed } from '$lib/transport/reply'
import {
  CHUNK,
  childPath,
  deleteFile,
  fileExists,
  listDirectory,
  makeDirectory,
  readFile,
  writeFile
} from '$lib/files'
import { fakeLink } from '../fake-robot/transport'
import { stockRobot, type StockRobot } from '../fake-robot/stock'

let robot: StockRobot
let broker: DataBroker

const bytesOf = (length: number) => Uint8Array.from({ length }, (_, i) => (i * 7 + 3) % 256)

const failure = async (action: Promise<unknown>) => {
  const error = await action.then(
    () => undefined,
    (e: unknown) => e
  )
  expect(error).toBeInstanceOf(RequestFailed)
  return (error as RequestFailed).failure
}

beforeEach(async () => {
  robot = stockRobot()
  broker = new DataBroker()
  const link = fakeLink(robot)
  broker.addTransport(link)
  await link.connect()
})

describe('listDirectory', () => {
  it('lists a directory longer than one page whole, each entry once', async () => {
    for (let i = 0; i < 60; i++) robot.files.put(`/big/file${String(i).padStart(2, '0')}.txt`, 'x')
    robot.files.put('/big/sub/inner.txt', 'x')

    const entries = await listDirectory('/big', broker)

    const names = entries.map(e => e.name)
    expect(names).toHaveLength(61)
    expect(new Set(names).size).toBe(61)
    expect(names[0]).toBe('sub')
    expect(entries[0].isDirectory).toBe(true)
    expect(names.slice(1)).toEqual([...names.slice(1)].sort())
    expect(robot.requests('fileList').map(r => r.fileList!.offset)).toEqual([0, 26, 52])
  })

  it('lists each entry once when one is added before the next page', async () => {
    for (let i = 0; i < 60; i++) robot.files.put(`/busy/file${String(i).padStart(2, '0')}`, 'x')
    robot.respond('fileList', ({ fileList: l }) => {
      if (l!.offset === 26) robot.files.put('/busy/aaa', 'x')
      return robot.files.list(l!.path, l!.offset)
    })

    const names = (await listDirectory('/busy', broker)).map(e => e.name)

    expect(new Set(names).size).toBe(names.length)
    expect(names).toHaveLength(60)
    expect(robot.requests('fileList').map(r => r.fileList!.offset)).toEqual([0, 26, 52])
  })

  it('skips a name over 63 bytes, counted in UTF-8, as the robot does', async () => {
    robot.files.put(`/names/${'o'.repeat(21)}`, 'x')
    robot.files.put(`/names/${'\u00f8'.repeat(32)}`, 'x')
    robot.files.put(`/names/${'\u00f8'.repeat(31)}`, 'x')

    const names = (await listDirectory('/names', broker)).map(e => e.name)

    expect(names).toEqual(['o'.repeat(21), '\u00f8'.repeat(31)])
  })

  it('stops without a further request when the last page ends the directory', async () => {
    for (let i = 0; i < 52; i++) robot.files.put(`/even/f${String(i).padStart(2, '0')}`, 'x')

    expect(await listDirectory('/even', broker)).toHaveLength(52)
    expect(robot.requests('fileList').map(r => r.fileList!.offset)).toEqual([0, 26])
  })

  it('lists an empty directory with one request', async () => {
    await makeDirectory('/empty', broker)

    expect(await listDirectory('/empty', broker)).toEqual([])
    expect(robot.requests('fileList')).toHaveLength(1)
  })

  it('reports the size of a file', async () => {
    expect(await listDirectory('/animations', broker)).toEqual([
      { name: 'stretch.pb', isDirectory: false, size: 1830 },
      { name: 'wave.pb', isDirectory: false, size: 2210 }
    ])
  })

  it('rejects with the refusal for a directory that is not there', async () => {
    expect(await failure(listDirectory('/nothing', broker))).toMatchObject({
      kind: 'refused',
      status: 404
    })
  })

  it('refuses config, which the root does not list either', async () => {
    expect(await failure(listDirectory('/config', broker))).toMatchObject({
      kind: 'refused',
      status: 400,
      message: 'bad path'
    })
    expect((await listDirectory('/', broker)).map(e => e.name)).toEqual(['animations'])
  })

  // Firmware up to v0.1.1 has no listing and answers it with a bare 400, as it does every request it does not know.
  it('says the firmware is too old when the robot answers the listing with a bare 400', async () => {
    robot.respond('fileList', () => ({ statusCode: 400 }))

    await expect(listDirectory('/', broker)).rejects.toThrow(
      "the robot's firmware does not support listing /; update it"
    )
  })
})

describe('fileExists', () => {
  it('tells a file that is there from one that is not, without reading it', async () => {
    expect(await fileExists('/animations/wave.pb', broker)).toBe(true)
    expect(await fileExists('/animations/gone.pb', broker)).toBe(false)
    expect(robot.requests('fileReadChunk').map(r => r.fileReadChunk!.length)).toEqual([0, 0])
  })

  it('rejects when the robot refuses the path', async () => {
    expect(await failure(fileExists('/config/wifiSettings.pb', broker))).toMatchObject({
      kind: 'refused',
      status: 400
    })
  })
})

describe('childPath', () => {
  it('joins a name to the root and to a directory', () => {
    expect(childPath('/', 'animations')).toBe('/animations')
    expect(childPath('/animations', 'wave.pb')).toBe('/animations/wave.pb')
  })
})

describe('writeFile and readFile', () => {
  it('writes and reads a file over one chunk in 512-byte chunks with progress', async () => {
    const bytes = bytesOf(2 * CHUNK + 276)
    const written: [number, number][] = []

    await writeFile('/data.bin', bytes, (done, total) => written.push([done, total]), broker)

    const writes = robot.requests('fileWriteChunk').map(r => r.fileWriteChunk!)
    expect(writes.map(w => w.offset)).toEqual([0, 512, 1024])
    expect(writes.map(w => w.content.length)).toEqual([512, 512, 276])
    expect(writes.every(w => w.totalSize === bytes.length && w.path === '/data.bin')).toBe(true)
    expect(written).toEqual([
      [512, 1300],
      [1024, 1300],
      [1300, 1300]
    ])
    expect(robot.files.bytes('/data.bin')).toEqual(bytes)

    const read: [number, number][] = []
    const back = await readFile('/data.bin', (done, total) => read.push([done, total]), broker)

    expect(back).toEqual(bytes)
    const reads = robot.requests('fileReadChunk').map(r => r.fileReadChunk!)
    expect(reads.map(r => r.offset)).toEqual([0, 512, 1024])
    expect(reads.every(r => r.length === CHUNK && r.path === '/data.bin')).toBe(true)
    expect(read).toEqual(written)
  })

  it('writes an empty file as one empty chunk', async () => {
    await writeFile('/empty.txt', new Uint8Array(), undefined, broker)

    const writes = robot.requests('fileWriteChunk').map(r => r.fileWriteChunk!)
    expect(writes).toHaveLength(1)
    expect(writes[0]).toMatchObject({ offset: 0, totalSize: 0 })
    expect(robot.files.bytes('/empty.txt')).toEqual(new Uint8Array())
    expect(await readFile('/empty.txt', undefined, broker)).toEqual(new Uint8Array())
  })

  it('replaces a longer file with a shorter one', async () => {
    await writeFile('/note.txt', bytesOf(900), undefined, broker)
    await writeFile('/note.txt', bytesOf(10), undefined, broker)

    expect(robot.files.bytes('/note.txt')).toEqual(bytesOf(10))
  })

  it("rejects with the robot's reason when it refuses a chunk", async () => {
    robot.respond('fileWriteChunk', ({ fileWriteChunk: w }) =>
      w!.offset === 512 ?
        { statusCode: 500, errorMessage: 'the filesystem is full' }
      : robot.files.write(w!.path, w!.offset, w!.totalSize, w!.content)
    )

    const failed = writeFile('/big.bin', bytesOf(1300), undefined, broker)

    await expect(failed).rejects.toThrow('the filesystem is full')
    expect(robot.requests('fileWriteChunk')).toHaveLength(2)
    expect(robot.files.has('/big.bin')).toBe(false)
  })

  it('rejects a read of a missing file with its status', async () => {
    expect(await failure(readFile('/missing.txt', undefined, broker))).toMatchObject({
      kind: 'refused',
      status: 404
    })
  })

  it('rejects a read that returns no data before the end of the file', async () => {
    robot.respond('fileReadChunk', () => ({
      fileChunk: { content: new Uint8Array(), totalSize: 9 }
    }))

    await expect(readFile('/stuck.bin', undefined, broker)).rejects.toThrow(/offset 0/)
    expect(robot.requests('fileReadChunk')).toHaveLength(1)
  })

  it('refuses to read, write or delete under config', async () => {
    for (const action of [
      readFile('/config/wifiSettings.pb', undefined, broker),
      writeFile('/config/wifiSettings.pb', bytesOf(3), undefined, broker),
      writeFile('/config/new.pb', bytesOf(3), undefined, broker),
      deleteFile('/config/wifiSettings.pb', broker),
      makeDirectory('/config/sub', broker)
    ])
      expect(await failure(action)).toMatchObject({ kind: 'refused', status: 400 })
    expect(robot.files.bytes('/config/wifiSettings.pb')).toHaveLength(112)
    expect(robot.files.has('/config/new.pb')).toBe(false)
  })
})

describe('makeDirectory and deleteFile', () => {
  it('creates a directory, accepts one that exists and refuses a file in the way', async () => {
    await makeDirectory('/logs/today', broker)
    await makeDirectory('/logs/today', broker)
    expect(robot.files.isDirectory('/logs/today')).toBe(true)

    robot.files.put('/logs/file', 'x')
    expect(await failure(makeDirectory('/logs/file', broker))).toMatchObject({
      kind: 'refused',
      status: 409
    })
  })

  it('deletes a file and rejects a delete of one that is not there', async () => {
    await deleteFile('/animations/wave.pb', broker)

    expect(robot.files.has('/animations/wave.pb')).toBe(false)
    expect(await failure(deleteFile('/animations/wave.pb', broker))).toMatchObject({
      kind: 'refused',
      status: 400
    })
  })
})
