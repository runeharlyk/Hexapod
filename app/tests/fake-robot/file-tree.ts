import type { Answer } from './robot'

const PAGE = 26
const CHUNK_MAX = 512
const NAME_MAX = 63
const PRIVATE_DIRECTORY = 'config'

const segmentsOf = (path: string) => path.split('/').filter(segment => segment && segment !== '.')
const keyOf = (segments: string[]) => '/' + segments.join('/')
const parentOf = (key: string) => keyOf(segmentsOf(key).slice(0, -1))
const bytesOf = (content: string | Uint8Array) =>
  typeof content === 'string' ? new TextEncoder().encode(content) : content
const byteOrder = (a: string, b: string) => {
  const [x, y] = [bytesOf(a), bytesOf(b)]
  const differing = x.findIndex((byte, i) => byte !== y[i])
  return differing < 0 ? x.length - y.length : x[differing] - (y[differing] ?? -1)
}

/** A path the firmware accepts (file_transfer::validPath): absolute, without `..`, outside the private directory. */
const valid = (path: string) =>
  path.startsWith('/') && !path.includes('..') && segmentsOf(path)[0] !== PRIVATE_DIRECTORY

/**
 * The robot's LittleFS as its file requests see it (firmware/include/file_transfer.h): the same statuses, a listing
 * paged and sorted as the firmware pages it, the private `config` directory refused and hidden from the root, a write
 * kept in a `.part` file until its last chunk replaces the target.
 */
export class FileTree {
  /** A file's bytes, or null for a directory; the root is implicit. */
  private readonly nodes = new Map<string, Uint8Array | null>()

  constructor(files: Record<string, string | Uint8Array> = {}) {
    for (const [path, content] of Object.entries(files)) this.put(path, content)
  }

  /** Stores a file, and its directories, as the robot's own firmware or another client would have (no path checks). */
  put(path: string, content: string | Uint8Array) {
    const key = keyOf(segmentsOf(path))
    this.makeParents(key)
    this.nodes.set(key, bytesOf(content))
  }

  has(path: string) {
    return this.nodes.has(keyOf(segmentsOf(path)))
  }

  isDirectory(path: string) {
    return this.nodes.get(keyOf(segmentsOf(path))) === null
  }

  /** A file's bytes. */
  bytes(path: string) {
    const bytes = this.nodes.get(keyOf(segmentsOf(path)))
    if (!bytes) throw new Error(`${path} is not a file`)
    return bytes
  }

  /** A file's content as text. */
  text(path: string) {
    return new TextDecoder().decode(this.bytes(path))
  }

  private makeParents(key: string) {
    const segments = segmentsOf(key)
    for (let depth = 1; depth < segments.length; depth++) {
      const ancestor = keyOf(segments.slice(0, depth))
      if (this.nodes.get(ancestor) instanceof Uint8Array) return false
      this.nodes.set(ancestor, null)
    }
    return true
  }

  list(path: string, offset: number): Answer {
    if (!valid(path)) return { statusCode: 400, errorMessage: 'bad path' }
    const key = keyOf(segmentsOf(path))
    if (key !== '/' && this.nodes.get(key) !== null) return { statusCode: 404 }
    const entries = [...this.nodes]
      .filter(([child]) => child !== key && parentOf(child) === key && valid(child))
      .map(([child, node]) => ({
        name: segmentsOf(child).at(-1)!,
        isDirectory: node === null,
        size: node?.length ?? 0
      }))
      .filter(entry => bytesOf(entry.name).length <= NAME_MAX)
      .sort((a, b) =>
        a.isDirectory === b.isDirectory ? byteOrder(a.name, b.name)
        : a.isDirectory ? -1
        : 1
      )
    return { fileList: { entries: entries.slice(offset, offset + PAGE), total: entries.length } }
  }

  mkdir(path: string): Answer {
    if (!valid(path)) return { statusCode: 400 }
    const key = keyOf(segmentsOf(path))
    const present = this.nodes.get(key)
    if (present !== undefined) return { statusCode: present === null ? 200 : 409 }
    if (!this.makeParents(key)) return { statusCode: 500 }
    this.nodes.set(key, null)
    return {}
  }

  read(path: string, offset: number, length: number): Answer {
    if (!valid(path) || length > CHUNK_MAX) return { statusCode: 400 }
    const bytes = this.nodes.get(keyOf(segmentsOf(path)))
    if (!bytes) return { statusCode: 404 }
    return { fileChunk: { content: bytes.slice(offset, offset + length), totalSize: bytes.length } }
  }

  write(path: string, offset: number, totalSize: number, content: Uint8Array): Answer {
    if (!valid(path) || content.length > CHUNK_MAX || offset + content.length > totalSize)
      return { statusCode: 400 }
    const key = keyOf(segmentsOf(path))
    const part = key + '.part'
    const held = this.nodes.get(part)
    if (offset !== 0 && held?.length !== offset) return { statusCode: 400 }
    const parent = parentOf(key)
    if (parent !== '/' && this.nodes.get(parent) !== null) return { statusCode: 500 }
    const bytes = new Uint8Array(offset + content.length)
    if (held) bytes.set(held.subarray(0, offset))
    bytes.set(content, offset)
    this.nodes.set(part, bytes)
    if (bytes.length < totalSize) return {}
    if (this.nodes.get(key) === null) return { statusCode: 500 }
    this.nodes.delete(part)
    this.nodes.set(key, bytes)
    return {}
  }

  delete(path: string): Answer {
    if (!valid(path) || !(this.nodes.get(keyOf(segmentsOf(path))) instanceof Uint8Array))
      return { statusCode: 400 }
    this.nodes.delete(keyOf(segmentsOf(path)))
    return {}
  }
}
