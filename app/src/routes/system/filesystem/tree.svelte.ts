import { listDirectory, type FileEntry } from '$lib/files'

/** What the browser knows of one directory: its entries as last read, and whether a read is under way or failed. */
export interface Listing {
  entries: FileEntry[]
  loading: boolean
  error: string
}

/**
 * The directories the browser has opened, each read when it is opened. A directory stays as it was while it is read
 * again, so a new file or a delete does not fold the tree up.
 */
export class DirectoryTree {
  listings = $state<Record<string, Listing>>({})
  private readonly reads = new Map<string, number>()

  async load(path: string) {
    const read = (this.reads.get(path) ?? 0) + 1
    this.reads.set(path, read)
    const entries = this.listings[path]?.entries ?? []
    this.listings[path] = { entries, loading: true, error: '' }
    try {
      const listed = await listDirectory(path)
      if (this.reads.get(path) === read)
        this.listings[path] = { entries: listed, loading: false, error: '' }
    } catch (error) {
      if (this.reads.get(path) === read)
        this.listings[path] = {
          entries,
          loading: false,
          error: error instanceof Error ? error.message : String(error)
        }
    }
  }

  /** Reads a directory again when it has been opened; one never opened has nothing to bring up to date. */
  refresh(path: string) {
    if (path in this.listings) return this.load(path)
  }
}
