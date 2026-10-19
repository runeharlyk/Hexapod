// What the virtual robot (scripts/build_virtual_robot.js) is built from, and whether a build still matches it. A build
// records a hash of every input next to the WebAssembly; a later check hashes the inputs again and names any that
// changed, so a stale build fails loudly instead of failing tests on the firmware it predates.
import { createHash } from 'crypto'
import fs from 'fs'
import path from 'path'

/**
 * @typedef {{ dir: string, only?: RegExp, skip?: string[], depth?: number }} InputDir
 * @typedef {{ file: string }} InputFile
 * @typedef {Record<string, string>} Hashes
 */

// Relative to the repository root. Whole directories stand for the headers the sources include, since a header can
// only reach the build from them; generated files the build writes itself are left out.
/** @type {(InputDir | InputFile)[]} */
const INPUTS = [
  { dir: 'firmware/host' },
  { dir: 'firmware/include', skip: ['firmware/include/WWWData.h'] },
  { dir: 'firmware/src', only: /\.h$/, skip: ['firmware/src/platform_shared'] },
  { dir: 'platform_shared', only: /\.(proto|options)$/ },
  { dir: 'animations', only: /\.json$/, depth: 1 },
  { dir: 'submodules/nanopb', only: /^pb.*\.[ch]$/, depth: 1 },
  { file: 'submodules/nanopb/generator/nanopb_generator.py' },
  { file: 'firmware/features.ini' },
  { file: 'firmware/scripts/compile_protos.py' },
  { file: 'firmware/scripts/pack_animations.py' },
  { file: 'app/scripts/build_virtual_robot.js' }
]

const STAMP = 'virtual_robot.inputs.json'

/**
 * @param {string} root
 * @param {InputDir} input
 * @returns {string[]}
 */
function listDir(root, { dir, only, skip = [], depth = Infinity }) {
  /** @type {string[]} */
  const found = []
  /** @type {(relative: string, level: number) => void} */
  const walk = (relative, level) => {
    if (skip.some(s => relative === s || relative.startsWith(`${s}/`))) return
    const absolute = path.join(root, relative)
    if (!fs.existsSync(absolute)) return
    for (const entry of fs.readdirSync(absolute, { withFileTypes: true })) {
      const child = `${relative}/${entry.name}`
      if (entry.isDirectory()) {
        if (level < depth) walk(child, level + 1)
      } else if (!only || only.test(entry.name)) {
        if (!skip.includes(child)) found.push(child)
      }
    }
  }
  walk(dir, 1)
  return found
}

/**
 * Every input file under `root` with the SHA-256 of its content, keyed by its path from the root.
 * @param {string} root
 * @returns {Hashes}
 */
export function hashInputs(root) {
  const files = INPUTS.flatMap(input =>
    'file' in input ?
      fs.existsSync(path.join(root, input.file)) ?
        [input.file]
      : []
    : listDir(root, input)
  )
  return Object.fromEntries(
    [...new Set(files)].sort().map(file => [
      file,
      createHash('sha256')
        .update(new Uint8Array(fs.readFileSync(path.join(root, file))))
        .digest('hex')
    ])
  )
}

/**
 * The inputs whose content differs between two hashings, added and removed ones included.
 * @param {Hashes} built
 * @param {Hashes} current
 */
export function changedInputs(built, current) {
  const paths = new Set([...Object.keys(built), ...Object.keys(current)])
  return [...paths].filter(file => built[file] !== current[file]).sort()
}

/** @param {string} outputDir */
export const stampPath = outputDir => path.join(outputDir, STAMP)

/**
 * @param {string} outputDir
 * @param {Hashes} hashes
 */
export const writeStamp = (outputDir, hashes) =>
  fs.writeFileSync(stampPath(outputDir), JSON.stringify(hashes, null, 1) + '\n')

/**
 * The hashes the last build recorded, or null when no build recorded any.
 * @param {string} outputDir
 * @returns {Hashes | null}
 */
export function readStamp(outputDir) {
  try {
    return JSON.parse(fs.readFileSync(stampPath(outputDir), 'utf8'))
  } catch {
    return null
  }
}
