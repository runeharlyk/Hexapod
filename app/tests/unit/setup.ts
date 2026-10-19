// Under jsdom, Vite rewrites the Emscripten module's own URL to a dev-server address, where Emscripten's Node loader
// cannot read its .wasm; the module is pointed at the file on disk instead. A build older than the firmware sources
// (which scripts/check_virtual_robot.js rebuilds when it can) fails the suite that loads it, and only that suite.
import { resolve } from 'node:path'
import { vi } from 'vitest'

// vi.mock is hoisted above the module's other statements, so everything it uses is inside it.
vi.mock('../../src/lib/simulation/virtual-robot/virtual_robot.mjs', async importOriginal => {
  const { staleBuild } = await import('../../scripts/virtual-robot-inputs.js')
  const outputDir = resolve(__dirname, '../../src/lib/simulation/virtual-robot')
  const stale = staleBuild(resolve(__dirname, '../../..'), outputDir)
  if (stale !== null) return { default: () => Promise.reject(new Error(stale)) }
  const { default: create } = await importOriginal<{
    default: (options: object) => Promise<unknown>
  }>()
  const wasm = resolve(outputDir, 'virtual_robot.wasm')
  return { default: () => create({ locateFile: () => wasm }) }
})
