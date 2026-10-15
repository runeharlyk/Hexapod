// Under jsdom, Vite rewrites the Emscripten module's own URL to a dev-server address, where Emscripten's Node loader
// cannot read its .wasm; the module is pointed at the file on disk instead.
import { resolve } from 'node:path'
import { vi } from 'vitest'

// vi.mock is hoisted above the module's other statements, so everything it uses is inside it.
vi.mock('../../src/lib/simulation/virtual-robot/virtual_robot.mjs', async importOriginal => {
  const { default: create } = await importOriginal<{
    default: (options: object) => Promise<unknown>
  }>()
  const wasm = resolve(__dirname, '../../src/lib/simulation/virtual-robot/virtual_robot.wasm')
  return { default: () => create({ locateFile: () => wasm }) }
})
