#!/usr/bin/env node
// Runs before the unit tests, which run the firmware's motion code through the virtual robot: a build older than the
// firmware sources would fail them on behaviour the firmware no longer has, so a stale build is rebuilt here.
// Without Emscripten the build stays stale and only the suites that load it fail, each saying why (tests/unit/setup.ts),
// so the rest of the unit tests, and a run of one unrelated file, still run.
import { execFileSync } from 'child_process'
import path from 'path'
import { fileURLToPath } from 'url'
import { staleBuild } from './virtual-robot-inputs.js'

const __dirname = path.dirname(fileURLToPath(import.meta.url))
const root = path.resolve(__dirname, '..', '..')
const outputDir = path.resolve(__dirname, '..', 'src', 'lib', 'simulation', 'virtual-robot')

if (staleBuild(root, outputDir) === null) process.exit(0)

console.log('The virtual robot predates the firmware sources; rebuilding it.')
try {
  execFileSync(process.execPath, [path.join(__dirname, 'build_virtual_robot.js')], {
    stdio: 'inherit'
  })
} catch {
  // The compiler has said what failed; the build left no record, so the next run tries again.
  console.error('\nBuilding the virtual robot failed (see above), so the unit tests do not run.\n')
  process.exit(1)
}

const reason = staleBuild(root, outputDir)
if (reason !== null)
  console.warn(`\n${reason}\nThe suites that load it will fail; the others run.\n`)
