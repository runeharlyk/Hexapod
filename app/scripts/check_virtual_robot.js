#!/usr/bin/env node
// Runs before the unit tests, which run the firmware's motion code through the virtual robot: a build older than the
// firmware sources fails them on behaviour the firmware no longer has. A stale build is rebuilt; without Emscripten
// to rebuild it, the check fails and says how.
import { execFileSync } from 'child_process'
import fs from 'fs'
import path from 'path'
import { fileURLToPath } from 'url'
import { changedInputs, hashInputs, readStamp } from './virtual-robot-inputs.js'

const __dirname = path.dirname(fileURLToPath(import.meta.url))
const root = path.resolve(__dirname, '..', '..')
const outputDir = path.resolve(__dirname, '..', 'src', 'lib', 'simulation', 'virtual-robot')

// The inputs changed since the build, or null when there is no build to compare.
const changedSinceBuild = () => {
  const built = fs.existsSync(path.join(outputDir, 'virtual_robot.wasm')) && readStamp(outputDir)
  return built ? changedInputs(built, hashInputs(root)) : null
}

const SHOWN = 5
const describe = changed =>
  changed.length > SHOWN ?
    `${changed.slice(0, SHOWN).join(', ')} and ${changed.length - SHOWN} more`
  : changed.join(', ')

let changed = changedSinceBuild()
if (changed?.length === 0) process.exit(0)

console.log(
  changed ?
    `The virtual robot predates ${describe(changed)}; rebuilding it.`
  : 'The virtual robot has no recorded build; building it.'
)
execFileSync(process.execPath, [path.join(__dirname, 'build_virtual_robot.js')], {
  stdio: 'inherit'
})

changed = changedSinceBuild()
if (changed?.length === 0) process.exit(0)
console.error(
  `\nThe virtual robot (src/lib/simulation/virtual-robot/virtual_robot.wasm) is ${
    changed ?
      `older than ${describe(changed)}`
    : 'missing or has no record of the sources it was built from'
  }, so the tests that run the firmware's motion code through it would test other code.\n` +
    'Build it with `pnpm virtual-robot`, which needs Emscripten: set EMSDK to the emsdk directory or put em++ on PATH.\n'
)
process.exit(1)
