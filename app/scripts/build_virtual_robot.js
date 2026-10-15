#!/usr/bin/env node
// Compiles the firmware's Hexapod (firmware/host/virtual_robot.h) to WebAssembly for the simulation view:
// the same MotionService, gait, kinematics, policy and animation runner the robot runs, so the view needs
// no copy of them. Needs Emscripten: em++ from $EMSDK or on PATH. Without it the output is a module that
// rejects with the reason, so the rest of the app still builds; CI must have it and fails without it.
import { execFileSync } from 'child_process'
import fs from 'fs'
import os from 'os'
import path from 'path'
import { fileURLToPath } from 'url'
import { findPython, pyEnv } from './python.js'

const __dirname = path.dirname(fileURLToPath(import.meta.url))
const root = path.resolve(__dirname, '..', '..')
const outputDir = path.resolve(__dirname, '..', 'src', 'lib', 'simulation', 'virtual-robot')
const output = path.join(outputDir, 'virtual_robot.mjs')
const exe = os.platform() === 'win32' ? '.exe' : ''

function findEmxx() {
  const candidates = []
  if (process.env.EMSDK)
    candidates.push(path.join(process.env.EMSDK, 'upstream', 'emscripten', `em++${exe}`))
  candidates.push(`em++${exe}`, 'em++')
  for (const candidate of candidates) {
    try {
      execFileSync(candidate, ['--version'], { stdio: 'ignore' })
      return candidate
    } catch {
      /* try next */
    }
  }
  return null
}

// features.ini is the robot's build configuration; the virtual robot compiles with the same flags.
function featureFlags() {
  const ini = fs.readFileSync(path.join(root, 'firmware', 'features.ini'), 'utf8')
  return [...ini.matchAll(/^\s*-D\s*([A-Z0-9_]+=\S+)/gm)].map(([, define]) => `-D${define}`)
}

fs.mkdirSync(outputDir, { recursive: true })
const emxx = findEmxx()
if (!emxx) {
  if (process.env.CI)
    throw new Error('em++ not found: the virtual robot needs Emscripten (set EMSDK)')
  console.warn(
    'em++ not found; the simulation view will report that the virtual robot was not built'
  )
  fs.writeFileSync(
    output,
    "export default () => Promise.reject(new Error('The virtual robot was not built: install Emscripten and rebuild'))\n"
  )
  process.exit(0)
}

const python = findPython()
for (const script of ['compile_protos.py', 'pack_animations.py']) {
  execFileSync(python, [path.join('firmware', 'scripts', script)], {
    stdio: 'inherit',
    cwd: root,
    env: pyEnv
  })
}

const generated = path.join('firmware', 'src', 'platform_shared')
const sources = [
  'firmware/host/wasm_api.cpp',
  'firmware/host/filesystem_host.cpp',
  ...['pb_common.c', 'pb_decode.c', 'pb_encode.c'].map(f => path.join('submodules', 'nanopb', f)),
  ...['animation.pb.c', 'api.pb.c', 'message.pb.c'].map(f => path.join(generated, f))
]
execFileSync(
  emxx,
  [
    '-O2',
    '-std=gnu++2a',
    ...featureFlags(),
    ...['firmware/host', 'firmware/include', 'firmware/src', generated, 'submodules/nanopb'].map(
      d => `-I${d}`
    ),
    ...sources.flatMap(s => (s.endsWith('.c') ? ['-x', 'c', s, '-x', 'none'] : [s])),
    '--embed-file',
    'firmware/data/animations@/littlefs/animations',
    '-sMODULARIZE',
    '-sEXPORT_ES6',
    '-sEXPORT_NAME=createVirtualRobot',
    '-sENVIRONMENT=web,node',
    '-sALLOW_MEMORY_GROWTH',
    '-sEXPORTED_RUNTIME_METHODS=HEAPF32',
    '-o',
    output
  ],
  { stdio: 'inherit', cwd: root }
)
console.log(`Virtual robot built: ${path.relative(root, output)}`)
