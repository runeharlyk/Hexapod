#!/usr/bin/env node
// Generate ts-proto TypeScript from platform_shared/*.proto, the same schema the firmware
// compiles with nanopb. Uses python's grpc_tools.protoc as the protoc (the same dependency
// the firmware pipeline already needs) with a small node wrapper for the ts-proto plugin —
// the npm-bundled protoc mishandles the plugin stdio on Windows.
import { execFileSync } from 'child_process'
import { createRequire } from 'module'
import fs from 'fs'
import os from 'os'
import path from 'path'
import { fileURLToPath } from 'url'
import { findPython, pyEnv } from './python.js'

const require = createRequire(import.meta.url)
const __dirname = path.dirname(fileURLToPath(import.meta.url))
const isWindows = os.platform() === 'win32'
const projectRoot = path.resolve(__dirname, '..')
const platformSharedDir = path.resolve(projectRoot, '..', 'platform_shared')
const outputDir = path.resolve(projectRoot, 'src', 'lib', 'platform_shared')
const protoFiles = ['message.proto', 'api.proto', 'animation.proto']

function writePluginWrapper() {
  const pluginJs = require.resolve('ts-proto/protoc-gen-ts_proto')
  const wrapper = path.join(__dirname, isWindows ? 'ts-proto-plugin.bat' : 'ts-proto-plugin.sh')
  if (isWindows) {
    fs.writeFileSync(wrapper, `@node "${pluginJs}" %*\r\n`)
  } else {
    fs.writeFileSync(wrapper, `#!/bin/sh\nexec node "${pluginJs}" "$@"\n`)
    fs.chmodSync(wrapper, 0o755)
  }
  return wrapper
}

fs.mkdirSync(outputDir, { recursive: true })
const python = findPython()
const wrapper = writePluginWrapper()
const opts = ['useExactTypes=false', 'outputExtensions=true', 'outputSchema=true'].join(',')

const args = [
  '-m',
  'grpc_tools.protoc',
  `--plugin=protoc-gen-ts_proto=${wrapper}`,
  `--ts_proto_out=${outputDir}`,
  `--ts_proto_opt=${opts}`,
  `-I${platformSharedDir}`,
  ...protoFiles.map(f => path.join(platformSharedDir, f))
]

console.log(`Compiling protos -> ${outputDir}`)
try {
  execFileSync(python, args, { stdio: 'inherit', cwd: projectRoot, env: pyEnv })
  console.log('Proto compilation complete')
} catch (error) {
  console.error('Proto compilation failed:', error.message)
  process.exit(1)
}
