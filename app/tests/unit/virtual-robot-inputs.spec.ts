// @vitest-environment node
import { afterEach, beforeEach, describe, expect, it } from 'vitest'
import fs from 'node:fs'
import os from 'node:os'
import path from 'node:path'
import {
  changedInputs,
  hashInputs,
  readStamp,
  staleBuild,
  writeStamp
} from '../../scripts/virtual-robot-inputs.js'

// A repository holding a few files of each kind the virtual robot is built from, and some it is not.
let root: string
const write = (file: string, content: string) => {
  fs.mkdirSync(path.dirname(path.join(root, file)), { recursive: true })
  fs.writeFileSync(path.join(root, file), content)
}

beforeEach(() => {
  root = fs.mkdtempSync(path.join(os.tmpdir(), 'virtual-robot-inputs-'))
  write('firmware/include/motion.h', 'struct Motion {};')
  write('firmware/include/utils/math_utils.h', 'float lerp();')
  write('firmware/host/virtual_robot.h', 'struct VirtualRobot {};')
  write('firmware/src/platform_shared/message.pb.h', '/* generated */')
  write('firmware/include/WWWData.h', '/* generated */')
  write('firmware/features.ini', '-D USE_CAMERA=1')
  write('platform_shared/message.proto', 'message Mode {}')
  write('platform_shared/sign_convention.json', '{}')
  write('animations/wave.json', '{}')
})

afterEach(() => fs.rmSync(root, { recursive: true, force: true }))

const changedSince = (built: Record<string, string>) => changedInputs(built, hashInputs(root))

describe('the virtual robot inputs', () => {
  it('match a build made from them', () => {
    expect(changedSince(hashInputs(root))).toEqual([])
  })

  it('name a header that changed since the build, however deep', () => {
    const built = hashInputs(root)
    write('firmware/include/motion.h', 'struct Motion { int mode; };')
    write('firmware/include/utils/math_utils.h', 'float lerp(float);')
    write('firmware/host/virtual_robot.h', 'struct VirtualRobot { int ticks; };')
    expect(changedSince(built)).toEqual([
      'firmware/host/virtual_robot.h',
      'firmware/include/motion.h',
      'firmware/include/utils/math_utils.h'
    ])
  })

  it('name a file added or removed since the build', () => {
    const built = hashInputs(root)
    write('firmware/include/odometry.h', 'struct Odometry {};')
    fs.rmSync(path.join(root, 'animations/wave.json'))
    expect(changedSince(built)).toEqual(['animations/wave.json', 'firmware/include/odometry.h'])
  })

  it('name the schema, the build flags and the bundled animations', () => {
    const built = hashInputs(root)
    write('platform_shared/message.proto', 'message Mode { int32 mode = 1; }')
    write('firmware/features.ini', '-D USE_CAMERA=0')
    write('animations/wave.json', '{"name": "wave"}')
    expect(changedSince(built)).toEqual([
      'animations/wave.json',
      'firmware/features.ini',
      'platform_shared/message.proto'
    ])
  })

  // The build writes the generated sources itself, and the sign convention is what the tests check the build against.
  it('leave out what the build generates and what only the tests read', () => {
    const built = hashInputs(root)
    write('firmware/src/platform_shared/message.pb.h', '/* generated again */')
    write('firmware/include/WWWData.h', '/* generated again */')
    write('platform_shared/sign_convention.json', '{"commands": []}')
    expect(changedSince(built)).toEqual([])
  })

  // secrets.h is included only by settings/ap_settings.h and wifi_service.cpp, neither of which the build compiles.
  it('leave out the Wi-Fi credentials, which the build does not compile', () => {
    write('firmware/include/secrets.h', '#define WIFI_PASSWORD "one"')
    const built = hashInputs(root)
    write('firmware/include/secrets.h', '#define WIFI_PASSWORD "two"')
    expect(Object.keys(built)).not.toContain('firmware/include/secrets.h')
    expect(changedSince(built)).toEqual([])
  })

  it('name the protobuf generator and the options schema it reads', () => {
    write('submodules/nanopb/generator/nanopb_generator.py', '# generator')
    write('submodules/nanopb/generator/proto/nanopb.proto', 'message NanoPBOptions {}')
    const built = hashInputs(root)
    write(
      'submodules/nanopb/generator/proto/nanopb.proto',
      'message NanoPBOptions { int32 max_size = 1; }'
    )
    expect(changedSince(built)).toEqual(['submodules/nanopb/generator/proto/nanopb.proto'])
  })

  it('read back the stamp a build wrote, and nothing where none was written', () => {
    const outputDir = path.join(root, 'out')
    fs.mkdirSync(outputDir)
    expect(readStamp(outputDir)).toBeNull()
    writeStamp(outputDir, hashInputs(root))
    expect(readStamp(outputDir)).toEqual(hashInputs(root))
  })
})

describe('staleBuild', () => {
  const outputDir = () => path.join(root, 'out')
  const build = () => {
    fs.mkdirSync(outputDir(), { recursive: true })
    write('out/virtual_robot.wasm', 'wasm')
    writeStamp(outputDir(), hashInputs(root))
  }

  it('finds nothing wrong with a build made from the current sources', () => {
    build()
    expect(staleBuild(root, outputDir())).toBeNull()
  })

  it('names the sources a build predates and how to rebuild it', () => {
    build()
    write('firmware/include/motion.h', 'struct Motion { int mode; };')
    const reason = staleBuild(root, outputDir())
    expect(reason).toContain('older than firmware/include/motion.h')
    expect(reason).toContain('pnpm virtual-robot')
  })

  it('reports a build that is missing or has no record', () => {
    expect(staleBuild(root, outputDir())).toContain('missing or has no record')
  })
})
