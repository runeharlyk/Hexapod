// The python the build scripts run: one with grpc_tools, which compiles the protos for both the app
// and the firmware.
import { execFileSync } from 'child_process'
import os from 'os'

const isWindows = os.platform() === 'win32'

// An active virtualenv (e.g. simulation/.venv) shadows the system python that carries
// grpcio-tools. Resolve python with the venv stripped from PATH so we reach the system one.
function systemPythonEnv() {
  const env = { ...process.env }
  const venv = env.VIRTUAL_ENV
  delete env.VIRTUAL_ENV
  delete env.PYTHONHOME
  if (env.PATH) {
    const sep = isWindows ? ';' : ':'
    env.PATH = env.PATH.split(sep)
      .filter(p => {
        const q = p.replace(/\\/g, '/').toLowerCase()
        if (venv && q.startsWith(venv.replace(/\\/g, '/').toLowerCase())) return false
        return !q.includes('/.venv/')
      })
      .join(sep)
  }
  return env
}

export const pyEnv = systemPythonEnv()

export function findPython() {
  for (const py of ['python', 'python3', 'py']) {
    try {
      execFileSync(py, ['-c', 'import grpc_tools.protoc'], { stdio: 'ignore', env: pyEnv })
      return py
    } catch {
      /* try next */
    }
  }
  // Fall back to the first system python that runs, installing grpcio-tools into it.
  for (const py of ['python', 'python3', 'py']) {
    try {
      execFileSync(py, ['--version'], { stdio: 'ignore', env: pyEnv })
      console.log(`Installing grpcio-tools into ${py}...`)
      execFileSync(py, ['-m', 'pip', 'install', 'grpcio-tools'], { stdio: 'inherit', env: pyEnv })
      return py
    } catch {
      /* try next */
    }
  }
  throw new Error('No python with grpc_tools found; install python and grpcio-tools')
}
