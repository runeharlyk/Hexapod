// What the simulation view loads when it first opens: MuJoCo's WebAssembly build (about 10 MB), the virtual
// robot and the hexapod's MuJoCo model, the one the training uses.
import type { MainModule } from '@mujoco/mujoco'
import wasmUrl from '@mujoco/mujoco/mujoco.wasm?url'
import sceneXml from '../../../../simulation/src/resources/model.xml?raw'
import { loadVirtualRobot, type VirtualRobot } from './virtual-robot'

let engine: Promise<MainModule> | undefined

/** The engine is fetched once per page and shared; a failed fetch is retried on the next load. */
const mujocoModule = () =>
  (engine ??= import('@mujoco/mujoco')
    .then(({ default: load }) => load({ locateFile: () => wasmUrl }))
    .catch(error => {
      engine = undefined
      throw error
    }))

export async function loadSimulation(): Promise<{
  mujoco: MainModule
  robot: VirtualRobot
  sceneXml: string
}> {
  const [mujoco, robot] = await Promise.all([mujocoModule(), loadVirtualRobot()])
  return { mujoco, robot, sceneXml }
}
