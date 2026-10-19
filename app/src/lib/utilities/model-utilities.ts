import { Box3, Vector3 } from 'three'
import URDFLoader, { type URDFRobot } from 'urdf-loader'
import { XacroLoader } from 'xacro-parser'
import { Result } from '$lib/utilities'
import { jointNames, model } from '$lib/stores'
import { resolve } from '$app/paths'
import { LEVEL } from '$lib/model-frame'

const base = resolve('/')

/**
 * A fresh copy of the robot's URDF model, oriented and scaled for the 3D view, and its moving joints' names,
 * once its meshes have loaded.
 */
export const loadModel = () => loadModelAsync(`${base}model.xacro`)

export const populateModelCache = async () => {
  const modelRes = await loadModel()
  if (modelRes.isOk()) {
    const [urdf, JOINT_NAME] = modelRes.inner
    jointNames.set(JOINT_NAME)
    model.set(urdf)
  } else {
    console.error(modelRes.inner, { exception: modelRes.exception })
  }
}

const loadModelAsync = async (url: string): Promise<Result<[URDFRobot, string[]], string>> => {
  return new Promise(res => {
    const xacroLoader = new XacroLoader()
    const urdfLoader = new URDFLoader()
    ;(xacroLoader as XacroLoader & { workingPath: string }).workingPath = base
    urdfLoader.packages = {
      hex: `${base}hex`
    }
    urdfLoader.workingPath = base

    xacroLoader.load(
      url,
      async xml => {
        try {
          // The meshes arrive after the parse, and centring the model measures them. Their loads are counted
          // here rather than by a LoadingManager, which never hears of a load three.js merged into another
          // page's request for the same file.
          let pendingMeshes = 0
          let model: URDFRobot | undefined = undefined
          const finish = () => {
            if (!model) return
            model.quaternion.copy(LEVEL)
            model.traverse(c => (c.castShadow = true))
            model.scale.setScalar(10)
            centerRobotPivot(model)
            model.updateMatrixWorld(true)
            const joints = Object.entries(model.joints)
              .filter(joint => joint[1].jointType !== 'fixed')
              .map(joint => joint[0])
            res(Result.ok([model, joints]))
          }
          urdfLoader.loadMeshCb = (path, manager, done) => {
            pendingMeshes++
            urdfLoader.defaultMeshLoader(path, manager, (mesh, error) => {
              done(mesh, error)
              if (--pendingMeshes === 0) finish()
            })
          }
          model = urdfLoader.parse(xml)
          if (pendingMeshes === 0) finish()
        } catch (error) {
          res(Result.err('Failed to load model', error))
        }
      },
      error => res(Result.err('Failed to load model', error))
    )
  })
}

const centerRobotPivot = (robot: URDFRobot) => {
  robot.updateMatrixWorld(true)
  const bounds = new Box3().setFromObject(robot)
  const center = bounds.getCenter(new Vector3())

  robot.position.x -= center.x
  robot.position.z -= center.z
}
