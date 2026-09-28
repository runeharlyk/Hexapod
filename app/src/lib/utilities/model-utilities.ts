import { Box3, Vector3 } from 'three'
import URDFLoader, { type URDFRobot } from 'urdf-loader'
import { XacroLoader } from 'xacro-parser'
import { Result } from '$lib/utilities'
import { jointNames, model } from '$lib/stores'
import { resolve } from '$app/paths'

const base = resolve('/')

export const populateModelCache = async () => {
  const modelRes = await loadModelAsync(`${base}model.xacro`)
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
          const model = urdfLoader.parse(xml)

          model.rotation.x = -Math.PI / 2
          model.rotation.z = Math.PI / 2
          model.traverse(c => (c.castShadow = true))
          model.scale.setScalar(10)
          centerRobotPivot(model)
          model.updateMatrixWorld(true)
          const joints = Object.entries(model.joints)
            .filter(joint => joint[1].jointType !== 'fixed')
            .map(joint => joint[0])

          res(Result.ok([model, joints]))
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
