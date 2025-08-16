<script lang="ts">
  import { onDestroy, onMount } from 'svelte'
  import { gait, jointNames, mode, model, outControllerData } from '$lib/stores'
  import { populateModelCache } from '$lib/utilities'
  import SceneBuilder from '$lib/sceneBuilder'
  import { GUI } from 'three/addons/libs/lil-gui.module.min.js'
  import type { URDFRobot } from 'urdf-loader'
  import { degToRad, lerp } from 'three/src/math/MathUtils'
  import { GaitLabels, GaitType } from '$lib/gait'
  import Motion, { MotionModes } from '$lib/motion'
  import { dataBroker } from '$lib/transport/databroker'
  import { MessageTopic } from '$lib/interfaces/transport.interface'

  interface Props {
    sky?: boolean
    orbit?: boolean
    panel?: boolean
    debug?: boolean
    ground?: boolean
  }

  let { sky = true, orbit = false, panel = true, debug = false, ground = true }: Props = $props()

  let sceneManager = $state(new SceneBuilder())
  let canvas: HTMLCanvasElement | null = $state(null)

  const angles = new Array(18).fill(0)
  const targetAngles = new Array(18).fill(0)

  const motion = new Motion()

  let gui_panel: GUI

  let settings = {
    'Internal kinematic': true,
    'Robot transform controls': false,
    'Auto orient robot': true,
    'Trace feet': debug,
    'Target position': false,
    'Trace points': 30,
    'Fix camera on robot': true,
    'Smooth motion': true,
    Background: 'black'
  }

  let jointAngles: Record<string, number> = {}

  onMount(async () => {
    await populateModelCache()

    jointAngles = $jointNames.reduce((prev, cur) => ({ ...prev, [cur]: 0 }), {})

    await createScene()
    if (panel) createPanel()

    outControllerData.subscribe(data => motion.handleCommand(data))
    dataBroker.on<number[]>(MessageTopic.ANGLE, data => {
      settings['Internal kinematic'] = false
      const correctedData = data.map((angle, index) => {
        return index % 3 === 2 ? angle - 90 : angle
      })
      setTargetAngles(correctedData.map(degToRad))
    })
    mode.subscribe(mode => motion.setMode(mode))
    gait.subscribe(gait => motion.setGait(gait))
  })

  onDestroy(() => {
    canvas?.remove()
    gui_panel?.destroy()
  })

  const createPanel = () => {
    gui_panel = new GUI({ width: 310 })
    gui_panel.close()
    gui_panel.domElement.id = 'three-gui-panel'

    const general = gui_panel.addFolder('General')
    general.add(settings, 'Internal kinematic')
    general.add(settings, 'Robot transform controls')
    general.add(settings, 'Auto orient robot')

    const gait_gui = gui_panel.addFolder('Gait')
    gait_gui.add(motion.gait_state, 'step_height', 0, 50, 0.01).name('Step Height')
    gait_gui.add(motion.gait_state, 'step_x', -50, 50, 0.01).name('Step X')
    gait_gui.add(motion.gait_state, 'step_z', -50, 50, 0.01).name('Step Z')
    gait_gui
      .add(motion.gait_state, 'step_angle', -Math.PI / 4, Math.PI / 4, 0.01)
      .name('Step Angle')
    gait_gui.add(motion.gait_state, 'step_speed', 0, 2, 0.01).name('Step Speed')
    gait_gui.add(motion.gait_state, 'step_depth', 0, 0.01, 0.001).name('Step Depth')
    gait_gui.add(motion.gait_state, 'stand_frac', 0, 1, 0.01).name('Stand Fraction')
    gait_gui
      .add(motion.gait_state, 'gait_type', GaitLabels)
      .name('Gait Type')
      .onChange((value: GaitType) => {
        gait.set(value)
      })

    const kin = gui_panel.addFolder('Kinematics')
    for (const name of ['omega', 'phi', 'psi']) {
      kin.add(motion.body_state, name as any, -Math.PI / 10, Math.PI / 10, 0.01)
    }
    kin.add(motion.body_state, 'xm', -60, 60, 0.01)
    kin.add(motion.body_state, 'ym', -60, 60, 0.01)
    kin.add(motion.body_state, 'zm', -60, 60, 0.01)

    const limbs = gui_panel.addFolder('Limbs')
    limbs.add({ center: resetLimbs }, 'center')
    for (const name of $jointNames) {
      limbs
        .add(jointAngles, name, -Math.PI / 1.2, Math.PI / 1.2)
        .onChange(updateLimbs)
        .listen()
    }

    const visualization = gui_panel.addFolder('Visualization')
    visualization.add(settings, 'Trace feet')
    visualization.add(settings, 'Trace points', 1, 1000, 1)
    visualization.add(settings, 'Target position')
    visualization.add(settings, 'Smooth motion')
    visualization.addColor(settings, 'Background')
  }

  const resetLimbs = () => {
    for (const name of $jointNames) {
      jointAngles[name] = 0
    }
    mode.set(MotionModes.IDLE)
    updateLimbs()
  }

  const updateLimbs = () => {
    for (let i = 0; i < $jointNames.length; i++) {
      targetAngles[i] = jointAngles[$jointNames[i]]
    }
  }

  const setTargetAngles = (angles: number[]) => {
    for (let i = 0; i < angles.length; i++) {
      targetAngles[i] = angles[i]
      jointAngles[$jointNames[i]] = targetAngles[i]
    }
  }

  const createScene = async () => {
    if (!canvas) return
    sceneManager
      .addRenderer({ antialias: true, canvas, alpha: true })
      .addPerspectiveCamera({ x: -0.5, y: 0.5, z: 1 })
      .addOrbitControls(40, 50, orbit)
      .addDirectionalLight({ x: 10, y: 20, z: 10, color: 0xffffff, intensity: 3 })
      .addAmbientLight({ color: 0xffffff, intensity: 0.5 })
      .addFogExp2(0xcccccc, 0.015)
      .addModel($model)
      .fillParent()
      .addRenderCb(render)
      .startRenderLoop()

    if (ground) sceneManager.addGroundPlane()

    if (sky) sceneManager.addSky()
  }

  const smooth = (start: number, end: number, amount: number) => {
    return settings['Smooth motion'] ? lerp(start, end, amount) : end
  }

  const update_camera = (robot: URDFRobot) => {
    if (!settings['Fix camera on robot']) return
    sceneManager.orbit.target = robot.position.clone()
  }

  const orient_robot = (robot: URDFRobot) => {
    if (settings['Robot transform controls'] || !settings['Auto orient robot']) return

    robot.position.z = smooth(robot.position.z, -motion.body_state.xm / 12, 0.1)
    robot.position.x = smooth(robot.position.x, -motion.body_state.ym / 12, 0.1)

    robot.rotation.z = smooth(robot.rotation.z, -motion.body_state.psi + Math.PI / 2, 0.1)
    robot.rotation.y = smooth(robot.rotation.y, motion.body_state.omega, 0.1)
    robot.rotation.x = smooth(robot.rotation.x, -motion.body_state.phi - Math.PI / 2, 0.1)
  }

  const render = () => {
    const robot = sceneManager.model
    if (!robot) return
    if (settings['Internal kinematic']) {
      const updated = motion.step()
      if (updated) setTargetAngles(motion.targetAngles)
    }
    update_camera(robot)
    orient_robot(robot)

    for (let i = 0; i < $jointNames.length; i++) {
      angles[i] = smooth(robot.joints[$jointNames[i]].angle as number, targetAngles[i], 0.1)
      robot.joints[$jointNames[i]].setJointValue(angles[i])
    }
  }
</script>

<svelte:window onresize={sceneManager.fillParent} />

<canvas bind:this={canvas}></canvas>
