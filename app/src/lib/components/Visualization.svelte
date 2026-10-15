<script lang="ts">
  import { onDestroy, onMount } from 'svelte'
  import { gait, jointNames, mode, model, outControllerData } from '$lib/stores'
  import { populateModelCache } from '$lib/utilities'
  import SceneBuilder from '$lib/sceneBuilder'
  import { GUI } from 'three/addons/libs/lil-gui.module.min.js'
  import type { URDFRobot } from 'urdf-loader'
  import { degToRad, lerp } from 'three/src/math/MathUtils'
  import { GaitType } from '$lib/gait'
  import { MotionModes, urdfJointOrder } from '$lib/motion'
  import { dataBroker } from '$lib/transport/databroker'
  import { AnglesData } from '$lib/platform_shared/message'
  import { Vector3 } from 'three'
  import { requestMode } from '$lib/control'
  import { isLinked } from '$lib/stores/link'
  import { animationPreview, type AnimationPreview } from '$lib/stores/animation'
  import Kinematics, { levelBody, type body_state_t } from '$lib/kinematic'
  import { config } from '$lib/components/config'
  import {
    loadVirtualRobot,
    TICK_SECONDS,
    type Command,
    type VirtualRobot
  } from '$lib/simulation/virtual-robot'
  import type { Vec3 } from '$lib/animation/model'
  import { DEFAULT_FEET } from '$lib/animation/model'
  import { handleColor, legMask, sceneToOffset, stanceToScene } from '$lib/animation/handles'

  interface Props {
    sky?: boolean
    orbit?: boolean
    panel?: boolean
    ground?: boolean
    // Drives the preview locally instead of following the robot's mode store.
    previewMode?: MotionModes
    // Draggable foot spheres; offsets are body-frame mm from DEFAULT_FEET.
    handles?: { onDrag: (leg: number, offsetMm: Vec3) => void; onDragEnd: () => void }
  }

  let {
    sky = true,
    orbit = false,
    panel = true,
    ground = true,
    previewMode,
    handles
  }: Props = $props()

  let sceneManager = $state(new SceneBuilder())
  let canvas: HTMLCanvasElement | null = $state(null)

  const angles = new Array(18).fill(0)
  const targetAngles = new Array(18).fill(0)

  const kinematics = new Kinematics(config)
  const STAND_Z = DEFAULT_FEET[0][2]
  // The body pose shown: recovered from the feet of the joint angles shown, whoever produced them. Before any arrive,
  // the robot stands level on its stance.
  let shownBody = levelBody()

  // While no robot sends its angles, the firmware's own motion code animates the model. The robot-hosted build is
  // always linked, and the build-time constant leaves the virtual robot out of it.
  let virtualRobot: VirtualRobot | undefined
  let sticks: Command = [0, 0, 0, 0, 0, 0, 0, 0]
  let pendingSeconds = 0
  let lastFrame = 0
  const MAX_FRAME_SECONDS = 0.05
  const modeIndex = (m: MotionModes) => Object.values(MotionModes).indexOf(m)
  const gaitIndex = (g: GaitType) => Object.values(GaitType).indexOf(g)

  let gui_panel: GUI

  let settings = {
    'Internal kinematic': true,
    'Auto orient robot': true,
    'Fix camera on robot': true,
    'Smooth motion': false
  }

  let jointAngles: Record<string, number> = {}
  let lastRobotPosition = new Vector3()
  let unsubscribers: (() => void)[] = []
  let destroyed = false
  let preview: AnimationPreview | null = null

  onMount(async () => {
    await populateModelCache()
    if (destroyed) return

    jointAngles = $jointNames.reduce((prev, cur) => ({ ...prev, [cur]: 0 }), {})

    createScene()
    if (panel) createPanel()

    unsubscribers = [
      outControllerData.subscribe(data => (sticks = data as Command)),
      dataBroker.on(AnglesData, data => {
        settings['Internal kinematic'] = false
        show(data.angles)
      }),
      // Without a link no angles arrive, so the local model has to animate the robot again.
      isLinked.subscribe(linked => {
        if (!linked) settings['Internal kinematic'] = true
      }),
      gait.subscribe(g => virtualRobot?.setGait(gaitIndex(g))),
      animationPreview.subscribe(value => (preview = value)),
      ...(previewMode ? [] : [mode.subscribe(m => virtualRobot?.setMode(modeIndex(m)))])
    ]
    if (import.meta.env.VITE_USE_HOST_NAME !== 'true') {
      loadVirtualRobot()
        .then(robot => {
          if (destroyed) return
          // Until the firmware sends a pose, the model rests in its stand rather than with its legs out straight.
          show(robot.standPose())
          robot.setMode(modeIndex(previewMode ?? $mode))
          robot.setGait(gaitIndex($gait))
          virtualRobot = robot
          lastFrame = performance.now()
        })
        .catch(error => console.warn('The 3D view cannot animate the robot on its own:', error))
    }
  })

  onDestroy(() => {
    destroyed = true
    unsubscribers.forEach(unsubscribe => unsubscribe())
    sceneManager.dispose()
    canvas?.remove()
    gui_panel?.destroy()
  })

  const createPanel = () => {
    gui_panel = new GUI({ width: 310 })
    gui_panel.close()
    gui_panel.domElement.id = 'three-gui-panel'

    const general = gui_panel.addFolder('General')
    general.add(settings, 'Internal kinematic').listen()
    general.add(settings, 'Auto orient robot')

    const limbs = gui_panel.addFolder('Limbs')
    limbs.add({ center: resetLimbs }, 'center')
    for (const name of $jointNames) {
      limbs
        .add(jointAngles, name, -Math.PI / 1.2, Math.PI / 1.2)
        .onChange(updateLimbs)
        .listen()
    }

    const visualization = gui_panel.addFolder('Visualization')
    visualization.add(settings, 'Smooth motion')
  }

  const resetLimbs = () => {
    for (const name of $jointNames) {
      jointAngles[name] = 0
    }
    requestMode(MotionModes.IDLE)
    updateLimbs()
  }

  const updateLimbs = () => {
    for (let i = 0; i < $jointNames.length; i++) {
      targetAngles[i] = jointAngles[$jointNames[i]]
    }
  }

  // Joint angles in degrees in the firmware's order, as the robot reports them and the virtual robot drives them.
  const show = (anglesDeg: ArrayLike<number>) => {
    const rad = Array.from(anglesDeg, degToRad)
    setTargetAngles(urdfJointOrder(rad))
    const legs = [0, 1, 2, 3, 4, 5].map(
      leg => rad.slice(leg * 3, leg * 3 + 3) as [number, number, number]
    )
    shownBody = kinematics.bodyOverFeet(legs, STAND_Z)
  }

  // Runs the virtual robot for the time since the last frame in whole control ticks, at most MAX_FRAME_SECONDS.
  const stepVirtualRobot = (robot: VirtualRobot) => {
    const now = performance.now()
    pendingSeconds = Math.min(pendingSeconds + (now - lastFrame) / 1000, MAX_FRAME_SECONDS)
    lastFrame = now
    for (; pendingSeconds >= TICK_SECONDS; pendingSeconds -= TICK_SECONDS) {
      robot.command(sticks)
      robot.tick()
    }
    if (robot.servosDriven()) show(robot.servoAngles())
  }

  const setTargetAngles = (angles: number[]) => {
    for (let i = 0; i < angles.length; i++) {
      targetAngles[i] = angles[i]
      jointAngles[$jointNames[i]] = targetAngles[i]
    }
  }

  $effect(() => {
    if (sceneManager.orbit) sceneManager.orbit.autoRotate = orbit
  })

  const createScene = () => {
    if (!canvas) return
    sceneManager
      .addRenderer({ antialias: true, canvas, alpha: true })
      .addPerspectiveCamera({ x: -0.5, y: 0.5, z: 1 })
      .addOrbitControls(20, 50, orbit)
      .addDirectionalLight({ x: 10, y: 20, z: 10, color: 0xffffff, intensity: 3 })
      .addAmbientLight({ color: 0xffffff, intensity: 0.5 })
      .addFogExp2(0xcccccc, 0.015)
      .addModel($model)
      .fillParent()
      .addRenderCb(render)
      .startRenderLoop()

    if (ground) sceneManager.addGroundPlane()

    if (sky) sceneManager.addSky()

    if (handles) createHandles()
  }

  const createHandles = () => {
    sceneManager.addFootHandles(DEFAULT_FEET.map(stanceToScene))
    const transform = sceneManager.transform!
    transform.addEventListener('objectChange', () => {
      const leg = sceneManager.handles.findIndex(handle => handle === transform.object)
      if (leg < 0) return
      handles?.onDrag(leg, sceneToOffset(transform.object!.position.toArray(), DEFAULT_FEET[leg]))
    })
    transform.addEventListener('mouseUp', () => handles?.onDragEnd())
  }

  const updateHandles = () => {
    const feet = preview?.handleFeet ?? preview?.body.feet ?? DEFAULT_FEET
    const transform = sceneManager.transform
    const dragged = transform?.dragging ? transform.object : undefined
    sceneManager.handles.forEach((handle, leg) => {
      if (handle !== dragged) sceneManager.setHandlePosition(leg, stanceToScene(feet[leg]))
      handle.material.color.setHex(handleColor(legMask(preview?.mask ?? 0, leg)))
    })
  }

  const smooth = (start: number, end: number, amount: number) => {
    return settings['Smooth motion'] ? lerp(start, end, amount) : end
  }

  const update_camera = (robot: URDFRobot) => {
    if (!settings['Fix camera on robot']) return
    const delta = robot.position.clone().sub(lastRobotPosition)
    sceneManager.orbit.target.add(delta)
    sceneManager.camera.position.add(delta)
    lastRobotPosition.copy(robot.position)
  }

  const calculateGroundHeight = (body: body_state_t) => {
    const feet = body.feet
    if (!feet || feet.length === 0) return 0

    let minZ = feet[0][2]
    for (let i = 1; i < feet.length; i++) {
      if (feet[i][2] < minZ) {
        minZ = feet[i][2]
      }
    }

    const feetDepth = Math.max(0, STAND_Z - minZ)

    const bodyHeight = body.zm / 12
    return -bodyHeight + 0.5 + feetDepth
  }

  const orient_robot = (robot: URDFRobot, body: body_state_t) => {
    if (!settings['Auto orient robot']) return

    const ORIENT_SMOOTH = 0.1
    const POSITION_SCALE = 1 / 12

    const cmdXm = body.xm ?? 0
    const cmdYm = body.ym ?? 0
    const cmdYaw = -(body.psi ?? 0)
    const cmdRoll = body.omega ?? 0
    const cmdPitch = -(body.phi ?? 0)

    const cumulativeYaw = body.cumulative_yaw ?? 0
    const cumulativePitch = body.cumulative_pitch ?? 0
    const cumulativeRoll = body.cumulative_roll ?? 0
    const cumulativeXm = body.cumulative_x ?? 0
    const cumulativeZm = body.cumulative_z ?? body.cumulative_y ?? 0

    const totalYaw = cmdYaw + cumulativeYaw

    const cosYaw = Math.cos(totalYaw)
    const sinYaw = Math.sin(totalYaw)

    const rotatedXm = cmdXm * cosYaw - cmdYm * sinYaw
    const rotatedYm = cmdXm * sinYaw + cmdYm * cosYaw

    robot.position.z = smooth(
      robot.position.z,
      (-rotatedXm + cumulativeXm) * POSITION_SCALE,
      ORIENT_SMOOTH
    )

    robot.position.x = smooth(
      robot.position.x,
      (-rotatedYm + cumulativeZm) * POSITION_SCALE,
      ORIENT_SMOOTH
    )

    robot.position.y = smooth(robot.position.y, calculateGroundHeight(body), ORIENT_SMOOTH)

    robot.rotation.z = smooth(robot.rotation.z, totalYaw + Math.PI / 2, ORIENT_SMOOTH)

    robot.rotation.x = smooth(
      robot.rotation.x,
      -Math.PI / 2 + cmdPitch * cosYaw - cmdRoll * sinYaw + cumulativePitch,
      ORIENT_SMOOTH
    )

    robot.rotation.y = smooth(
      robot.rotation.y,
      cmdPitch * sinYaw + cmdRoll * cosYaw + cumulativeRoll,
      ORIENT_SMOOTH
    )
  }

  const render = () => {
    const robot = sceneManager.model
    if (!robot) return
    if (preview) {
      setTargetAngles(urdfJointOrder(preview.angles))
    } else if (settings['Internal kinematic'] && virtualRobot) {
      stepVirtualRobot(virtualRobot)
    }
    update_camera(robot)
    orient_robot(robot, preview?.body ?? shownBody)
    if (handles) updateHandles()

    for (let i = 0; i < $jointNames.length; i++) {
      angles[i] = smooth(robot.joints[$jointNames[i]].angle as number, targetAngles[i], 0.1)
      robot.joints[$jointNames[i]].setJointValue(angles[i])
    }
  }
</script>

<svelte:window onresize={sceneManager.fillParent} />

<canvas bind:this={canvas}></canvas>
