<script lang="ts">
  import { onDestroy, onMount } from 'svelte'
  import { jointNames, model, outControllerData } from '$lib/stores'
  import { gait, mode } from '$lib/robot/state'
  import { populateModelCache } from '$lib/utilities'
  import SceneBuilder from '$lib/sceneBuilder'
  import { GUI } from 'three/addons/libs/lil-gui.module.min.js'
  import type { URDFRobot } from 'urdf-loader'
  import { degToRad, lerp } from 'three/src/math/MathUtils'
  import { GaitType } from '$lib/gait'
  import { MotionModes, modeToWire, urdfJointOrder } from '$lib/motion'
  import { dataBroker } from '$lib/transport/databroker'
  import { AnglesData, IMUData } from '$lib/platform_shared/message'
  import { Quaternion, Vector3 } from 'three'
  import { isLinked } from '$lib/stores/link'
  import { animationPreview, type AnimationPreview } from '$lib/stores/animation'
  import Kinematics from '$lib/kinematic'
  import { ShownRobot } from '$lib/ground-placement'
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
  import { JointDrive, drivingJoints, withinTravel } from '$lib/joint-drive'
  import { JointDrag } from '$lib/joint-drag'
  import type { URDFJoint } from 'urdf-loader'

  // A view of the robot: it reads the robot's angles, mode and gait and never changes the robot's mode, which only the
  // controller does, and its panel poses the drawn model alone. The controller page's view, given driveJoints, is the
  // one that sends anything: while the robot idles, a joint dragged with the pointer drives that servo (joint-drive.ts).
  interface Props {
    sky?: boolean
    orbit?: boolean
    panel?: boolean
    ground?: boolean
    // Drives the preview locally instead of following the robot's mode store.
    previewMode?: MotionModes
    // Draggable foot spheres; offsets are body-frame mm from DEFAULT_FEET.
    handles?: { onDrag: (leg: number, offsetMm: Vec3) => void; onDragEnd: () => void }
    // The controller page's view: its joints drive the robot's servos while drivingJoints holds.
    driveJoints?: boolean
  }

  let {
    sky = true,
    orbit = false,
    panel = true,
    ground = true,
    previewMode,
    handles,
    driveJoints = false
  }: Props = $props()

  let sceneManager = $state(new SceneBuilder())
  let canvas: HTMLCanvasElement | null = $state(null)

  const angles = new Array(18).fill(0)
  const targetAngles = new Array(18).fill(0)

  type LegRad = [number, number, number]
  const shown = new ShownRobot(new Kinematics(config))

  // While no robot sends its angles, the firmware's own motion code animates the model. The robot-hosted build is
  // always linked, and the build-time constant leaves the virtual robot out of it.
  let virtualRobot: VirtualRobot | undefined
  let sticks: Command = [0, 0, 0, 0, 0, 0, 0, 0, 0]
  let pendingSeconds = 0
  let lastFrame = 0
  const MAX_FRAME_SECONDS = 0.05
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
  let jointDrive: JointDrive | undefined
  let jointDrag: JointDrag | undefined

  onMount(async () => {
    await populateModelCache()
    if (destroyed) return

    jointAngles = $jointNames.reduce((prev, cur) => ({ ...prev, [cur]: 0 }), {})

    createScene()
    if (panel) createPanel()
    if (driveJoints) {
      jointDrive = new JointDrive(drivingJoints)
      unsubscribers.push(drivingJoints.subscribe(followDriving))
    }

    unsubscribers.push(
      outControllerData.subscribe(data => (sticks = data)),
      dataBroker.on(AnglesData, data => {
        settings['Internal kinematic'] = false
        show(data.angles)
      }),
      dataBroker.on(IMUData, data => shown.measure(data)),
      // Without a link no angles arrive, so the local model has to animate the robot again.
      isLinked.subscribe(linked => {
        shown.link(linked)
        if (!linked) settings['Internal kinematic'] = true
      }),
      gait.subscribe(g => virtualRobot?.setGait(gaitIndex(g))),
      animationPreview.subscribe(value => (preview = value)),
      // A mode this app does not know is none the local model can animate; it keeps the last one.
      ...(previewMode ? [] : [mode.subscribe(m => m && virtualRobot?.setMode(modeToWire(m)))])
    )
    if (import.meta.env.VITE_USE_HOST_NAME !== 'true') {
      loadVirtualRobot()
        .then(robot => {
          if (destroyed) return
          // Until the firmware sends a pose, the model rests in its stand rather than with its legs out straight.
          show(robot.standPose())
          const shownMode = previewMode ?? $mode
          if (shownMode) robot.setMode(modeToWire(shownMode))
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
    jointDrag?.dispose()
    jointDrive?.dispose()
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
    limbs.add({ center: centerLimbs }, 'center')
    for (const name of $jointNames) {
      limbs
        .add(jointAngles, name, -Math.PI / 1.2, Math.PI / 1.2)
        .onChange(poseByHand)
        .listen()
    }

    const visualization = gui_panel.addFolder('Visualization')
    visualization.add(settings, 'Smooth motion')
  }

  const centerLimbs = () => {
    for (const name of $jointNames) {
      jointAngles[name] = 0
    }
    poseByHand()
  }

  // The virtual robot stops, so the pose holds until 'Internal kinematic' is ticked again; angles the robot sends
  // still replace it, since they are the truth.
  const poseByHand = () => {
    settings['Internal kinematic'] = false
    for (let i = 0; i < $jointNames.length; i++) {
      targetAngles[i] = jointAngles[$jointNames[i]]
    }
  }

  // Dragging a joint takes effect only while the robot idles over a link; leaving IDLE or losing the link ends it.
  const followDriving = (driving: boolean) => {
    if (!driving) {
      jointDrag?.dispose()
      jointDrag = undefined
    } else if (!jointDrag && sceneManager.model) {
      jointDrag = new JointDrag(
        sceneManager.model,
        sceneManager.camera,
        sceneManager.renderer.domElement,
        turnJoint
      )
    }
  }

  // The model shows the joint where the drag put it, within the servo's travel and the model's limits, as the panel's
  // pose by hand does, and the robot is sent that angle.
  const turnJoint = (joint: URDFJoint, angle: number) => {
    const index = $jointNames.indexOf(joint.urdfName)
    if (index < 0) return
    joint.setJointValue(withinTravel(index, angle))
    settings['Internal kinematic'] = false
    targetAngles[index] = joint.angle
    jointAngles[joint.urdfName] = joint.angle
    sceneManager.invalidate()
    jointDrive?.drag(index, joint.angle)
  }

  // Joint angles in degrees in the firmware's order, as the robot reports them and the virtual robot drives them.
  const show = (anglesDeg: ArrayLike<number>) => {
    const rad = Array.from(anglesDeg, degToRad)
    setTargetAngles(urdfJointOrder(rad))
    shown.show(legsOf(rad))
  }

  const legsOf = (rad: readonly number[]): LegRad[] =>
    [0, 1, 2, 3, 4, 5].map(leg => rad.slice(leg * 3, leg * 3 + 3) as LegRad)

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
      sceneManager.setHandleColor(leg, handleColor(legMask(preview?.mask ?? 0, leg)))
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

  const ORIENT_SMOOTH = 0.1

  const placedQuaternion = new Quaternion()
  const placedPosition = new Vector3()

  const placeRobot = (robot: URDFRobot) => {
    if (!settings['Auto orient robot']) return
    const { orientation, position } = shown.placement(
      preview && { legs: legsOf(preview.angles), yaw: -preview.body.psi }
    )
    placedQuaternion.copy(robot.quaternion)
    placedPosition.copy(robot.position)
    if (settings['Smooth motion']) {
      robot.quaternion.slerp(orientation, ORIENT_SMOOTH)
      robot.position.lerp(position, ORIENT_SMOOTH)
    } else {
      robot.quaternion.copy(orientation)
      robot.position.copy(position)
    }
    if (!robot.quaternion.equals(placedQuaternion) || !robot.position.equals(placedPosition))
      sceneManager.invalidate()
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
    placeRobot(robot)
    if (handles) updateHandles()

    for (let i = 0; i < $jointNames.length; i++) {
      angles[i] = smooth(robot.joints[$jointNames[i]].angle as number, targetAngles[i], 0.1)
      if (robot.joints[$jointNames[i]].setJointValue(angles[i])) sceneManager.invalidate()
    }
  }
</script>

<svelte:window onresize={sceneManager.fillParent} />

<canvas bind:this={canvas}></canvas>
