<script lang="ts">
  import { onDestroy, onMount } from 'svelte'
  import { get } from 'svelte/store'
  import { Box3, Group, Mesh, Object3D, Vector3 } from 'three'
  import type { URDFRobot } from 'urdf-loader'
  import SceneBuilder from '$lib/sceneBuilder'
  import { driveSimulation, simulation } from '$lib/stores/drive'
  import { loadModel } from '$lib/utilities/model-utilities'
  import { MotionModes, urdfJointOrder } from '$lib/motion'
  import { GaitType } from '$lib/gait'
  import { loadSimulation } from '$lib/simulation/load'
  import { HexapodSim } from '$lib/simulation/hexapod-sim'
  import type { VirtualRobot } from '$lib/simulation/virtual-robot'
  import { mountModel } from '$lib/simulation/model-mount'

  // The 3D view draws 12 mm per scene unit.
  const SCENE_UNITS_PER_METRE = 1000 / 12

  let canvas: HTMLCanvasElement
  let status = $state<'loading' | 'running' | 'error'>('loading')
  let error = $state('')
  let fallen = $state(false)

  const sceneManager = new SceneBuilder()
  let resize: ResizeObserver | undefined
  let sim: HexapodSim | undefined
  let robot: VirtualRobot | undefined
  let model: URDFRobot | undefined
  let jointNames: string[] = []
  const pose = new Object3D()
  let lastFrame = 0
  let destroyed = false
  let unsubscribers: (() => void)[] = []
  const followed = new Vector3()
  const robotWorld = new Vector3()

  async function start() {
    status = 'loading'
    try {
      const [{ mujoco, robot: loaded, sceneXml }, urdf] = await Promise.all([
        loadSimulation(),
        loadModel()
      ])
      if (destroyed) return
      if (urdf.isErr()) throw new Error(urdf.inner)
      robot = loaded
      sim = new HexapodSim(mujoco, sceneXml, robot)
      ;[model, jointNames] = urdf.inner
      placeModel(model)
      unsubscribers = [
        simulation.mode.subscribe(m => robot?.setMode(Object.values(MotionModes).indexOf(m))),
        simulation.gait.subscribe(g => robot?.setGait(Object.values(GaitType).indexOf(g)))
      ]
      lastFrame = performance.now()
      status = 'running'
    } catch (cause) {
      if (destroyed) return
      error = cause instanceof Error ? cause.message : String(cause)
      status = 'error'
    }
  }

  /**
   * MuJoCo's world (metres, z up) maps onto the scene through `world`, and the base pose goes onto `pose` as is.
   * The URDF model sits in `pose` in the MuJoCo body frame, lowered so its feet meet the floor at spawn.
   */
  function placeModel(urdf: URDFRobot) {
    const world = new Group()
    world.rotation.x = -Math.PI / 2
    world.scale.setScalar(SCENE_UNITS_PER_METRE)
    mountModel(urdf, pose)
    world.add(pose)
    sceneManager.scene.add(world)
    update()
    // Box3 reads world matrices but refreshes none of the parents', so the scene's are brought up to date first.
    sceneManager.scene.updateMatrixWorld(true)
    urdf.position.z = -new Box3().setFromObject(urdf).min.y / SCENE_UNITS_PER_METRE
  }

  function update() {
    if (!sim || !model) return
    const [x, y, z] = sim.basePosition()
    const [w, qx, qy, qz] = sim.baseQuaternion()
    pose.position.set(x, y, z)
    pose.quaternion.set(qx, qy, qz, w)
    const angles = urdfJointOrder(sim.jointAngles())
    jointNames.forEach((name, i) => model!.joints[name]?.setJointValue(angles[i]))
  }

  function frame() {
    if (!sim || !robot) return
    const now = performance.now()
    robot.command(get(simulation.command))
    sim.step((now - lastFrame) / 1000)
    lastFrame = now
    update()
    followRobot()
    fallen = sim.hasFallen()
  }

  // The camera travels with the robot, so a walking robot stays in view at the chosen angle.
  function followRobot() {
    pose.getWorldPosition(robotWorld)
    robotWorld.y = 0
    const delta = robotWorld.clone().sub(followed)
    sceneManager.camera.position.add(delta)
    sceneManager.orbit.target.add(delta)
    followed.copy(robotWorld)
  }

  const reset = () => {
    sim?.reset()
    robot?.setMode(Object.values(MotionModes).indexOf(get(simulation.mode)))
    lastFrame = performance.now()
  }

  onMount(() => {
    // The view is a robot of its own: while it is shown the controls drive it, and never the real robot.
    driveSimulation(true)
    sceneManager
      .addRenderer({ antialias: true, canvas, alpha: true })
      .addPerspectiveCamera({ x: -30, y: 25, z: 40 })
      .addOrbitControls(5, 80, false)
      .addDirectionalLight({ x: 10, y: 20, z: 10, color: 0xffffff, intensity: 3 })
      .addAmbientLight({ color: 0xffffff, intensity: 0.5 })
      .addFogExp2(0xcccccc, 0.015)
      .addGroundPlane()
      .fillParent()
      .addRenderCb(frame)
      .startRenderLoop()

    const parent = canvas.parentElement
    if (parent) {
      resize = new ResizeObserver(() => sceneManager.fillParent())
      resize.observe(parent)
    }
    void start()
  })

  onDestroy(() => {
    destroyed = true
    driveSimulation(false)
    unsubscribers.forEach(unsubscribe => unsubscribe())
    sceneManager.dispose()
    resize?.disconnect()
    sim?.dispose()
    model?.traverse(part => {
      if (!(part instanceof Mesh)) return
      part.geometry.dispose()
      for (const material of [part.material].flat()) material.dispose()
    })
  })
</script>

<div class="relative h-full w-full">
  <canvas bind:this={canvas}></canvas>

  <span class="bg-base-100/70 rounded-box absolute top-2 left-2 px-2 py-1 text-xs opacity-80">
    Simulation: the firmware's motion code, approximate physics
  </span>

  {#if status === 'loading'}
    <div class="absolute inset-0 flex flex-col items-center justify-center gap-2">
      <span class="loading loading-spinner loading-lg text-primary"></span>
      <span class="text-sm">Loading the physics engine (10 MB)</span>
    </div>
  {:else if status === 'error'}
    <div role="alert" class="alert alert-error absolute inset-x-4 top-12">
      <span>The simulation could not start: {error}</span>
      <button class="btn btn-sm" onclick={start}>Retry</button>
    </div>
  {:else if fallen}
    <div
      role="status"
      class="bg-base-100/80 rounded-box absolute top-12 left-1/2 flex -translate-x-1/2 items-center gap-3 px-3 py-2 text-sm"
    >
      The hexapod fell over
      <button class="btn btn-sm btn-primary" onclick={reset}>Reset</button>
    </div>
  {/if}
</div>
