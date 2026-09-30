import {
  Mesh,
  PerspectiveCamera,
  PlaneGeometry,
  Scene,
  WebGLRenderer,
  AmbientLight,
  DirectionalLight,
  PCFSoftShadowMap,
  Vector3,
  FogExp2,
  CanvasTexture,
  type ColorRepresentation,
  type WebGLRendererParameters,
  EquirectangularReflectionMapping,
  ACESFilmicToneMapping,
  MathUtils,
  Group,
  MeshBasicMaterial,
  RepeatWrapping,
  Raycaster,
  SphereGeometry,
  Vector2,
  type Object3D
} from 'three'
import { Sky } from 'three/addons/objects/Sky.js'
import { OrbitControls } from 'three/addons/controls/OrbitControls.js'
import { TransformControls } from 'three/examples/jsm/controls/TransformControls.js'
import { Reflector } from 'three/addons/objects/Reflector.js'
import { type URDFRobot } from 'urdf-loader'
import { sunCalculator } from './utilities/position-utilities'
import { HANDLE_NEUTRAL } from './animation/handles'
import type { Vec3 } from './animation/model'

interface position {
  x?: number
  y?: number
  z?: number
}

interface light {
  color?: ColorRepresentation
  intensity?: number
}

type directionalLight = position & light

export default class SceneBuilder {
  public scene: Scene
  public camera!: PerspectiveCamera
  public ground!: Mesh
  public renderer!: WebGLRenderer
  public orbit!: OrbitControls
  public callback: (() => void) | undefined
  public model!: URDFRobot
  private isLoaded: boolean = false
  private shadowTimers: ReturnType<typeof setTimeout>[] = []
  sky!: Sky
  public modelGroup!: Group
  public handles: Mesh<SphereGeometry, MeshBasicMaterial>[] = []
  public transform: TransformControls | undefined
  private raycaster = new Raycaster()

  constructor() {
    this.scene = new Scene()
    if (this.scene.environment?.mapping) {
      this.scene.environment.mapping = EquirectangularReflectionMapping
    }
    return this
  }

  public addRenderer = (parameters?: WebGLRendererParameters) => {
    this.renderer = new WebGLRenderer(parameters)
    this.renderer.outputColorSpace = 'srgb'
    this.renderer.shadowMap.enabled = true
    this.renderer.shadowMap.type = PCFSoftShadowMap
    this.renderer.toneMapping = ACESFilmicToneMapping
    this.renderer.toneMappingExposure = 0.85
    if (!parameters?.canvas) document.body.appendChild(this.renderer.domElement)
    return this
  }

  public addSky = () => {
    this.sky = new Sky()
    this.sky.scale.setScalar(450000)
    this.scene.add(this.sky)
    const effectController = {
      turbidity: 10,
      rayleigh: 3,
      mieCoefficient: 0.005,
      mieDirectionalG: 0.7,
      elevation: sunCalculator.calculateSunElevation(),
      azimuth: 200,
      exposure: this.renderer.toneMappingExposure
    }
    const uniforms = this.sky.material.uniforms
    uniforms['turbidity'].value = effectController.turbidity
    uniforms['rayleigh'].value = effectController.rayleigh
    uniforms['mieCoefficient'].value = effectController.mieCoefficient
    uniforms['mieDirectionalG'].value = effectController.mieDirectionalG
    this.renderer.toneMappingExposure = 0.5
    const phi = MathUtils.degToRad(90 - effectController.elevation)
    const theta = MathUtils.degToRad(effectController.azimuth)
    const sun = new Vector3()

    sun.setFromSphericalCoords(1, phi, theta)
    uniforms['sunPosition'].value.copy(sun)
    return this
  }

  public addPerspectiveCamera = (options: position) => {
    this.camera = new PerspectiveCamera()
    this.camera.position.set(options.x ?? 0, options.y ?? 2.7, options.z ?? 0)
    this.scene.add(this.camera)
    return this
  }

  public addGroundPlane = (options?: position) => {
    const checkerboardTexture = this.createCheckerboardTexture(1024, 2)
    checkerboardTexture.wrapS = RepeatWrapping
    checkerboardTexture.wrapT = RepeatWrapping
    checkerboardTexture.repeat.set(100, 100)
    const checkerboardMat = new MeshBasicMaterial({
      map: checkerboardTexture,
      opacity: 0.1,
      transparent: true
    })

    const plane = new PlaneGeometry(400, 400)

    this.ground = new Mesh(plane, checkerboardMat)
    this.ground.rotation.x = -Math.PI / 2
    this.ground.position.set(options?.x ?? 0, options?.y ?? 0.01, options?.z ?? 0)
    this.ground.receiveShadow = true
    this.scene.add(this.ground)

    const mirror = new Reflector(plane, {
      clipBias: 0.003,
      textureWidth: window.innerWidth * window.devicePixelRatio,
      textureHeight: window.innerHeight * window.devicePixelRatio,
      color: 0x00bfff
    })
    mirror.rotateX(-Math.PI / 2)
    this.scene.add(mirror)

    return this
  }

  public addOrbitControls = (minDistance: number, maxDistance: number, autoRotate = true) => {
    this.orbit = new OrbitControls(this.camera, this.renderer.domElement)
    this.orbit.maxDistance = maxDistance
    this.orbit.minDistance = minDistance + (maxDistance - minDistance) / 2
    this.orbit.autoRotate = autoRotate
    this.orbit.update()
    this.orbit.minDistance = minDistance
    return this
  }

  public addAmbientLight = (options: light) => {
    const ambientLight = new AmbientLight(options.color, options.intensity)
    this.scene.add(ambientLight)
    return this
  }

  public addDirectionalLight = (options: directionalLight) => {
    const directionalLight = new DirectionalLight(options.color, options.intensity)
    directionalLight.castShadow = true
    directionalLight.shadow.camera.top = 10
    directionalLight.shadow.camera.bottom = -10
    directionalLight.shadow.camera.right = 10
    directionalLight.shadow.camera.left = -10
    directionalLight.shadow.mapSize.set(4096, 4096)

    directionalLight.position.set(options.x ?? 0, options.y ?? 0, options.z ?? 0)
    this.scene.add(directionalLight)
    return this
  }

  private createCheckerboardTexture = (size: number, squares: number) => {
    const canvas = document.createElement('canvas')
    canvas.width = size
    canvas.height = size
    const context = canvas.getContext('2d')

    const squareSize = size / squares

    for (let y = 0; y < squares; y++) {
      for (let x = 0; x < squares; x++) {
        context!.fillStyle = (x + y) % 2 === 0 ? '#ffffff' : '#000000'
        context!.fillRect(x * squareSize, y * squareSize, squareSize, squareSize)
      }
    }

    const texture = new CanvasTexture(canvas)
    texture.wrapS = texture.wrapT = RepeatWrapping
    texture.anisotropy = 16
    return texture
  }

  public addFogExp2 = (color: ColorRepresentation, density?: number) => {
    this.scene.fog = new FogExp2(color, density)
    return this
  }

  public fillParent = () => {
    const parentElement = this.renderer?.domElement.parentElement
    if (parentElement) this.handleResize(parentElement.clientWidth, parentElement.clientHeight)
    return this
  }

  public handleResize = (width = window.innerWidth, height = window.innerHeight) => {
    this.renderer.setSize(width, height)
    this.renderer.setPixelRatio(window.devicePixelRatio)
    this.camera.aspect = width / height
    this.camera.updateProjectionMatrix()
    return this
  }

  public addRenderCb = (callback: () => void) => {
    this.callback = callback
    return this
  }

  public startRenderLoop = () => {
    this.renderer.setAnimationLoop(() => {
      this.renderer.render(this.scene, this.camera)
      this.orbit.update()
      this.handleRobotShadow()
      this.callback?.()
    })
    return this
  }

  public addModel = (model: URDFRobot) => {
    this.modelGroup = new Group()
    this.modelGroup.add(model)
    this.model = model
    this.scene.add(this.modelGroup)
    return this
  }

  public addFootHandles = (positions: Vec3[]): Mesh[] => {
    const geometry = new SphereGeometry(0.6, 16, 12)
    this.handles = positions.map(position => {
      const handle = new Mesh(geometry, new MeshBasicMaterial({ color: HANDLE_NEUTRAL }))
      handle.position.set(...position)
      this.scene.add(handle)
      return handle
    })

    this.transform = new TransformControls(this.camera, this.renderer.domElement)
    this.transform.setMode('translate')
    this.transform.addEventListener('dragging-changed', event => {
      this.orbit.enabled = !event.value
    })
    this.scene.add(this.transform)
    // Registered after TransformControls' own listener, so a press on the gizmo is already dragging.
    this.renderer.domElement.addEventListener('pointerdown', this.pickHandle)
    return this.handles
  }

  public attachTransform = (target: Object3D | null) => {
    if (target) this.transform?.attach(target)
    else this.transform?.detach()
  }

  public setHandlePosition = (leg: number, position: Vec3) => {
    this.handles[leg]?.position.set(...position)
  }

  private pickHandle = (event: PointerEvent) => {
    if (event.button !== 0 || this.transform?.dragging) return
    const rect = this.renderer.domElement.getBoundingClientRect()
    const pointer = new Vector2(
      ((event.clientX - rect.left) / rect.width) * 2 - 1,
      -((event.clientY - rect.top) / rect.height) * 2 + 1
    )
    this.raycaster.setFromCamera(pointer, this.camera)
    const [hit] = this.raycaster.intersectObjects(this.handles, false)
    this.attachTransform(hit?.object ?? null)
  }

  public dispose = () => {
    this.shadowTimers.forEach(clearTimeout)
    this.shadowTimers = []
    this.callback = undefined
    this.renderer?.setAnimationLoop(null)
    this.renderer?.domElement.removeEventListener('pointerdown', this.pickHandle)
    this.transform?.dispose()
    this.handles.forEach(handle => handle.material.dispose())
    this.handles[0]?.geometry.dispose()
    this.orbit?.dispose()
    this.renderer?.dispose()
  }

  private handleRobotShadow = () => {
    if (this.isLoaded) return
    const intervalId = setInterval(() => this.model?.traverse(c => (c.castShadow = true)), 10)
    this.shadowTimers.push(
      intervalId,
      setTimeout(() => clearInterval(intervalId), 1000)
    )
    this.isLoaded = true
  }
}
