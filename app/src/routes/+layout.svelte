<script lang="ts">
  import { onDestroy, onMount } from 'svelte'
  import { page } from '$app/state'
  import { Modals, modals } from 'svelte-modals'
  import Toast from '$lib/components/toasts/Toast.svelte'
  import { fade } from 'svelte/transition'
  import '../app.css'
  import Menu from '../lib/components/menu/Menu.svelte'
  import Statusbar from '../lib/components/statusbar/statusbar.svelte'
  import { mode, outControllerData, gait } from '$lib/stores'
  import { MotionModes } from '$lib/motion'
  import { dataBroker } from '$lib/transport/databroker'
  import { ble } from '$lib/transport/ble-adapter'
  import { websocket } from '$lib/transport/websocket-adapter'
  import { MessageTopic } from '$lib/interfaces/transport.interface'
  import { GaitType } from '$lib/gait'
  import { throttler } from '$lib/utilities'

  interface Props {
    children?: import('svelte').Snippet
  }

  dataBroker.addTransport(ble)
  dataBroker.addTransport(websocket)

  const throttle = new throttler()
  const COMMAND_HEARTBEAT_MS = 250
  let lastCommand = [0, 0, 0, 0, 0, 0, 0, 0]
  let currentMode = MotionModes.DEACTIVATED
  let commandHeartbeatId: ReturnType<typeof setInterval> | undefined

  let { children }: Props = $props()

  onMount(async () => {
    await websocket.connect()

    outControllerData.subscribe(data => {
      lastCommand = [...data]
      throttle.throttle(() => dataBroker.emit(MessageTopic.COMMAND, data), 40)
    })

    dataBroker.on<number>(MessageTopic.MODE, data => {
      const nextMode = Object.values(MotionModes)[data]
      if (nextMode !== undefined) {
        mode.set(nextMode)
      }
    })

    dataBroker.on<number>(MessageTopic.GAIT, data => {
      const nextGait = Object.values(GaitType)[data]
      if (nextGait !== undefined) {
        gait.set(nextGait)
      }
    })

    mode.subscribe(value => {
      currentMode = value
    })

    commandHeartbeatId = setInterval(() => {
      if (currentMode !== MotionModes.STAND && currentMode !== MotionModes.WALK) return
      dataBroker.send(MessageTopic.COMMAND, lastCommand)
    }, COMMAND_HEARTBEAT_MS)
  })

  onDestroy(() => {
    if (commandHeartbeatId) clearInterval(commandHeartbeatId)
  })

  let menuOpen = $state(false)
</script>

<svelte:head>
  <title>{page.data.title}</title>
</svelte:head>

<div class="drawer">
  <input id="main-menu" type="checkbox" class="drawer-toggle" bind:checked={menuOpen} />
  <div class="drawer-content flex flex-col">
    <!-- Status bar content here -->
    <Statusbar />

    <!-- Main page content here -->
    {@render children?.()}
  </div>
  <!-- Side Navigation -->
  <div class="drawer-side z-30 shadow-lg">
    <label for="main-menu" class="drawer-overlay"></label>
    <Menu menuClicked={() => (menuOpen = false)} />
  </div>
</div>

<Modals>
  <!-- svelte-ignore a11y_click_events_have_key_events -->
  <!-- svelte-ignore a11y_no_static_element_interactions -->
  {#snippet backdrop()}
    <div
      class="fixed inset-0 z-40 max-h-full max-w-full bg-black/20 backdrop-blur-sm"
      transition:fade
      onclick={modals.closeAll}
    ></div>
  {/snippet}
</Modals>

<Toast />
