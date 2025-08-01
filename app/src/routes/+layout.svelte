<script lang="ts">
  import { onMount } from 'svelte'
  import { page } from '$app/state'
  import { Modals, modals } from 'svelte-modals'
  import Toast from '$lib/components/toasts/Toast.svelte'
  import { fade } from 'svelte/transition'
  import '../app.css'
  import Menu from '../lib/components/menu/Menu.svelte'
  import Statusbar from '../lib/components/statusbar/statusbar.svelte'
  import { kinematicData, mode, outControllerData, servoAnglesOut, gait } from '$lib/stores'
  import { MotionModes } from '$lib/motion'
  import { dataBroker } from '$lib/transport/databroker'
  import { ble } from '$lib/transport/ble-adapter'
  import { websocket } from '$lib/transport/websocket-adapter'
  import { MessageTopic } from '$lib/interfaces/transport.interface'

  interface Props {
    children?: import('svelte').Snippet
  }

  dataBroker.addTransport(ble)
  dataBroker.addTransport(websocket)

  let { children }: Props = $props()

  onMount(async () => {
    await websocket.connect()

    outControllerData.subscribe(data => dataBroker.emit(MessageTopic.COMMAND, data))

    mode.subscribe(data =>
      dataBroker.emit(MessageTopic.MODE, Object.values(MotionModes).indexOf(data))
    )
    gait.subscribe(data => dataBroker.emit(MessageTopic.GAIT, data))
    servoAnglesOut.subscribe(data => {})
    kinematicData.subscribe(data => dataBroker.emit(MessageTopic.COMMAND, data))
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
      onclick={modals.closeAll}>
    </div>
  {/snippet}
</Modals>

<Toast />
