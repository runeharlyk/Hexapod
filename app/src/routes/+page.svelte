<script lang="ts">
  import { goto } from '$app/navigation'
  import { resolve } from '$app/paths'
  import Visualization from '$lib/components/Visualization.svelte'
  import { websocket } from '$lib/transport/websocket-adapter'
  import { onMount } from 'svelte'

  const base = resolve('/')

  const connected = websocket.connected

  onMount(() => {
    connected.subscribe(isConnected => {
      if (isConnected) {
        goto(`${base}controller`)
      }
    })
  })
</script>

<div class="hero bg-base-100 h-screen">
  <div class="card md:card-side bg-base-200 shadow-2xl flex justify-center items-center">
    <div class="w-64 h-64">
      <Visualization sky={false} orbit panel={false} ground={false} />
    </div>
    <div class="card-body w-80">
      <h2 class="card-title text-center text-2xl">Begin you journey</h2>
      <p class="py-6 text-center"></p>
      <a class="btn btn-primary" href={resolve($connected ? '/controller' : '/connection')}>
        Connect to robot
      </a>
    </div>
  </div>
</div>
