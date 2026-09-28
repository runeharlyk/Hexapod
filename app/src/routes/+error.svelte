<script lang="ts">
  import { goto } from '$app/navigation'
  import { resolve } from '$app/paths'
  import { onDestroy, onMount } from 'svelte'

  let redirectTimer: ReturnType<typeof setTimeout> | undefined

  onMount(() => {
    redirectTimer = setTimeout(() => goto(resolve('/')), 3000)
  })

  onDestroy(() => clearTimeout(redirectTimer))
</script>

<div class="flex justify-center items-center w-full h-full">
  <h1 class="text-4xl">404 - Page not found</h1>
  <p>You will be redirected to the home page in 3 seconds</p>
</div>
