<script lang="ts">
  import { focusTrap } from 'svelte-focus-trap'
  import { fly } from 'svelte/transition'
  import { exitBeforeEnter, modals, type ModalProps } from 'svelte-modals'
  import { Cancel, Check } from '$lib/components/icons'

  let { isOpen, ssid, onConfirm }: ModalProps & { ssid: string } = $props()
  let password = $state('')

  const connect = () => {
    onConfirm(password)
    modals.close()
  }
</script>

{#if isOpen}
  <div
    role="dialog"
    class="pointer-events-none fixed inset-0 z-50 flex items-center justify-center"
    transition:fly={{ y: 50 }}
    use:exitBeforeEnter
    use:focusTrap
  >
    <div
      class="rounded-box bg-base-100 pointer-events-auto flex min-w-fit max-w-md flex-col justify-between p-4 shadow-lg"
    >
      <h2 class="text-base-content text-start text-2xl font-bold">Connect to {ssid}</h2>
      <div class="divider my-2"></div>
      <!-- svelte-ignore a11y_autofocus -->
      <input
        type="password"
        class="input input-bordered w-full"
        placeholder="Password"
        autofocus
        bind:value={password}
        onkeydown={e => e.key === 'Enter' && connect()}
      />
      <div class="divider my-2"></div>
      <div class="flex justify-end gap-2">
        <button class="btn btn-error inline-flex items-center" onclick={() => modals.close()}>
          <Cancel class="mr-2 h-5 w-5" /><span>Cancel</span>
        </button>
        <button class="btn btn-primary inline-flex items-center" onclick={connect}>
          <Check class="mr-2 h-5 w-5" /><span>Connect</span>
        </button>
      </div>
    </div>
  </div>
{/if}
