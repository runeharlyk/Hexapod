<script lang="ts">
  import { useFeatureFlags } from '$lib/stores'
  import { modals } from 'svelte-modals'
  import ConfirmDialog from '$lib/components/ConfirmDialog.svelte'
  import { SystemCommandData, SystemCommand } from '$lib/platform_shared/message'
  import { dataBroker } from '$lib/transport/databroker'
  import { Cancel, Power } from '../icons'

  const features = useFeatureFlags()

  const postSleep = () => dataBroker.emit(SystemCommandData, { command: SystemCommand.SYS_SLEEP })

  const confirmSleep = () => {
    modals.open(ConfirmDialog, {
      title: 'Confirm Power Down',
      message: 'Are you sure you want to switch off the device?',
      labels: {
        cancel: { label: 'Abort', icon: Cancel },
        confirm: { label: 'Switch Off', icon: Power }
      },
      onConfirm: () => {
        modals.close()
        postSleep()
      }
    })
  }
</script>

{#if $features.sleep}
  <div class="flex-none">
    <button class="btn btn-square btn-ghost h-9 w-10" onclick={confirmSleep}>
      <Power class="text-error h-9 w-9" />
    </button>
  </div>
{/if}
