<script lang="ts">
  import { modals } from 'svelte-modals'
  import ConfirmDialog from '$lib/components/ConfirmDialog.svelte'
  import SettingsCard from '$lib/components/SettingsCard.svelte'

  import { resolveUrl } from '$lib/proto-api'
  import { notifications } from '$lib/components/toasts/notifications'
  import { Cancel, OTA, Warning } from '$lib/components/icons'

  let files: FileList | undefined = $state()

  async function uploadBIN() {
    const file = files?.[0]
    if (!file) return
    // Sent as the raw request body: /api/firmware streams it into the inactive OTA slot, so there
    // is no multipart envelope for the firmware to unwrap.
    const res = await fetch(resolveUrl('/api/firmware'), {
      method: 'POST',
      headers: { 'Content-Type': 'application/octet-stream' },
      body: file
    }).catch(() => null)
    if (!res?.ok) {
      notifications.error('Firmware upload failed.', 5000)
      return
    }
    notifications.success('Firmware written - the robot is restarting.', 5000)
  }

  function confirmBinUpload() {
    modals.open(ConfirmDialog, {
      title: 'Confirm Flashing the Device',
      message: 'Are you sure you want to overwrite the existing firmware with a new one?',
      labels: {
        cancel: { label: 'Abort', icon: Cancel },
        confirm: { label: 'Upload', icon: OTA }
      },
      onConfirm: () => {
        modals.close()
        uploadBIN()
      }
    })
  }
</script>

<SettingsCard collapsible={false}>
  {#snippet icon()}
    <OTA class="lex-shrink-0 mr-2 h-6 w-6 self-end rounded-full" />
  {/snippet}
  {#snippet title()}
    <span>Upload Firmware</span>
  {/snippet}
  <div class="alert alert-warning shadow-lg">
    <Warning class="h-6 w-6 shrink-0" />
    <span
      >Uploading a new firmware (.bin) file will replace the existing firmware. The bootloader
      verifies the image, and a firmware that fails to boot rolls back to the running one.
    </span>
  </div>

  <input
    type="file"
    id="binFile"
    class="file-input file-input-bordered file-input-secondary mt-4 w-full"
    bind:files
    accept=".bin"
    onchange={confirmBinUpload}
  />
</SettingsCard>
