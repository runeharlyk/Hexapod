<script lang="ts">
  import { modals } from 'svelte-modals'
  import ConfirmDialog from '$lib/components/ConfirmDialog.svelte'
  import SettingsCard from '$lib/components/SettingsCard.svelte'

  import { resolveUrl } from '$lib/proto-api'
  import { notifications } from '$lib/components/toasts/notifications'
  import { Cancel, OTA, Warning } from '$lib/components/icons'
  import { IMAGE_HEAD_BYTES, inspectImage, type AppImage } from '$lib/firmware/image'
  import { rememberUpdate } from '$lib/firmware/update-outcome'
  import { sendFirmware } from '$lib/firmware/ota'
  import { dataBroker } from '$lib/transport/databroker'
  import { useFeatureFlags } from '$lib/stores'

  const features = useFeatureFlags()

  let files: FileList | undefined = $state()
  let input: HTMLInputElement | undefined = $state()

  let sending = $state<{ sent: number; total: number } | null>(null)

  // The robot-hosted http page posts to the update endpoint; any other page (https on Bluetooth or
  // USB) cannot reach it and sends the image over the protobuf link instead.
  const overHttp = () => window.location.protocol === 'http:'

  const postImage = async (file: File) => {
    // Sent as the raw request body: /api/firmware streams it into the inactive OTA slot, so there
    // is no multipart envelope for the firmware to unwrap.
    const res = await fetch(resolveUrl('/api/firmware'), {
      method: 'POST',
      headers: { 'Content-Type': 'application/octet-stream' },
      body: file
    }).catch(() => null)
    if (!res?.ok) throw new Error((res && (await res.text().catch(() => ''))) || 'no answer')
  }

  const sendOverLink = async (file: File) => {
    sending = { sent: 0, total: file.size }
    try {
      await sendFirmware(
        new Uint8Array(await file.arrayBuffer()),
        payload => dataBroker.request(payload),
        (sent, total) => (sending = { sent, total })
      )
    } finally {
      sending = null
    }
  }

  async function uploadBIN(file: File, image: AppImage) {
    const previousSha256 = $features.firmware_elf_sha256
    try {
      await (overHttp() ? postImage(file) : sendOverLink(file))
    } catch (error) {
      notifications.error(
        `Firmware upload failed: ${error instanceof Error ? error.message : error}`,
        6000
      )
      return
    }
    rememberUpdate({ expectedSha256: image.elfSha256, previousSha256, requestedAt: Date.now() })
    notifications.success('Firmware written - the robot is restarting.', 5000)
  }

  // The robot refuses a factory image too, but only after the upload; checking the head first
  // answers at once and names what was wrong.
  async function checkAndConfirm() {
    const file = files?.[0]
    if (!file) return
    const check = inspectImage(new Uint8Array(await file.slice(0, IMAGE_HEAD_BYTES).arrayBuffer()))
    if (!check.ok) {
      notifications.error(check.error, 6000)
      if (input) input.value = ''
      return
    }
    const { image } = check
    modals.open(ConfirmDialog, {
      title: 'Confirm Flashing the Device',
      message: `Replace the running firmware with ${image.projectName} ${image.version}?`,
      labels: {
        cancel: { label: 'Abort', icon: Cancel },
        confirm: { label: 'Upload', icon: OTA }
      },
      onConfirm: () => {
        modals.close()
        uploadBIN(file, image)
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
    bind:this={input}
    accept=".bin"
    onchange={checkAndConfirm}
    disabled={sending !== null}
  />
  {#if sending}
    <progress class="progress progress-primary mt-2 w-full" value={sending.sent} max={sending.total}
    ></progress>
    <span class="text-sm opacity-70">Sending the firmware over the link; keep this page open.</span>
  {/if}
</SettingsCard>
