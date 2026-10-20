<script lang="ts">
  import { fly } from 'svelte/transition'
  import { exitBeforeEnter, type ModalProps } from 'svelte-modals'
  import { Cancel, Check } from '$lib/components/icons'
  import { PasswordInput } from '$lib/components/input'
  import Field from '$lib/components/layout/Field.svelte'
  import { IPV4_HINT, IPV4_PATTERN } from '$lib/components/layout/validators'
  import type { KnownNetworkItem } from '$lib/types/models'

  interface Props extends ModalProps {
    /** The network to edit; a new one has only its SSID, if any. */
    network: KnownNetworkItem
    isNew: boolean
    onConfirm: (network: KnownNetworkItem) => void
    /** Told whether the draft differs from the network as the dialog opened. */
    onEdit?: (changed: boolean) => void
  }

  const { isOpen, close, network, isNew, onConfirm, onEdit }: Props = $props()

  // A copy of the network as the dialog opened, so cancelling leaves the saved list as it was.
  // svelte-ignore state_referenced_locally
  const draft = $state({ ...network })
  // svelte-ignore state_referenced_locally
  const opened = JSON.stringify(network)

  $effect(() => onEdit?.(JSON.stringify(draft) !== opened))

  // The button that opened the dialog, which gets focus back when it closes.
  const opener = document.activeElement

  // A modal <dialog>: the browser moves focus to its first field, keeps Tab inside it, makes the page behind it inert,
  // and cancels it on Escape.
  const showModal = (dialog: HTMLDialogElement) => {
    dialog.showModal()
    return {
      destroy: () => {
        if (opener instanceof HTMLElement && opener.isConnected) opener.focus()
      }
    }
  }

  const cancel = (event: Event) => {
    event.preventDefault()
    close()
  }

  // A press that starts and ends outside the box, on the backdrop, closes it; a drag out of a field does not.
  let pressedOutside = false
  const closeFromBackdrop = (event: MouseEvent) => {
    if (pressedOutside && event.target === event.currentTarget) close()
  }

  const ADDRESSES = [
    { key: 'local_ip', label: 'Local IP' },
    { key: 'gateway_ip', label: 'Gateway IP' },
    { key: 'subnet_mask', label: 'Subnet mask' },
    { key: 'dns_ip_1', label: 'DNS 1' },
    { key: 'dns_ip_2', label: 'DNS 2' }
  ] as const

  // The browser has checked every field against its constraints before this runs.
  const submit = (event: SubmitEvent) => {
    event.preventDefault()
    onConfirm($state.snapshot(draft))
    close()
  }
</script>

{#if isOpen}
  <dialog
    aria-labelledby="network-dialog-title"
    class="rounded-box bg-base-100 text-base-content m-auto max-h-[calc(100dvh-2rem)] w-[calc(100%-2rem)] max-w-xl overflow-y-auto p-0 shadow-lg backdrop:bg-transparent"
    use:showModal
    oncancel={cancel}
    onclose={() => close()}
    onpointerdown={event => (pressedOutside = event.target === event.currentTarget)}
    onclick={closeFromBackdrop}
    transition:fly|global={{ y: 50 }}
    use:exitBeforeEnter
  >
    <form class="flex flex-col gap-4 p-4 sm:p-6" onsubmit={submit}>
      <h2 id="network-dialog-title" class="text-lg font-semibold">
        {isNew ? 'Add network' : `Edit ${network.ssid}`}
      </h2>
      <fieldset class="fieldset gap-4 p-0 text-sm">
        <Field label="SSID" for="network-ssid" hint="3 to 32 characters">
          <input
            id="network-ssid"
            class="input validator w-full"
            type="text"
            required
            pattern={'.{3,32}'}
            maxlength="32"
            bind:value={draft.ssid}
          />
        </Field>
        <Field label="Password" for="network-password">
          <PasswordInput
            id="network-password"
            placeholder={isNew ? '' : 'Unchanged'}
            bind:value={draft.password}
          />
        </Field>
        <Field label="Static IP" for="network-static">
          <input
            id="network-static"
            type="checkbox"
            class="toggle"
            bind:checked={draft.static_ip_config}
          />
        </Field>
        {#if draft.static_ip_config}
          {#each ADDRESSES as address (address.key)}
            <Field label={address.label} for="network-{address.key}" hint={IPV4_HINT}>
              <input
                id="network-{address.key}"
                class="input validator w-full"
                type="text"
                required
                pattern={IPV4_PATTERN}
                bind:value={draft[address.key]}
              />
            </Field>
          {/each}
        {/if}
      </fieldset>
      <div class="flex flex-wrap justify-end gap-2">
        <button type="button" class="btn btn-ghost" onclick={() => close()}>
          <Cancel class="size-5" />Cancel
        </button>
        <button type="submit" class="btn btn-primary">
          <Check class="size-5" />{isNew ? 'Add network' : 'Update network'}
        </button>
      </div>
    </form>
  </dialog>
{/if}
