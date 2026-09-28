<script lang="ts">
  import { MdiEyeOffOutline, MdiEyeOutline } from '../icons'

  interface Props {
    show?: boolean
    value?: string
    id?: string
    placeholder?: string
  }

  let {
    show = $bindable(false),
    value = $bindable(''),
    id = '',
    placeholder = ''
  }: Props = $props()

  let type = $derived(show ? 'text' : 'password')

  const handleInput = (e: Event) => (value = (e.currentTarget as HTMLInputElement).value)

  const togglePassword = () => (show = !show)
</script>

<label class="input input-bordered flex items-center gap-2">
  <input {type} class="grow" {value} oninput={handleInput} {id} {placeholder} />
  <!-- svelte-ignore a11y_click_events_have_key_events -->
  <div onclick={togglePassword} role="button" tabindex="0">
    <MdiEyeOffOutline class="text-base-content/50 h-6 {show ? 'block' : 'hidden'}" />
    <MdiEyeOutline class="text-base-content/50 h-6 {show ? 'hidden' : 'block'}" />
  </div>
</label>
