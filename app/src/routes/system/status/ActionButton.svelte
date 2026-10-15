<script>
  // eslint-plugin-svelte 2 lints every component as a custom element; this is not one.
  // svelte-ignore custom_element_props_identifier
  const { icon, type, label, ...rest } = $props()

  const Icon = $derived(icon)
</script>

<button class="btn btn-{type} inline-flex items-center" {...rest}>
  <Icon class="mr-2 h-5 w-5" />
  <span>{label}</span>
</button>
