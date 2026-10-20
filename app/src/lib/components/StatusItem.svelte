<script lang="ts">
  import type { Snippet } from 'svelte'
  import type { IconComponent } from './icons'

  type Variant = 'success' | 'error' | 'primary' | 'info' | 'warning'

  // One reading as a row of a daisyUI `list`: inside a Card the list is a base-100 box, inside a Section it is bare.
  interface Props {
    icon?: IconComponent
    title: string
    description?: string | number
    variant?: Variant
    children?: Snippet
  }

  const { icon: Icon, title, description = '', variant = 'primary', children }: Props = $props()

  const variants: Record<Variant, [string, string]> = {
    success: ['bg-success', 'text-success-content'],
    error: ['bg-error', 'text-error-content'],
    primary: ['bg-primary', 'text-primary-content'],
    info: ['bg-info', 'text-info-content'],
    warning: ['bg-warning', 'text-warning-content']
  }

  const [bgColor, textColor] = $derived(variants[variant])
</script>

<li class="list-row items-center py-2">
  {#if Icon}
    <div class="mask mask-hexagon {bgColor} w-10">
      <Icon class="{textColor} h-auto w-full scale-75" />
    </div>
  {/if}
  <div class="min-w-0">
    <div class="font-semibold">{title}</div>
    <div class="text-base-content/70 text-sm break-words">{description}</div>
  </div>
  {#if children}
    <div class="flex items-center gap-1">{@render children()}</div>
  {/if}
</li>
