import { afterEach, describe, expect, it } from 'vitest'
import { daisyColor } from '../../src/lib/utilities/color-utilities'

// daisyUI 5 keeps each theme colour whole, as `--color-<name>`; daisyUI 4's `--p` holds no value any more.
// The app's own theme gives some colours as hex (primary #00bfff) and the stock themes as oklch().
describe('daisyColor', () => {
  const root = document.documentElement.style

  afterEach(() => root.removeProperty('--color-primary'))

  it("reads the theme colour from daisyUI 5's variable", () => {
    root.setProperty('--color-primary', 'oklch(45% .24 277.023)')
    expect(daisyColor('primary')).toBe('oklch(45% .24 277.023)')
  })

  it.each(['oklch(45% .24 277.023)', '#00bfff'])('fades %s to the opacity', color => {
    root.setProperty('--color-primary', color)
    expect(daisyColor('primary', 50)).toBe(`color-mix(in oklab, ${color} 50%, transparent)`)
  })
})
