/** A daisyUI 5 theme colour by name (`primary`, `base-content`, ...), at `opacity` percent, for a canvas. */
export const daisyColor = (name: string, opacity: number = 100) => {
  const color = getComputedStyle(document.documentElement)
    .getPropertyValue(`--color-${name}`)
    .trim()
  // A theme may give a colour as hex or as oklch(); color-mix fades either.
  return opacity === 100 ? color : `color-mix(in oklab, ${color} ${opacity}%, transparent)`
}
