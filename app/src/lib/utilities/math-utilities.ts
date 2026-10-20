export const toUint8 = (number: number, min: number, max: number) => {
  number = Math.max(min, Math.min(max, number))
  const scaled = ((number - min) / (max - min)) * 255
  return Math.round(scaled) & 0xff
}

export const toInt8 = (number: number, min: number, max: number) => {
  number = Math.max(min, Math.min(max, number))
  const scaled = ((number - min) / (max - min)) * 255 - 128
  return Math.max(-128, Math.min(127, Math.round(scaled))) | 0
}

export const fromInt8 = (int8: number, min: number, max: number) => {
  int8 = Math.max(-128, Math.min(127, int8))
  const scaled = (int8 + 128) / 255
  const number = scaled * (max - min) + min
  return number
}

/** The shortest decimal that is the same float32 as `value`, as a float32 field's reply reads best. */
export const shortestFloat32 = (value: number) => {
  const single = Math.fround(value)
  for (let digits = 1; digits < 9; digits++) {
    const shorter = Number(single.toPrecision(digits))
    if (Math.fround(shorter) === single) return shorter
  }
  return single
}
