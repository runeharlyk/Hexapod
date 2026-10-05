const DIRECTIONS = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW']

// The eight-point direction of a heading in degrees, any turn count or sign.
export const cardinalDirection = (heading: number) => {
  const normalized = ((heading % 360) + 360) % 360
  return DIRECTIONS[Math.round(normalized / 45) % 8]
}
