/**
 * Groups of top-level keys that form one field: a list and the index into it, say, which mean nothing apart. Every
 * other leaf of a settings group is a field of its own, such as one servo's centre.
 */
export type Together = readonly (readonly string[])[]

/** A deep copy, as the robot's values travel: through JSON, but for undefined, which stays undefined. */
export const clone = <T>(value: T): T =>
  value === undefined ? value : JSON.parse(JSON.stringify(value))

const isBranch = (value: unknown): value is Record<string, unknown> =>
  typeof value === 'object' && value !== null && Object.keys(value).length > 0

const leafPaths = (value: unknown, prefix = ''): string[] =>
  isBranch(value) ?
    Object.entries(value).flatMap(([key, child]) =>
      leafPaths(child, prefix ? `${prefix}.${key}` : key)
    )
  : prefix ? [prefix]
  : []

const getAt = (value: unknown, path: string): unknown =>
  path
    .split('.')
    .reduce<unknown>(
      (at, key) => (isBranch(at) ? (at as Record<string, unknown>)[key] : undefined),
      value
    )

const setAt = (target: object, path: string, value: unknown) => {
  const keys = path.split('.')
  const last = keys.pop() as string
  const parent = keys.reduce<Record<string, unknown>>(
    (at, key, index) => {
      at[key] ??= /^\d+$/.test(keys[index + 1] ?? last) ? [] : {}
      return at[key] as Record<string, unknown>
    },
    target as Record<string, unknown>
  )
  if (value === undefined) delete parent[last]
  else parent[last] = value
}

/** The field a dotted leaf path (`servos.3.centerPwm`) belongs to: its group's first key, else the path itself. */
export const fieldOf = (path: string, together: Together): string => {
  const [first] = path.split('.')
  return together.find(group => group.includes(first))?.[0] ?? path
}

/** What a field holds, as the page shows it: the value at the path, or the group's first key (a list). */
export const fieldValue = (value: unknown, field: string): unknown => getAt(value, field)

const pathsOf = (field: string, together: Together): readonly string[] =>
  together.find(group => group[0] === field) ?? [field]

const signature = (value: unknown, field: string, together: Together): string =>
  JSON.stringify(pathsOf(field, together).map(path => getAt(value, path)))

export const sameField = (a: unknown, b: unknown, field: string, together: Together): boolean =>
  signature(a, field, together) === signature(b, field, together)

/** Puts `from`'s value of a field into `into`, the whole group for a grouped field. */
export const copyField = (into: object, from: unknown, field: string, together: Together) => {
  for (const path of pathsOf(field, together)) setAt(into, path, clone(getAt(from, path)))
}

const fieldsOf = (together: Together, ...values: unknown[]): string[] => [
  ...new Set([
    ...together.map(group => group[0]),
    ...values.flatMap(value => leafPaths(value).map(path => fieldOf(path, together)))
  ])
]

/**
 * A push, `next`, merged into a draft edited against `base`, field by field. A field the user has not changed (its draft
 * equals `base`) takes `next`, unless it is the one focused, which keeps its text. A changed field keeps the user's
 * value, and when the robot changed it too, to something other than the user's value, `notes` holds the robot's value.
 * Lists outside `together` are matched by position and must keep their length.
 */
export const mergePush = <T extends object>(
  draft: T,
  base: T,
  next: T,
  focused: string | null,
  together: Together
): { draft: T; notes: Record<string, unknown> } => {
  const merged = clone(draft)
  const notes: Record<string, unknown> = {}
  const focusedField = focused === null ? null : fieldOf(focused, together)
  for (const field of fieldsOf(together, draft, base, next)) {
    if (sameField(draft, base, field, together)) {
      if (field !== focusedField) copyField(merged, next, field, together)
    } else if (
      !sameField(draft, next, field, together) &&
      !sameField(base, next, field, together)
    ) {
      notes[field] = clone(fieldValue(next, field))
    }
  }
  return { draft: merged, notes }
}
