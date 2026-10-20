// The selected network is an index into the saved list, so it has to follow its network when the list changes.

/** No network selected: its network was deleted. The robot then starts from the first, as for any index past its list. */
export const NONE = -1

/** The selection once the network at `index` is deleted. */
export const afterDelete = (selected: number, index: number) =>
  index === selected ? NONE
  : index < selected ? selected - 1
  : selected

/** The selection once the network at `from` moves to `to`. */
export const afterMove = (selected: number, from: number, to: number) => {
  if (selected === from) return to
  if (from < selected && selected <= to) return selected - 1
  if (to <= selected && selected < from) return selected + 1
  return selected
}
