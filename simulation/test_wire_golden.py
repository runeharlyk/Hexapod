"""platform_shared/wire_golden.json must still describe platform_shared/*.proto (see gen_wire_golden.py)."""
import json

import gen_wire_golden as gen


def _differences(committed: dict, current: dict) -> list[str]:
    lines = []
    for kind in ("enums", "messages"):
        old, new = committed[kind], current[kind]
        for name in sorted(old.keys() | new.keys()):
            if name not in new:
                lines.append(f"{kind[:-1]} {name} removed")
            elif name not in old:
                lines.append(f"{kind[:-1]} {name} added")
            else:
                for entry in sorted(old[name].keys() | new[name].keys()):
                    before, after = old[name].get(entry), new[name].get(entry)
                    if before != after:
                        lines.append(f"{name}.{entry}: {before} -> {after}")
    return lines


def test_the_protos_still_have_the_committed_wire_numbers():
    committed = json.loads(gen.GOLDEN.read_text())
    differences = _differences(committed, gen.wire_numbers())
    assert not differences, (
        "the wire format changed:\n  " + "\n  ".join(differences) + "\n"
        "If this is intended, the app and the firmware must ship together; run `uv run python gen_wire_golden.py`.")

