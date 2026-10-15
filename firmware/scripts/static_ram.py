"""Fails when the firmware's static internal RAM outgrows the budget in firmware/static_ram_budget.json.

Internal RAM is the robot's binding limit (docs/firmware.md), and what is left of it at runtime cannot be measured in
CI. What the linker places in it can: DRAM's .dram0.data and .dram0.bss, and IRAM's .iram0 sections, read from the
ELF. A change that needs more raises the budget in the same commit and says why there.

    python firmware/scripts/static_ram.py .pio/build/esp32-wroom-camera/firmware.elf
"""
import json
import sys
from pathlib import Path

BUDGET = Path(__file__).resolve().parents[1] / "static_ram_budget.json"
DRAM_SECTIONS = (".dram0.data", ".dram0.bss")
IRAM_PREFIX = ".iram0."


def usage(sections: dict[str, int]) -> dict[str, int]:
    """Static DRAM and IRAM in bytes, from section name -> size."""
    return {"dram_bytes": sum(sections.get(name, 0) for name in DRAM_SECTIONS),
            "iram_bytes": sum(size for name, size in sections.items() if name.startswith(IRAM_PREFIX))}


def over_budget(used: dict[str, int], budget: dict[str, int]) -> list[str]:
    return [f"{key}: {used[key]} bytes, over the budget of {budget[key]} by {used[key] - budget[key]}"
            for key in ("dram_bytes", "iram_bytes") if used[key] > budget[key]]


def elf_sections(path: Path) -> dict[str, int]:
    from elftools.elf.elffile import ELFFile

    with path.open("rb") as f:
        return {s.name: s["sh_size"] for s in ELFFile(f).iter_sections()}


def main() -> int:
    used = usage(elf_sections(Path(sys.argv[1])))
    budget = json.loads(BUDGET.read_text())
    for key in ("dram_bytes", "iram_bytes"):
        print(f"{key}: {used[key]} of {budget[key]} ({budget[key] - used[key]} left)")
    failures = over_budget(used, budget)
    for failure in failures:
        print(f"error: {failure}; raise {BUDGET.name} only in a commit that justifies the growth", file=sys.stderr)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
