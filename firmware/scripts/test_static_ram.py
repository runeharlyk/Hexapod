"""The static internal-RAM gate the firmware build runs (static_ram.py).

    uv run --no-project --with pytest pytest firmware/scripts
"""
from static_ram import over_budget, usage

# Section sizes of a real esp32-wroom-camera build: flash and the dummy placeholders do not count.
SECTIONS = {".iram0.vectors": 1027, ".iram0.text": 91959, ".iram0.text_end": 197, ".iram0.data": 0,
            ".iram0.bss": 0, ".dram0.dummy": 76800, ".dram0.data": 33124, ".dram0.bss": 84240, ".noinit": 0,
            ".flash.text": 1334516, ".flash.rodata": 538604, ".ext_ram.dummy": 1966048}


def test_usage_counts_the_static_data_and_the_iram_sections_only():
    assert usage(SECTIONS) == {"dram_bytes": 117364, "iram_bytes": 93183}


def test_a_20_kb_static_buffer_fails_the_gate():
    budget = {"dram_bytes": 120000, "iram_bytes": 95000}
    assert over_budget(usage(SECTIONS), budget) == []
    grown = {**SECTIONS, ".dram0.bss": SECTIONS[".dram0.bss"] + 20 * 1024}
    assert over_budget(usage(grown), budget) == ["dram_bytes: 137844 bytes, over the budget of 120000 by 17844"]
