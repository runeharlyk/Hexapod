"""animations/fixtures/expected.json must be what the reference implementation produces today."""
import json

import numpy as np

import gen_animation_fixtures as gen
from src.robot import animation as an
from src.robot.animation_files import load_json, to_proto


def test_every_fixture_animation_is_valid_and_covered():
    names = {p.stem for p in gen.FIXTURE_DIR.glob("fx_*.json")}
    assert names == {"fx_mixed_legs", "fx_overlay", "fx_params", "fx_single"}
    for p in gen.FIXTURE_DIR.glob("fx_*.json"):
        assert an.validate(load_json(p)) is None, p.name
    covered = {c[0] for c in gen.EVALUATE_CASES} | {c["animation"] for c in gen.PLAYER_CASES}
    assert covered == names


def test_expected_json_is_current():
    committed = json.loads(gen.EXPECTED.read_text())
    fresh = gen.generate()
    assert committed["tolerance"] == fresh["tolerance"]
    assert len(committed["evaluate"]) == len(fresh["evaluate"])
    for c, f in zip(committed["evaluate"], fresh["evaluate"]):
        assert c["animation"] == f["animation"] and c["params"] == f["params"]
        assert len(c["samples"]) == len(f["samples"])
        for sc, sf in zip(c["samples"], f["samples"]):
            assert sc["t"] == sf["t"] and sc["mask"] == sf["mask"]
            assert np.allclose(sc["angles"], sf["angles"], atol=fresh["tolerance"])
    assert len(committed["player"]) == len(fresh["player"])
    for c, f in zip(committed["player"], fresh["player"]):
        assert c["animation"] == f["animation"] and c["params"] == f["params"] and c["events"] == f["events"]
        assert c["dt"] == f["dt"] and c["live"] == f["live"]
        assert len(c["trace"]) == len(f["trace"])
        assert [s["state"] for s in c["trace"]] == [s["state"] for s in f["trace"]]
        for sc, sf in zip(c["trace"], f["trace"]):
            assert sc["mask"] == sf["mask"]
            assert np.allclose(sc["angles"], sf["angles"], atol=fresh["tolerance"])


def test_player_traces_visit_every_state():
    expected = json.loads(gen.EXPECTED.read_text())
    states = {s["state"] for case in expected["player"] for s in case["trace"]}
    assert states == {"ENTRY", "PLAYING", "HOLD", "EXIT", "IDLE"}


def _assert_same_dump(committed: str, fresh: str) -> None:
    # Values that round to zero print as 0.000000 or -0.000000 depending on the platform's libm, and a
    # value on a rounding boundary may land one unit either way, so numbers compare within one unit of
    # the sixth decimal; every other token, and the line structure, must match exactly.
    committed_lines, fresh_lines = committed.splitlines(), fresh.splitlines()
    assert len(committed_lines) == len(fresh_lines)
    for number, (a, b) in enumerate(zip(committed_lines, fresh_lines), 1):
        ta, tb = a.split(), b.split()
        assert len(ta) == len(tb), f"line {number}"
        for x, y in zip(ta, tb):
            if x == y:
                continue
            try:
                close = abs(float(x) - float(y)) <= 1.5e-6
            except ValueError:
                close = False
            assert close, f"line {number}: {x} != {y}"


def test_expected_txt_and_fixture_binaries_are_current():
    _assert_same_dump(gen.EXPECTED_TXT.read_text(), gen.text_dump(gen.generate()))
    for path in sorted(gen.FIXTURE_DIR.glob("fx_*.json")):
        assert path.with_suffix(".pb").read_bytes() == to_proto(load_json(path)).SerializeToString(), path.name


def test_expected_txt_covers_every_sample_and_step():
    expected = json.loads(gen.EXPECTED.read_text())
    lines = gen.EXPECTED_TXT.read_text().splitlines()
    assert sum(1 for l in lines if l.startswith("E ")) == sum(len(c["samples"]) for c in expected["evaluate"])
    assert sum(1 for l in lines if l.startswith("S ")) == sum(len(c["trace"]) for c in expected["player"])
    assert sum(1 for l in lines if l.startswith("P ")) == len(expected["player"])
