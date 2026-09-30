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


def test_expected_txt_and_fixture_binaries_are_current():
    assert gen.EXPECTED_TXT.read_bytes() == gen.text_dump(gen.generate()).encode()
    for path in sorted(gen.FIXTURE_DIR.glob("fx_*.json")):
        assert path.with_suffix(".pb").read_bytes() == to_proto(load_json(path)).SerializeToString(), path.name


def test_expected_txt_covers_every_sample_and_step():
    expected = json.loads(gen.EXPECTED.read_text())
    lines = gen.EXPECTED_TXT.read_text().splitlines()
    assert sum(1 for l in lines if l.startswith("E ")) == sum(len(c["samples"]) for c in expected["evaluate"])
    assert sum(1 for l in lines if l.startswith("S ")) == sum(len(c["trace"]) for c in expected["player"])
    assert sum(1 for l in lines if l.startswith("P ")) == len(expected["player"])
