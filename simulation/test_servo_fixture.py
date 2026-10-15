"""The app's servo model (app/src/lib/simulation/servo.ts) is checked against a fixture of HexapodSim's servo; this
fails when the fixture no longer matches HexapodSim, so a change to the servo model reaches the app."""
import json

import pytest

from scripts.make_servo_fixture import OUT, fixture


def test_the_app_servo_fixture_matches_hexapod_sim():
    written = json.loads(OUT.read_text())
    current = fixture()
    assert written["constants"] == current["constants"]
    assert len(written["samples"]) == len(current["samples"])
    for stored, now in zip(written["samples"], current["samples"]):
        assert stored == pytest.approx(now, abs=1e-12), "stale: uv run python scripts/make_servo_fixture.py"
