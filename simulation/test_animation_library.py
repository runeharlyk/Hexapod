"""Every bundled animation loads, validates, and evaluates without a clamped joint at its keyframes."""
from pathlib import Path

import pytest

from src.robot import animation as an
from src.robot.animation_files import load_json
from src.robot.firmware_gait import Kinematics

LIBRARY = Path(__file__).resolve().parents[1] / "animations"
FILES = sorted(LIBRARY.glob("*.json"))
KIN = Kinematics()


def test_the_library_has_the_seven_bundled_animations():
    assert {p.stem for p in FILES} == {"wave", "crouch", "wiggle", "stretch", "spooked", "play_dead", "body_roll_test"}


@pytest.mark.parametrize("path", FILES, ids=[p.stem for p in FILES])
def test_bundled_animation_is_valid_and_named_after_its_file(path):
    a = load_json(path)
    assert an.validate(a) is None
    assert a.name == path.stem


@pytest.mark.parametrize("path", FILES, ids=[p.stem for p in FILES])
def test_bundled_animation_keyframes_stay_inside_joint_travel(path):
    a = load_json(path)
    params = an.resolve_params(a, None)
    for k in a.keyframes:
        _, mask = an.pose_to_angles(an.evaluate(a, params, k.time, KIN), KIN)
        assert mask == 0, f"{path.stem} clamps joints {mask:018b} at t={k.time}"


def sample_times(a):
    times = [k.time for k in a.keyframes]
    return times + [(t0 + t1) / 2 for t0, t1 in zip(times, times[1:])]


def param_corners(a):
    """Each declared multiplier at its min and at its max with the rest at default."""
    for spec in a.params:
        if spec.id in (an.ParamId.SPEED, an.ParamId.REPEAT):
            continue
        for value in (spec.min, spec.max):
            yield f"{an.ParamId(spec.id).name}={value}", {spec.id: value}


@pytest.mark.parametrize("path", FILES, ids=[p.stem for p in FILES])
def test_bundled_animation_stays_inside_joint_travel_at_its_parameter_extremes(path):
    a = load_json(path)
    for label, values in param_corners(a):
        params = an.resolve_params(a, values)
        for t in sample_times(a):
            _, mask = an.pose_to_angles(an.evaluate(a, params, t, KIN), KIN)
            assert mask == 0, f"{path.stem} clamps joints {mask:018b} at t={t} with {label}"


from src.robot.animation_files import load_binary


def test_pack_script_writes_a_decodable_pb_per_animation(tmp_path):
    script = LIBRARY.parent / "firmware" / "scripts" / "pack_animations.py"
    source = script.read_text().replace("\nmain()\n", "\n")  # do not pack into firmware/data from a test
    namespace = {"__file__": str(script), "__name__": "pack_animations"}
    exec(compile(source, str(script), "exec"), namespace)
    written = namespace["pack"](LIBRARY, tmp_path)
    assert {p.stem for p in written} == {p.stem for p in FILES}
    for p in written:
        assert an.validate(load_binary(p)) is None