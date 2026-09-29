# Animation Foundation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Define the animation file schema and build the Python reference evaluator, player, golden fixtures, bundled animation library, headless feasibility checker and a sim sandbox Animate mode.

**Architecture:** `platform_shared/animation.proto` is the single schema; bundled animations are its proto3 JSON mapping under `animations/`.
`simulation/src/robot/animation.py` is the pure reference implementation (no protobuf import) that the later firmware and app ports mirror line for line; `animation_files.py` converts between the generated protobuf classes and the dataclasses.
`animations/fixtures/expected.json` is generated from the reference and is what the firmware and app parity tests will consume.

**Tech Stack:** protobuf 3 (grpcio-tools for codegen, `google.protobuf.json_format`), Python 3.10+ with numpy under `uv`, MuJoCo via the existing `HexapodSim`, pytest, Tkinter sandbox.

**Spec:** `docs/superpowers/specs/2026-09-29-animation-system-design.md`

This is plan 1 of 3.
Plan 2 (firmware) and plan 3 (app) are written after this plan lands because their parity tests consume the fixtures produced here.

## Global Constraints

- All code, comments, docs and commit messages in English and ASCII: write the letter x rather than a multiplication sign and a hyphen rather than an en dash.
- Commit messages are one line, `[Gitmoji] [Verb] ...`, no body and no trailers of any kind.
- Python work runs through `uv` from `simulation/` (`uv run ...`); never plain `pip`.
- Markdown: each sentence on its own line.
- Comments only for genuine documentation or a non-obvious why; prefer self-documenting names.
- No dead code: everything added is used or deleted.
- The schema limits from the spec: `name` max 32 chars of `[a-z0-9_-]`, `description` max 96, `keyframes` max 32, `legs` 0 or exactly 6, `overlays` max 8, `params` max 10, `schema` is 1.
- Units: body angles radians, lengths millimetres, joint angles degrees in the IK output convention, time seconds.
- Joint clamp is symmetric `+-JOINT_LIMIT_DEG = {31.5, 90.0, 149.0}` for coxa, femur, tibia, in IK-output angle space (the tibia +90 servo offset is applied after the clamp in `servo_controller.h`, so it never enters this code).
- Forward is body +Y (front legs sit at `y = +152`); positive body `z` offset crouches (firmware `zm` convention).
- Default entry and exit blend is 0.5 s when the file says 0.
- Step arc: a foot whose horizontal travel during Entry or Exit exceeds 2 mm lifts by `45 mm * min(1, travel / 40 mm) * sin(pi * u)`.
- Speed multiplies only the Playing clock; Entry and Exit run on wall time.
- An overlay on a leg that is currently a joint-angle target is ignored.

## Review Focus

1. A keyframe whose `legs` has 3 entries: loading must fail with a clear message, never index out of range.
   Test in Task 2.
2. `evaluate` called with `t` before 0 or after the last keyframe: clamps to the ends, no exception, overlays outside their window inactive.
   Test in Task 3.
3. Foot-to-joint interpolation for a leg where the foot endpoint is unreachable: IK's acos clamps silently, the clamp mask must still flag the joint at the endpoints.
   Test in Task 3.
4. Player `update` with `dt = 0` or a single-keyframe animation (duration 0): must not divide by zero and must reach Hold or Exit.
   Test in Task 4.
5. `play` called while in Exit: the new Entry starts from the current blended pose, not from stance, and no foot jumps.
   Test in Task 4.

---

## File Structure

| Path | Responsibility |
| --- | --- |
| `platform_shared/animation.proto` | the schema |
| `platform_shared/animation.options` | nanopb size limits (must be valid now because the firmware build globs every proto) |
| `simulation/scripts/compile_protos.py` | generates `simulation/src/platform_shared/animation_pb2.py` (gitignored) |
| `simulation/src/robot/animation.py` | dataclasses, validation, evaluator, pose to angles, player; pure numpy |
| `simulation/src/robot/animation_files.py` | protobuf and JSON conversion, load and save |
| `simulation/test_animation.py` | unit tests for the reference implementation |
| `simulation/test_animation_fixtures.py` | fixtures are current and complete |
| `simulation/gen_animation_fixtures.py` | writes `animations/fixtures/expected.json` |
| `animations/*.json` | bundled library |
| `animations/fixtures/*.json`, `expected.json` | parity fixtures |
| `firmware/scripts/pack_animations.py` | JSON to `.pb` into `firmware/data/animations/` (gitignored) on every build |
| `simulation/check_animation.py` | headless MuJoCo feasibility checker |
| `simulation/sim_sandbox.py` | gains an Animate mode |
| `simulation/README.md`, `CLAUDE.md` | command reference |

---

### Task 1: Schema, nanopb options and Python codegen

**Files:**
- Create: `platform_shared/animation.proto`
- Create: `platform_shared/animation.options`
- Create: `simulation/scripts/compile_protos.py`
- Modify: `simulation/pyproject.toml` (dev group)
- Modify: `simulation/.gitignore`
- Test: `simulation/test_animation.py`

**Interfaces:**
- Produces: importable module `src.platform_shared.animation_pb2` with `Animation`, `Keyframe`, `LegTarget`, `FootOffset`, `JointAngles`, `BodyPose`, `Overlay`, `ParamSpec`, enums `Ease`, `ParamId`.

- [ ] **Step 1: Write the schema**

`platform_shared/animation.proto`:

```proto
syntax = "proto3";

package animation;

// One animation file. Offsets are relative to the standing posture so an animation composes
// with ride height and the feet-distance slider. Angles rad, lengths mm, time seconds.

message BodyPose {
  float roll = 1;
  float pitch = 2;
  float yaw = 3;
  float x = 4;
  float y = 5;
  float z = 6;  // positive crouches (firmware zm convention)
}

message FootOffset {
  float x = 1;
  float y = 2;
  float z = 3;  // positive lifts
}

// Degrees in the IK output convention (coxa yaw, absolute femur, tibia relative to femur).
message JointAngles {
  float coxa = 1;
  float femur = 2;
  float tibia = 3;
}

message LegTarget {
  oneof target {
    FootOffset foot = 1;
    JointAngles joints = 2;
  }
}

enum Ease {
  LINEAR = 0;
  EASE_IN = 1;
  EASE_OUT = 2;
  EASE_IN_OUT = 3;
}

message Keyframe {
  float time = 1;               // strictly increasing, first keyframe at 0
  Ease ease = 2;                // shapes the segment that ends at this keyframe
  BodyPose body = 3;            // absent = zero offset
  repeated LegTarget legs = 4;  // 0 entries = every foot holds stance, else exactly 6
}

// Additive sine on one channel. body_axis 0..5 in BodyPose field order, foot_channel = leg * 3 + axis.
message Overlay {
  oneof channel {
    uint32 body_axis = 1;
    uint32 foot_channel = 2;
  }
  float amplitude = 3;
  float frequency = 4;  // Hz of animation time
  float phase = 5;      // rad
  float start = 6;      // active window in animation time, start < end
  float end = 7;
}

enum ParamId {
  SPEED = 0;              // multiplies the animation clock
  BODY_X = 1;             // BODY_* multiply that body channel
  BODY_Y = 2;
  BODY_Z = 3;
  BODY_ROLL = 4;
  BODY_PITCH = 5;
  BODY_YAW = 6;
  FOOT_LIFT = 7;          // multiplies every foot offset z
  OVERLAY_AMPLITUDE = 8;  // multiplies every overlay amplitude
  REPEAT = 9;             // plays of a non-looping animation, rounded to an integer
}

// default_value rather than default: default is a C keyword and nanopb emits field names verbatim.
message ParamSpec {
  ParamId id = 1;
  float min = 2;
  float default_value = 3;
  float max = 4;
}

message Animation {
  string name = 1;         // [a-z0-9_-], unique on the robot, equals the file stem
  string description = 2;
  uint32 schema = 3;       // 1
  bool loop = 4;           // repeat until stopped
  bool hold_end = 5;       // freeze on the last keyframe until stopped, else exit to stance
  float entry_time = 6;    // seconds; 0 means the default 0.5
  float exit_time = 7;     // seconds; 0 means the default 0.5
  repeated Keyframe keyframes = 8;
  repeated Overlay overlays = 9;
  repeated ParamSpec params = 10;  // only the ids listed are exposed
}
```

`platform_shared/animation.options`:

```
animation.Animation.name max_size:33
animation.Animation.description max_size:97
animation.Animation.keyframes max_count:32
animation.Animation.overlays max_count:8
animation.Animation.params max_count:10
animation.Keyframe.legs max_count:6
```

(`max_size` counts the terminating NUL, hence 33 and 97.)

- [ ] **Step 2: Prove the firmware generator accepts the schema**

The firmware pre-build compiles every `platform_shared/*.proto`, so a bad options file would break `pio run`.

Run from the repo root in PowerShell: `python firmware/scripts/compile_protos.py`
Expected: `Compiled 3 proto file(s)` and `firmware/src/platform_shared/animation.pb.h` exists containing `animation_ParamSpec` with a member named `default_value`.

- [ ] **Step 3: Add the Python codegen script**

`simulation/scripts/compile_protos.py`:

```python
"""Generate Python protobuf modules for the simulation from platform_shared/*.proto.

Only the schemas without imports are generated: protoc emits absolute `import x_pb2` lines for
imported files, which break inside the src.platform_shared package.
"""
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PROTO_DIR = ROOT / "platform_shared"
OUT_DIR = ROOT / "simulation" / "src" / "platform_shared"
PROTO_FILES = ["animation.proto"]


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    (OUT_DIR / "__init__.py").touch()
    cmd = [sys.executable, "-m", "grpc_tools.protoc", f"-I{PROTO_DIR}", f"--python_out={OUT_DIR}"]
    cmd += [str(PROTO_DIR / f) for f in PROTO_FILES]
    subprocess.run(cmd, check=True)
    print(f"protoc: {', '.join(PROTO_FILES)} -> {OUT_DIR}")


if __name__ == "__main__":
    main()
```

Add to `simulation/.gitignore`:

```
src/platform_shared/
```

Add `grpcio-tools` to the dev group in `simulation/pyproject.toml`:

```toml
[dependency-groups]
dev = ["pytest", "grpcio-tools"]
```

Run from `simulation/`: `uv sync`
Expected: resolves without error.
The lock already has `protobuf 7.35.1`; if uv reports a conflict between `grpcio-tools` and that protobuf, pin `grpcio-tools>=1.75` and re-run.
Do not downgrade protobuf, tensorboard depends on it.

Run: `uv run python scripts/compile_protos.py`
Expected: `simulation/src/platform_shared/animation_pb2.py` exists.

- [ ] **Step 4: Write the round-trip test**

`simulation/test_animation.py` (new file, more tests are appended in later tasks):

```python
"""Unit tests for the animation reference implementation (src/robot/animation.py)."""
import json

import numpy as np
import pytest
from google.protobuf import json_format

from src.platform_shared import animation_pb2 as pb


def test_proto_json_round_trip_keeps_the_leg_oneof():
    src = {
        "name": "rt",
        "schema": 1,
        "keyframes": [
            {"time": 0},
            {"time": 1.5, "ease": "EASE_IN_OUT", "body": {"roll": 0.1, "z": 20},
             "legs": [{"foot": {"z": 30}}, {"joints": {"femur": 80, "tibia": -110}},
                      {"foot": {}}, {"foot": {}}, {"foot": {}}, {"foot": {}}]},
        ],
        "params": [{"id": "FOOT_LIFT", "min": 0.5, "defaultValue": 1, "max": 1.5}],
    }
    msg = json_format.ParseDict(src, pb.Animation())
    assert msg.keyframes[1].legs[0].WhichOneof("target") == "foot"
    assert msg.keyframes[1].legs[1].WhichOneof("target") == "joints"
    assert msg.keyframes[1].legs[2].WhichOneof("target") == "foot"
    assert msg.params[0].default_value == 1
    back = json.loads(json_format.MessageToJson(msg))
    assert back["keyframes"][1]["legs"][1] == {"joints": {"femur": 80, "tibia": -110}}
    assert pb.Animation.FromString(msg.SerializeToString()) == msg
```

- [ ] **Step 5: Run the test**

Run from `simulation/`: `uv run pytest test_animation.py -q`
Expected: 1 passed.

- [ ] **Step 6: Commit**

```bash
git add platform_shared/animation.proto platform_shared/animation.options simulation/scripts/compile_protos.py simulation/pyproject.toml simulation/uv.lock simulation/.gitignore simulation/test_animation.py
git commit -m "✨ Adds the animation file schema and its Python codegen"
```

---

### Task 2: Data model, validation and file conversion

**Files:**
- Create: `simulation/src/robot/animation.py` (first part)
- Create: `simulation/src/robot/animation_files.py`
- Test: `simulation/test_animation.py`

**Interfaces:**
- Produces in `animation.py`: `Ease`, `ParamId`, `BodyAxis` (IntEnums); dataclasses `LegTarget(foot, joints)`, `Keyframe(time, ease, body, legs)`, `Overlay(body_axis, foot_channel, amplitude, frequency, phase, start, end)`, `ParamSpec(id, min, default_value, max)`, `Animation(...)` with `duration`, `entry_seconds()`, `exit_seconds()`; `validate(anim) -> str | None`; `leg_target(keyframe, leg) -> LegTarget`.
- Produces in `animation_files.py`: `from_proto(msg) -> Animation`, `to_proto(anim) -> pb.Animation`, `load_json(path) -> Animation`, `save_json(anim, path)`, `load_binary(path)`, `save_binary(anim, path)`, `json_text(anim) -> str`.

- [ ] **Step 1: Write the failing tests**

Append to `simulation/test_animation.py`:

```python
import math

from src.robot import animation as an
from src.robot.animation_files import from_proto, to_proto, json_text, load_json
from src.robot.firmware_gait import BodyState, Kinematics

KIN = Kinematics()


def stance_legs():
    return [an.LegTarget.stance() for _ in range(6)]


def two_keyframes(**kw):
    return an.Animation(name="t", keyframes=[an.Keyframe(0.0), an.Keyframe(1.0)], **kw)


def test_validate_accepts_a_minimal_animation():
    assert an.validate(an.Animation(name="ok", keyframes=[an.Keyframe(0.0)])) is None


@pytest.mark.parametrize("mutate, message", [
    (lambda a: setattr(a, "schema", 2), "schema"),
    (lambda a: setattr(a, "name", "Bad Name"), "name"),
    (lambda a: setattr(a, "name", "x" * 33), "name"),
    (lambda a: setattr(a, "description", "d" * 97), "description"),
    (lambda a: setattr(a, "keyframes", []), "keyframe"),
    (lambda a: setattr(a.keyframes[0], "time", 0.1), "time 0"),
    (lambda a: setattr(a.keyframes[1], "time", 0.0), "increase"),
    (lambda a: setattr(a.keyframes[1], "legs", stance_legs()[:3]), "0 or 6"),
    (lambda a: a.overlays.append(an.Overlay(amplitude=1, frequency=1, start=0, end=1)), "channel"),
    (lambda a: a.overlays.append(an.Overlay(body_axis=6, amplitude=1, frequency=1, start=0, end=1)), "body_axis"),
    (lambda a: a.overlays.append(an.Overlay(foot_channel=18, amplitude=1, frequency=1, start=0, end=1)), "foot_channel"),
    (lambda a: a.overlays.append(an.Overlay(body_axis=0, amplitude=1, frequency=1, start=0.5, end=0.5)), "start"),
    (lambda a: a.overlays.append(an.Overlay(body_axis=0, amplitude=1, frequency=1, start=0, end=1.5)), "end"),
    (lambda a: a.params.extend([an.ParamSpec(an.ParamId.SPEED, 0.5, 1, 2)] * 2), "unique"),
    (lambda a: a.params.append(an.ParamSpec(an.ParamId.BODY_Z, 0.5, 3, 2)), "min <= default_value <= max"),
    (lambda a: a.params.append(an.ParamSpec(an.ParamId.SPEED, 0.0, 1, 2)), "SPEED"),
])
def test_validate_reports_each_structural_rule(mutate, message):
    a = two_keyframes()
    mutate(a)
    err = an.validate(a)
    assert err is not None and message in err


def test_validate_rejects_too_many_of_everything():
    a = an.Animation(name="t", keyframes=[an.Keyframe(float(i)) for i in range(33)])
    assert "32" in an.validate(a)
    a = two_keyframes(overlays=[an.Overlay(body_axis=0, amplitude=1, frequency=1, start=0, end=1)] * 9)
    assert "8" in an.validate(a)
    a = two_keyframes(params=[an.ParamSpec(an.ParamId(i), 0.5, 1, 2) for i in range(10)])
    assert an.validate(a) is None
    a.params.append(an.ParamSpec(an.ParamId.REPEAT, 1, 1, 1))
    assert "10" in an.validate(a)


def test_entry_and_exit_default_to_half_a_second():
    a = two_keyframes()
    assert a.entry_seconds() == 0.5 and a.exit_seconds() == 0.5
    a.entry_time, a.exit_time = 0.2, 0.9
    assert a.entry_seconds() == 0.2 and a.exit_seconds() == 0.9


def test_leg_target_of_an_empty_legs_array_is_stance():
    k = an.Keyframe(0.0, legs=[])
    t = an.leg_target(k, 4)
    assert not t.is_joints() and np.array_equal(t.foot, np.zeros(3))


def test_from_proto_round_trips_through_to_proto_and_json():
    a = an.Animation(
        name="rt", description="d", loop=True, hold_end=False, entry_time=0.3, exit_time=0.0,
        keyframes=[an.Keyframe(0.0), an.Keyframe(1.0, an.Ease.EASE_OUT, np.array([0.1, 0, 0, 0, 0, 20.0]),
                                                  [an.LegTarget(foot=np.array([0, 0, 30.0]))]
                                                  + [an.LegTarget(joints=np.array([0, 80.0, -110.0]))]
                                                  + stance_legs()[:4])],
        overlays=[an.Overlay(foot_channel=2, amplitude=5, frequency=2, phase=0.5, start=0, end=1)],
        params=[an.ParamSpec(an.ParamId.FOOT_LIFT, 0.5, 1.0, 1.5)],
    )
    b = from_proto(to_proto(a))
    assert an.validate(b) is None
    assert b.name == "rt" and b.loop and b.entry_time == pytest.approx(0.3)
    assert b.keyframes[1].ease == an.Ease.EASE_OUT
    assert np.allclose(b.keyframes[1].body, a.keyframes[1].body)
    assert b.keyframes[1].legs[1].is_joints()
    assert np.allclose(b.keyframes[1].legs[1].joints, [0, 80, -110])
    assert b.overlays[0].foot_channel == 2 and b.overlays[0].body_axis is None
    assert b.params[0].id == an.ParamId.FOOT_LIFT and b.params[0].default_value == 1.0
    text = json_text(b)
    assert '"defaultValue": 1.0' in text and '"holdEnd"' not in text


def test_from_proto_keeps_a_three_leg_keyframe_for_validate_to_reject():
    msg = to_proto(two_keyframes())
    for _ in range(3):
        msg.keyframes[1].legs.add().foot.z = 1.0
    a = from_proto(msg)
    assert len(a.keyframes[1].legs) == 3
    assert "0 or 6" in an.validate(a)


def test_load_json_reads_a_file(tmp_path):
    p = tmp_path / "x.json"
    p.write_text('{"name": "x", "schema": 1, "keyframes": [{"time": 0}, {"time": 0.5, "legs": [{"foot": {}}, {"foot": {}}, {"foot": {}}, {"foot": {}}, {"foot": {}}, {"joints": {"femur": 45}}]}]}')
    a = load_json(p)
    assert an.validate(a) is None
    assert a.keyframes[1].legs[5].joints[1] == 45
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `uv run pytest test_animation.py -q`
Expected: ImportError on `src.robot.animation`.

- [ ] **Step 3: Write the data model and validation**

`simulation/src/robot/animation.py`:

```python
"""Reference implementation of the animation evaluator and player.

This module is the port target for firmware/include/animation/ and app/src/lib/animation/.
The golden fixtures in animations/fixtures/expected.json are generated from it, and the other two
ports are tested against those fixtures, so a behaviour change here is a behaviour change on the
robot. It deliberately imports no protobuf code; animation_files.py does the conversion.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import IntEnum

import numpy as np

from src.robot.firmware_gait import DEFAULT_FEET, BodyState, Kinematics

JOINT_LIMIT_DEG = np.array([31.5, 90.0, 149.0])  # mirrors kinematics.h JOINT_LIMIT_DEG
SCHEMA_VERSION = 1
NAME_MAX = 32
DESCRIPTION_MAX = 96
KEYFRAME_MAX = 32
OVERLAY_MAX = 8
PARAM_MAX = 10
NAME_CHARS = frozenset("abcdefghijklmnopqrstuvwxyz0123456789_-")
DEFAULT_ENTRY_S = 0.5
DEFAULT_EXIT_S = 0.5
STEP_ARC_MM = 45.0
STEP_ARC_FULL_TRAVEL_MM = 40.0
STEP_ARC_MIN_TRAVEL_MM = 2.0


class Ease(IntEnum):
    LINEAR = 0
    EASE_IN = 1
    EASE_OUT = 2
    EASE_IN_OUT = 3


class ParamId(IntEnum):
    SPEED = 0
    BODY_X = 1
    BODY_Y = 2
    BODY_Z = 3
    BODY_ROLL = 4
    BODY_PITCH = 5
    BODY_YAW = 6
    FOOT_LIFT = 7
    OVERLAY_AMPLITUDE = 8
    REPEAT = 9


class BodyAxis(IntEnum):
    ROLL = 0
    PITCH = 1
    YAW = 2
    X = 3
    Y = 4
    Z = 5


BODY_PARAM_FOR_AXIS = (ParamId.BODY_ROLL, ParamId.BODY_PITCH, ParamId.BODY_YAW,
                       ParamId.BODY_X, ParamId.BODY_Y, ParamId.BODY_Z)


@dataclass
class LegTarget:
    foot: np.ndarray | None = None    # mm offset from the standing foot [x, y, z]
    joints: np.ndarray | None = None  # deg [coxa, femur, tibia]

    @staticmethod
    def stance() -> "LegTarget":
        return LegTarget(foot=np.zeros(3))

    def is_joints(self) -> bool:
        return self.joints is not None

    def copy(self) -> "LegTarget":
        return LegTarget(None if self.foot is None else self.foot.copy(),
                         None if self.joints is None else self.joints.copy())


@dataclass
class Keyframe:
    time: float
    ease: Ease = Ease.LINEAR
    body: np.ndarray = field(default_factory=lambda: np.zeros(6))
    legs: list[LegTarget] = field(default_factory=list)


@dataclass
class Overlay:
    body_axis: int | None = None
    foot_channel: int | None = None
    amplitude: float = 0.0
    frequency: float = 1.0
    phase: float = 0.0
    start: float = 0.0
    end: float = 0.0


@dataclass
class ParamSpec:
    id: ParamId
    min: float
    default_value: float
    max: float


@dataclass
class Animation:
    name: str
    description: str = ""
    schema: int = SCHEMA_VERSION
    loop: bool = False
    hold_end: bool = False
    entry_time: float = 0.0
    exit_time: float = 0.0
    keyframes: list[Keyframe] = field(default_factory=list)
    overlays: list[Overlay] = field(default_factory=list)
    params: list[ParamSpec] = field(default_factory=list)

    @property
    def duration(self) -> float:
        return self.keyframes[-1].time if self.keyframes else 0.0

    def entry_seconds(self) -> float:
        return self.entry_time if self.entry_time > 0 else DEFAULT_ENTRY_S

    def exit_seconds(self) -> float:
        return self.exit_time if self.exit_time > 0 else DEFAULT_EXIT_S


def leg_target(keyframe: Keyframe, leg: int) -> LegTarget:
    return keyframe.legs[leg] if keyframe.legs else LegTarget.stance()


def validate(anim: Animation) -> str | None:
    """Returns the first structural error, or None. The firmware and app apply the same rules."""
    if anim.schema != SCHEMA_VERSION:
        return f"schema {anim.schema} is not {SCHEMA_VERSION}"
    if not 1 <= len(anim.name) <= NAME_MAX or not set(anim.name) <= NAME_CHARS:
        return f"name must be 1-{NAME_MAX} characters of [a-z0-9_-]"
    if len(anim.description) > DESCRIPTION_MAX:
        return f"description longer than {DESCRIPTION_MAX}"
    if not anim.keyframes:
        return "at least one keyframe is required"
    if len(anim.keyframes) > KEYFRAME_MAX:
        return f"more than {KEYFRAME_MAX} keyframes"
    if anim.keyframes[0].time != 0.0:
        return "first keyframe must be at time 0"
    for i in range(1, len(anim.keyframes)):
        if anim.keyframes[i].time <= anim.keyframes[i - 1].time:
            return f"keyframe {i} time must increase"
    for i, k in enumerate(anim.keyframes):
        if len(k.legs) not in (0, 6):
            return f"keyframe {i} must have 0 or 6 legs"
    if len(anim.overlays) > OVERLAY_MAX:
        return f"more than {OVERLAY_MAX} overlays"
    for i, o in enumerate(anim.overlays):
        if (o.body_axis is None) == (o.foot_channel is None):
            return f"overlay {i} needs exactly one channel"
        if o.body_axis is not None and not 0 <= o.body_axis <= 5:
            return f"overlay {i} body_axis out of range"
        if o.foot_channel is not None and not 0 <= o.foot_channel <= 17:
            return f"overlay {i} foot_channel out of range"
        if o.start < 0 or o.start >= o.end:
            return f"overlay {i} window must have 0 <= start < end"
        if o.end > anim.duration:
            return f"overlay {i} end is after the last keyframe"
    if len(anim.params) > PARAM_MAX:
        return f"more than {PARAM_MAX} params"
    seen = set()
    for p in anim.params:
        if p.id in seen:
            return f"param {p.id.name} is not unique"
        seen.add(p.id)
        if not p.min <= p.default_value <= p.max:
            return f"param {p.id.name} needs min <= default_value <= max"
        if p.id == ParamId.SPEED and p.min <= 0:
            return "param SPEED needs a positive min"
    return None
```

`simulation/src/robot/animation_files.py`:

```python
"""Conversion between the animation dataclasses and the generated protobuf classes."""
from __future__ import annotations

from pathlib import Path

import numpy as np
from google.protobuf import json_format

from src.platform_shared import animation_pb2 as pb
from src.robot import animation as an


def _leg_from_proto(lt: pb.LegTarget) -> an.LegTarget:
    if lt.WhichOneof("target") == "joints":
        return an.LegTarget(joints=np.array([lt.joints.coxa, lt.joints.femur, lt.joints.tibia], dtype=float))
    return an.LegTarget(foot=np.array([lt.foot.x, lt.foot.y, lt.foot.z], dtype=float))


def _overlay_from_proto(o: pb.Overlay) -> an.Overlay:
    which = o.WhichOneof("channel")
    return an.Overlay(
        body_axis=o.body_axis if which == "body_axis" else None,
        foot_channel=o.foot_channel if which == "foot_channel" else None,
        amplitude=o.amplitude, frequency=o.frequency, phase=o.phase, start=o.start, end=o.end,
    )


def from_proto(msg: pb.Animation) -> an.Animation:
    keyframes = [
        an.Keyframe(
            time=k.time, ease=an.Ease(k.ease),
            body=np.array([k.body.roll, k.body.pitch, k.body.yaw, k.body.x, k.body.y, k.body.z], dtype=float),
            legs=[_leg_from_proto(lt) for lt in k.legs],
        )
        for k in msg.keyframes
    ]
    return an.Animation(
        name=msg.name, description=msg.description, schema=msg.schema, loop=msg.loop, hold_end=msg.hold_end,
        entry_time=msg.entry_time, exit_time=msg.exit_time, keyframes=keyframes,
        overlays=[_overlay_from_proto(o) for o in msg.overlays],
        params=[an.ParamSpec(an.ParamId(p.id), p.min, p.default_value, p.max) for p in msg.params],
    )


def to_proto(anim: an.Animation) -> pb.Animation:
    msg = pb.Animation(name=anim.name, description=anim.description, schema=anim.schema, loop=anim.loop,
                       hold_end=anim.hold_end, entry_time=anim.entry_time, exit_time=anim.exit_time)
    for k in anim.keyframes:
        kf = msg.keyframes.add(time=k.time, ease=int(k.ease))
        kf.body.roll, kf.body.pitch, kf.body.yaw, kf.body.x, kf.body.y, kf.body.z = map(float, k.body)
        for lt in k.legs:
            target = kf.legs.add()
            if lt.is_joints():
                target.joints.coxa, target.joints.femur, target.joints.tibia = map(float, lt.joints)
            else:
                target.foot.x, target.foot.y, target.foot.z = map(float, lt.foot)
    for o in anim.overlays:
        ov = msg.overlays.add(amplitude=o.amplitude, frequency=o.frequency, phase=o.phase, start=o.start, end=o.end)
        if o.body_axis is not None:
            ov.body_axis = o.body_axis
        else:
            ov.foot_channel = o.foot_channel
    for p in anim.params:
        msg.params.add(id=int(p.id), min=p.min, default_value=p.default_value, max=p.max)
    return msg


def json_text(anim: an.Animation) -> str:
    return json_format.MessageToJson(to_proto(anim), indent=2) + "\n"


def load_json(path: str | Path) -> an.Animation:
    return from_proto(json_format.Parse(Path(path).read_text(), pb.Animation()))


def save_json(anim: an.Animation, path: str | Path) -> None:
    Path(path).write_text(json_text(anim))


def load_binary(path: str | Path) -> an.Animation:
    return from_proto(pb.Animation.FromString(Path(path).read_bytes()))


def save_binary(anim: an.Animation, path: str | Path) -> None:
    Path(path).write_bytes(to_proto(anim).SerializeToString())
```

Note on `to_proto`: assigning `kf.body.roll = 0.0` marks `body` present, so a zero body is emitted as `"body": {}` in JSON.
That is harmless and keeps the converter simple.

- [ ] **Step 4: Run the tests**

Run: `uv run pytest test_animation.py -q`
Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add simulation/src/robot/animation.py simulation/src/robot/animation_files.py simulation/test_animation.py
git commit -m "✨ Adds the animation data model, validation and file conversion"
```

---

### Task 3: Evaluator and pose to angles

**Files:**
- Modify: `simulation/src/robot/animation.py`
- Test: `simulation/test_animation.py`

**Interfaces:**
- Produces: `Pose(body, legs)` dataclass with `Pose.stance()` and `copy()`; `ease_value(kind, t) -> float`; `resolve_params(anim, values: dict[ParamId, float] | None) -> np.ndarray` (length 10); `leg_joints_deg(kin, body6, foot_offset, leg) -> np.ndarray(3)`; `evaluate(anim, params, t, kin) -> Pose`; `pose_to_angles(pose, kin) -> tuple[np.ndarray(18), int]`.

- [ ] **Step 1: Write the failing tests**

Append to `simulation/test_animation.py`:

```python
def params_of(a, **values):
    return an.resolve_params(a, {an.ParamId[k]: v for k, v in values.items()})


def test_ease_curves_match_the_stash():
    assert an.ease_value(an.Ease.LINEAR, 0.3) == pytest.approx(0.3)
    assert an.ease_value(an.Ease.EASE_IN, 0.5) == pytest.approx(0.25)
    assert an.ease_value(an.Ease.EASE_OUT, 0.5) == pytest.approx(0.75)
    assert an.ease_value(an.Ease.EASE_IN_OUT, 0.25) == pytest.approx(0.125)
    assert an.ease_value(an.Ease.EASE_IN_OUT, 0.75) == pytest.approx(0.875)


def test_resolve_params_defaults_clamps_and_ignores_undeclared():
    a = two_keyframes(params=[an.ParamSpec(an.ParamId.SPEED, 0.5, 1.0, 2.0),
                              an.ParamSpec(an.ParamId.FOOT_LIFT, 0.0, 0.8, 1.0)])
    p = an.resolve_params(a, None)
    assert p[an.ParamId.SPEED] == 1.0 and p[an.ParamId.FOOT_LIFT] == 0.8 and p[an.ParamId.BODY_Z] == 1.0
    p = an.resolve_params(a, {an.ParamId.SPEED: 9.0, an.ParamId.BODY_Z: 0.1})
    assert p[an.ParamId.SPEED] == 2.0 and p[an.ParamId.BODY_Z] == 1.0


def test_evaluate_interpolates_body_and_feet_with_the_end_keyframe_ease():
    a = an.Animation(name="t", keyframes=[
        an.Keyframe(0.0),
        an.Keyframe(2.0, an.Ease.EASE_IN, np.array([0, 0, 0, 0, 0, 40.0]),
                    [an.LegTarget(foot=np.array([0, 0, 20.0]))] + stance_legs()[:5]),
    ])
    pose = an.evaluate(a, an.resolve_params(a, None), 1.0, KIN)
    assert pose.body[an.BodyAxis.Z] == pytest.approx(10.0)  # ease_in(0.5) = 0.25
    assert pose.legs[0].foot[2] == pytest.approx(5.0)
    assert np.array_equal(pose.legs[3].foot, np.zeros(3))


def test_evaluate_clamps_time_to_the_ends():
    a = two_keyframes()
    a.keyframes[1].body[an.BodyAxis.Z] = 30.0
    a.overlays.append(an.Overlay(body_axis=an.BodyAxis.ROLL, amplitude=1.0, frequency=0.25, phase=math.pi / 2,
                                 start=0.2, end=0.8))
    p = an.resolve_params(a, None)
    assert an.evaluate(a, p, -0.1, KIN).body[an.BodyAxis.Z] == 0.0
    assert an.evaluate(a, p, 5.0, KIN).body[an.BodyAxis.Z] == pytest.approx(30.0)
    assert an.evaluate(a, p, 5.0, KIN).body[an.BodyAxis.ROLL] == 0.0  # overlay window is closed


def test_overlay_adds_a_sine_on_a_body_or_foot_channel_and_scales_with_the_param():
    a = two_keyframes(overlays=[
        an.Overlay(body_axis=an.BodyAxis.YAW, amplitude=0.2, frequency=1.0, phase=0.0, start=0.0, end=1.0),
        an.Overlay(foot_channel=3 * 2 + 2, amplitude=10.0, frequency=1.0, phase=0.0, start=0.0, end=1.0),
    ], params=[an.ParamSpec(an.ParamId.OVERLAY_AMPLITUDE, 0.0, 1.0, 2.0)])
    pose = an.evaluate(a, params_of(a, OVERLAY_AMPLITUDE=0.5), 0.25, KIN)  # sin(pi/2) = 1
    assert pose.body[an.BodyAxis.YAW] == pytest.approx(0.1)
    assert pose.legs[2].foot[2] == pytest.approx(5.0)


def test_overlay_on_a_joint_leg_is_ignored():
    a = two_keyframes(overlays=[an.Overlay(foot_channel=0, amplitude=10.0, frequency=1.0, start=0.0, end=1.0)])
    a.keyframes[0].legs = [an.LegTarget(joints=np.array([0, 60.0, -100.0]))] + stance_legs()[:5]
    a.keyframes[1].legs = [an.LegTarget(joints=np.array([0, 60.0, -100.0]))] + stance_legs()[:5]
    pose = an.evaluate(a, an.resolve_params(a, None), 0.25, KIN)
    assert pose.legs[0].is_joints() and np.allclose(pose.legs[0].joints, [0, 60, -100])


def test_body_and_foot_lift_multipliers_apply_after_overlays():
    a = two_keyframes(overlays=[an.Overlay(body_axis=an.BodyAxis.ROLL, amplitude=0.2, frequency=1.0, start=0.0, end=1.0)],
                      params=[an.ParamSpec(an.ParamId.BODY_ROLL, 0, 1, 2), an.ParamSpec(an.ParamId.FOOT_LIFT, 0, 1, 2)])
    a.keyframes[1].body[an.BodyAxis.ROLL] = 0.4
    a.keyframes[1].legs = [an.LegTarget(foot=np.array([5.0, 0, 20.0]))] + stance_legs()[:5]
    pose = an.evaluate(a, params_of(a, BODY_ROLL=0.5, FOOT_LIFT=2.0), 0.25, KIN)
    assert pose.body[an.BodyAxis.ROLL] == pytest.approx((0.1 + 0.2) * 0.5)
    assert np.allclose(pose.legs[0].foot, [1.25, 0, 10.0])  # x is not lifted, z is


def test_mixed_foot_and_joint_keyframes_interpolate_in_joint_space():
    a = two_keyframes()
    a.keyframes[0].legs = [an.LegTarget(foot=np.zeros(3))] + stance_legs()[:5]
    a.keyframes[1].legs = [an.LegTarget(joints=np.array([0, 80.0, -110.0]))] + stance_legs()[:5]
    start = an.leg_joints_deg(KIN, np.zeros(6), np.zeros(3), 0)
    pose = an.evaluate(a, an.resolve_params(a, None), 0.5, KIN)
    assert pose.legs[0].is_joints()
    assert np.allclose(pose.legs[0].joints, (start + np.array([0, 80.0, -110.0])) / 2)


def test_leg_joints_deg_matches_the_full_body_ik():
    body = np.array([0.05, -0.02, 0.1, 5.0, -3.0, 12.0])
    b = BodyState(omega=0.05, phi=-0.02, psi=0.1, xm=5.0, ym=-3.0, zm=12.0)
    b.feet[4, :3] += [3.0, 4.0, 15.0]
    assert np.allclose(an.leg_joints_deg(KIN, body, np.array([3.0, 4.0, 15.0]), 4), KIN.inverse_kinematics(b)[12:15])


def test_pose_to_angles_overrides_joint_legs_and_clamps_with_a_mask():
    pose = an.Pose.stance()
    pose.legs[1] = an.LegTarget(joints=np.array([40.0, 95.0, -100.0]))  # coxa and femur over the limit
    pose.legs[5] = an.LegTarget(foot=np.array([0, 0, 120.0]))           # unreachable lift, IK clamps
    angles, mask = an.pose_to_angles(pose, KIN)
    assert angles.shape == (18,)
    assert angles[3] == 31.5 and angles[4] == 90.0 and angles[5] == -100.0
    assert mask & (1 << 3) and mask & (1 << 4) and not mask & (1 << 5)
    assert mask & (1 << 16)  # femur of leg 5 pinned
    assert not mask & 0b111  # leg 0 at stance is clean


def test_stance_pose_reproduces_the_standing_angles():
    angles, mask = an.pose_to_angles(an.Pose.stance(), KIN)
    assert mask == 0
    assert np.allclose(angles, KIN.inverse_kinematics(BodyState()))
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `uv run pytest test_animation.py -q`
Expected: AttributeError for `ease_value`, `evaluate`, `Pose`.

- [ ] **Step 3: Write the evaluator**

Append to `simulation/src/robot/animation.py`:

```python
@dataclass
class Pose:
    body: np.ndarray                 # [roll, pitch, yaw, x, y, z] offsets
    legs: list[LegTarget]            # exactly 6

    @staticmethod
    def stance() -> "Pose":
        return Pose(np.zeros(6), [LegTarget.stance() for _ in range(6)])

    def copy(self) -> "Pose":
        return Pose(self.body.copy(), [l.copy() for l in self.legs])


def ease_value(kind: Ease, t: float) -> float:
    if kind == Ease.EASE_IN:
        return t * t
    if kind == Ease.EASE_OUT:
        return t * (2.0 - t)
    if kind == Ease.EASE_IN_OUT:
        return 2.0 * t * t if t < 0.5 else -1.0 + (4.0 - 2.0 * t) * t
    return t


def resolve_params(anim: Animation, values: dict[ParamId, float] | None) -> np.ndarray:
    """Every id gets a value: declared ids take the caller's value clamped to the spec, else the
    default; undeclared ids are 1 (the neutral multiplier and a single play)."""
    out = np.ones(len(ParamId))
    for spec in anim.params:
        v = spec.default_value if values is None or spec.id not in values else values[spec.id]
        out[spec.id] = min(max(v, spec.min), spec.max)
    return out


def _body_state(body6: np.ndarray) -> BodyState:
    return BodyState(omega=float(body6[0]), phi=float(body6[1]), psi=float(body6[2]),
                     xm=float(body6[3]), ym=float(body6[4]), zm=float(body6[5]))


def leg_joints_deg(kin: Kinematics, body6: np.ndarray, foot_offset: np.ndarray, leg: int) -> np.ndarray:
    b = _body_state(body6)
    b.feet[leg, :3] += foot_offset
    return kin.inverse_kinematics(b)[leg * 3:leg * 3 + 3]


def _segment(anim: Animation, t: float) -> tuple[Keyframe, Keyframe, float]:
    """The keyframe pair bracketing t and the eased fraction between them, with t clamped."""
    kfs = anim.keyframes
    if t <= 0.0 or len(kfs) == 1:
        return kfs[0], kfs[0], 0.0
    if t >= kfs[-1].time:
        return kfs[-1], kfs[-1], 0.0
    i = 1
    while kfs[i].time < t:
        i += 1
    k0, k1 = kfs[i - 1], kfs[i]
    return k0, k1, ease_value(k1.ease, (t - k0.time) / (k1.time - k0.time))


def _lerp_leg(kin: Kinematics, a: LegTarget, b: LegTarget, body_a: np.ndarray, body_b: np.ndarray,
              leg: int, u: float) -> LegTarget:
    if not a.is_joints() and not b.is_joints():
        return LegTarget(foot=a.foot + (b.foot - a.foot) * u)
    ja = a.joints if a.is_joints() else leg_joints_deg(kin, body_a, a.foot, leg)
    jb = b.joints if b.is_joints() else leg_joints_deg(kin, body_b, b.foot, leg)
    return LegTarget(joints=ja + (jb - ja) * u)


def evaluate(anim: Animation, params: np.ndarray, t: float, kin: Kinematics) -> Pose:
    k0, k1, u = _segment(anim, t)
    t = min(max(t, 0.0), anim.duration)
    body = k0.body + (k1.body - k0.body) * u
    legs = [_lerp_leg(kin, leg_target(k0, i), leg_target(k1, i), k0.body, k1.body, i, u) for i in range(6)]
    for o in anim.overlays:
        if not o.start <= t <= o.end:
            continue
        v = o.amplitude * params[ParamId.OVERLAY_AMPLITUDE] * math.sin(2.0 * math.pi * o.frequency * t + o.phase)
        if o.body_axis is not None:
            body[o.body_axis] += v
        elif not legs[o.foot_channel // 3].is_joints():
            legs[o.foot_channel // 3].foot[o.foot_channel % 3] += v
    for axis, pid in enumerate(BODY_PARAM_FOR_AXIS):
        body[axis] *= params[pid]
    for leg in legs:
        if not leg.is_joints():
            leg.foot[2] *= params[ParamId.FOOT_LIFT]
    return Pose(body, legs)


def pose_to_angles(pose: Pose, kin: Kinematics) -> tuple[np.ndarray, int]:
    """18 servo angles (deg, IK order) and an 18-bit mask of the joints that hit a limit."""
    b = _body_state(pose.body)
    for i, leg in enumerate(pose.legs):
        if not leg.is_joints():
            b.feet[i, :3] += leg.foot
    angles = kin.inverse_kinematics(b)
    for i, leg in enumerate(pose.legs):
        if leg.is_joints():
            angles[i * 3:i * 3 + 3] = leg.joints
    limit = np.tile(JOINT_LIMIT_DEG, 6)
    clamped = np.clip(angles, -limit, limit)
    mask = 0
    for j in np.flatnonzero(clamped != angles):
        mask |= 1 << int(j)
    return clamped, mask
```

Note for the mixed-leg case: the foot endpoint's IK runs against that keyframe's own body pose, `k0.body` or `k1.body`, before multipliers.
The firmware and app ports must do the same or the fixtures will not match.

- [ ] **Step 4: Run the tests**

Run: `uv run pytest test_animation.py -q`
Expected: all pass.
`test_pose_to_angles_overrides_joint_legs_and_clamps_with_a_mask` relies on a 120 mm lift being unreachable at nominal stance (the measured femur ceiling is 54 mm); if the femur does not reach 90 there, raise the lift to 200 mm rather than weakening the assertion.

- [ ] **Step 5: Commit**

```bash
git add simulation/src/robot/animation.py simulation/test_animation.py
git commit -m "✨ Adds the animation evaluator and the pose to servo angle path"
```

---

### Task 4: Player state machine

**Files:**
- Modify: `simulation/src/robot/animation.py`
- Test: `simulation/test_animation.py`

**Interfaces:**
- Produces: `State` IntEnum `IDLE, ENTRY, PLAYING, HOLD, EXIT`; `capture_pose(body: BodyState) -> Pose`; `class Player(kin)` with `state`, `animation`, `params`, `t`, `last_pose`, `play(anim, values=None, live=None)`, `stop()`, `update(dt) -> Pose`.

- [ ] **Step 1: Write the failing tests**

Append to `simulation/test_animation.py`:

```python
def lifted_anim(**kw):
    """Leg 0 relocates 30 mm forward and 20 mm up over 1 s, body crouches 20 mm."""
    a = an.Animation(name="lift", keyframes=[
        an.Keyframe(0.0, legs=[an.LegTarget(foot=np.array([0, 30.0, 20.0]))] + stance_legs()[:5]),
        an.Keyframe(1.0, body=np.array([0, 0, 0, 0, 0, 20.0]),
                    legs=[an.LegTarget(foot=np.array([0, 30.0, 20.0]))] + stance_legs()[:5]),
    ], **kw)
    return a


DT = 0.02


def run(player, steps):
    """Advance by whole control steps. Counts are chosen with one step of slack past every
    transition so float accumulation in the blend clock cannot flip an assertion."""
    poses = []
    for _ in range(steps):
        poses.append(player.update(DT))
    return poses


def test_player_is_idle_at_stance_until_play():
    p = an.Player(KIN)
    assert p.state == an.State.IDLE
    pose = p.update(DT)
    assert np.array_equal(pose.body, np.zeros(6)) and not pose.legs[0].is_joints()


def test_entry_blends_from_the_live_pose_and_arcs_a_relocating_foot():
    p = an.Player(KIN)
    p.play(lifted_anim(entry_time=0.4))
    assert p.state == an.State.ENTRY
    mid = run(p, 10)[-1]                        # u = 0.5, ease_in_out(0.5) = 0.5, sin(pi/2) = 1
    travel = 30.0
    expected_z = 10.0 + an.STEP_ARC_MM * min(1.0, travel / an.STEP_ARC_FULL_TRAVEL_MM)
    assert mid.legs[0].foot[1] == pytest.approx(15.0, abs=1e-6)
    assert mid.legs[0].foot[2] == pytest.approx(expected_z, abs=1e-6)
    assert mid.legs[1].foot[2] == pytest.approx(0.0)  # planted feet get no arc
    end = run(p, 11)[-1]
    assert p.state == an.State.PLAYING
    assert np.allclose(end.legs[0].foot, [0, 30.0, 20.0])


def test_playing_then_exit_then_idle_with_a_step_home():
    p = an.Player(KIN)
    p.play(lifted_anim(entry_time=0.2, exit_time=0.4))
    run(p, 11)
    run(p, 52)
    assert p.state == an.State.EXIT
    mid = run(p, 5)[-1]
    y, z = mid.legs[0].foot[1], mid.legs[0].foot[2]
    assert 0.0 < y < 30.0
    assert z > y * 20.0 / 30.0  # above the straight line home, so the foot is arcing
    run(p, 20)
    assert p.state == an.State.IDLE
    assert np.allclose(p.last_pose.body, 0) and np.allclose(p.last_pose.legs[0].foot, 0)


def test_hold_end_freezes_on_the_last_keyframe():
    p = an.Player(KIN)
    p.play(lifted_anim(entry_time=0.2, hold_end=True))
    run(p, 75)
    assert p.state == an.State.HOLD
    pose = run(p, 50)[-1]
    assert pose.body[an.BodyAxis.Z] == pytest.approx(20.0)
    p.stop()
    assert p.state == an.State.EXIT


def test_speed_scales_only_the_playing_clock():
    slow, fast = an.Player(KIN), an.Player(KIN)
    a = lifted_anim(entry_time=0.2, params=[an.ParamSpec(an.ParamId.SPEED, 0.5, 1.0, 4.0)])
    slow.play(a)
    fast.play(a, {an.ParamId.SPEED: 2.0})
    run(slow, 11), run(fast, 11)
    assert slow.state == fast.state == an.State.PLAYING  # entry took the same wall time
    run(slow, 10), run(fast, 10)
    assert 10 * DT <= slow.t <= 11 * DT + 1e-9
    assert fast.t == pytest.approx(2 * slow.t)


def test_loop_wraps_and_repeat_counts_plays():
    p = an.Player(KIN)
    p.play(lifted_anim(entry_time=0.2, loop=True))
    run(p, 11)
    run(p, 125)
    assert p.state == an.State.PLAYING and 0.0 <= p.t < 1.0
    q = an.Player(KIN)
    q.play(lifted_anim(entry_time=0.2, params=[an.ParamSpec(an.ParamId.REPEAT, 1, 2, 5)]))
    run(q, 11)
    run(q, 74)
    assert q.state == an.State.PLAYING  # second play under way
    run(q, 30)
    assert q.state == an.State.EXIT


def test_zero_dt_and_a_single_keyframe_reach_hold_or_exit():
    p = an.Player(KIN)
    one = an.Animation(name="one", entry_time=0.2, keyframes=[an.Keyframe(0.0, body=np.array([0, 0, 0, 0, 0, 10.0]))])
    p.play(one)
    p.update(0.0)
    assert p.state == an.State.ENTRY
    run(p, 13)
    assert p.state == an.State.EXIT
    q = an.Player(KIN)
    one.hold_end = True
    q.play(one)
    run(q, 13)
    assert q.state == an.State.HOLD


def test_play_during_exit_enters_from_the_current_blend_without_a_jump():
    p = an.Player(KIN)
    p.play(lifted_anim(entry_time=0.2, exit_time=0.4))
    run(p, 11)
    run(p, 55)
    before = run(p, 5)[-1]
    assert p.state == an.State.EXIT
    p.play(lifted_anim(entry_time=0.4))
    after = p.update(0.0)
    assert p.state == an.State.ENTRY
    assert np.allclose(after.body, before.body) and np.allclose(after.legs[0].foot, before.legs[0].foot)


def test_entry_toward_a_joint_leg_blends_in_joint_space():
    a = an.Animation(name="j", entry_time=0.4, keyframes=[
        an.Keyframe(0.0, legs=[an.LegTarget(joints=np.array([0, 80.0, -110.0]))] + stance_legs()[:5]),
        an.Keyframe(1.0, legs=[an.LegTarget(joints=np.array([0, 80.0, -110.0]))] + stance_legs()[:5]),
    ])
    p = an.Player(KIN)
    p.play(a)
    mid = run(p, 10)[-1]
    start = an.leg_joints_deg(KIN, np.zeros(6), np.zeros(3), 0)
    assert mid.legs[0].is_joints()
    assert np.allclose(mid.legs[0].joints, (start + [0, 80.0, -110.0]) / 2)


def test_capture_pose_reads_offsets_from_a_body_state():
    b = BodyState(omega=0.1, zm=15.0)
    b.feet[2, :3] += [1.0, 2.0, 3.0]
    pose = an.capture_pose(b)
    assert pose.body[an.BodyAxis.ROLL] == 0.1 and pose.body[an.BodyAxis.Z] == 15.0
    assert np.allclose(pose.legs[2].foot, [1, 2, 3]) and np.allclose(pose.legs[0].foot, 0)
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `uv run pytest test_animation.py -q`
Expected: AttributeError for `Player`.

- [ ] **Step 3: Write the player**

Append to `simulation/src/robot/animation.py`:

```python
class State(IntEnum):
    IDLE = 0
    ENTRY = 1
    PLAYING = 2
    HOLD = 3
    EXIT = 4


def capture_pose(body: BodyState) -> Pose:
    body6 = np.array([body.omega, body.phi, body.psi, body.xm, body.ym, body.zm], dtype=float)
    legs = [LegTarget(foot=(body.feet[i, :3] - DEFAULT_FEET[i, :3]).astype(float)) for i in range(6)]
    return Pose(body6, legs)


def _blend_targets(kin: Kinematics, src: Pose, dst: Pose) -> tuple[Pose, Pose]:
    """Legs where either side is a joint target are converted to joints on both sides once, so the
    blend itself is a plain lerp. Foot legs keep their offsets and get the step arc."""
    a, b = src.copy(), dst.copy()
    for i in range(6):
        if a.legs[i].is_joints() or b.legs[i].is_joints():
            if not a.legs[i].is_joints():
                a.legs[i] = LegTarget(joints=leg_joints_deg(kin, a.body, a.legs[i].foot, i))
            if not b.legs[i].is_joints():
                b.legs[i] = LegTarget(joints=leg_joints_deg(kin, b.body, b.legs[i].foot, i))
    return a, b


def _blend(a: Pose, b: Pose, u: float) -> Pose:
    e = ease_value(Ease.EASE_IN_OUT, u)
    legs = []
    for la, lb in zip(a.legs, b.legs):
        if la.is_joints():
            legs.append(LegTarget(joints=la.joints + (lb.joints - la.joints) * e))
            continue
        foot = la.foot + (lb.foot - la.foot) * e
        travel = math.hypot(*(lb.foot[:2] - la.foot[:2]))
        if travel > STEP_ARC_MIN_TRAVEL_MM:
            foot[2] += STEP_ARC_MM * min(1.0, travel / STEP_ARC_FULL_TRAVEL_MM) * math.sin(math.pi * u)
        legs.append(LegTarget(foot=foot))
    return Pose(a.body + (b.body - a.body) * e, legs)


class Player:
    """Entry -> Playing -> Hold | Exit -> Done, around evaluate(). Mirrors the firmware AnimationPlayer."""

    def __init__(self, kin: Kinematics | None = None):
        self.kin = kin or Kinematics()
        self.state = State.IDLE
        self.animation: Animation | None = None
        self.params = np.ones(len(ParamId))
        self.t = 0.0
        self.last_pose = Pose.stance()
        self._plays_done = 0
        self._blend_t = 0.0
        self._blend_seconds = 1.0
        self._blend_from = Pose.stance()
        self._blend_to = Pose.stance()

    def play(self, anim: Animation, values: dict[ParamId, float] | None = None, live: Pose | None = None) -> None:
        self.animation = anim
        self.params = resolve_params(anim, values)
        self.t = 0.0
        self._plays_done = 0
        start = live if live is not None else self.last_pose
        self._start_blend(start, evaluate(anim, self.params, 0.0, self.kin), anim.entry_seconds(), State.ENTRY)

    def stop(self) -> None:
        if self.state == State.IDLE:
            return
        self._start_blend(self.last_pose, Pose.stance(), self.animation.exit_seconds(), State.EXIT)

    def update(self, dt: float) -> Pose:
        if self.state == State.IDLE:
            return self.last_pose
        if self.state in (State.ENTRY, State.EXIT):
            pose = self._advance_blend(dt)
        elif self.state == State.HOLD:
            pose = evaluate(self.animation, self.params, self.animation.duration, self.kin)
        else:
            pose = self._advance_playing(dt)
        self.last_pose = pose
        return pose

    def _start_blend(self, src: Pose, dst: Pose, seconds: float, state: State) -> None:
        self._blend_from, self._blend_to = _blend_targets(self.kin, src, dst)
        self._blend_seconds = seconds
        self._blend_t = 0.0
        self.state = state

    def _advance_blend(self, dt: float) -> Pose:
        self._blend_t += dt
        u = min(1.0, self._blend_t / self._blend_seconds)
        pose = _blend(self._blend_from, self._blend_to, u)
        if u >= 1.0:
            if self.state == State.ENTRY:
                self.state = State.PLAYING
                self.t = 0.0
            else:
                self.state = State.IDLE
        return pose

    def _advance_playing(self, dt: float) -> Pose:
        anim = self.animation
        duration = anim.duration
        self.t += dt * self.params[ParamId.SPEED]
        if anim.loop:
            self.t = self.t % duration if duration > 0.0 else 0.0
            return evaluate(anim, self.params, self.t, self.kin)
        if self.t < duration:
            return evaluate(anim, self.params, self.t, self.kin)
        self._plays_done += 1
        if self._plays_done < int(round(self.params[ParamId.REPEAT])):
            self.t = self.t - duration if duration > 0.0 else 0.0
            return evaluate(anim, self.params, self.t, self.kin)
        final = evaluate(anim, self.params, duration, self.kin)
        if anim.hold_end:
            self.state = State.HOLD
        else:
            self.last_pose = final
            self.stop()
        return final
```

`stop()` reads `self.last_pose`, which is why `_advance_playing` writes the final pose there before calling it: Exit must start from the last keyframe, not from the previous tick.

- [ ] **Step 4: Run the tests**

Run: `uv run pytest test_animation.py -q`
Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add simulation/src/robot/animation.py simulation/test_animation.py
git commit -m "✨ Adds the animation player state machine"
```

---

### Task 5: Parity fixtures

**Files:**
- Create: `animations/fixtures/fx_mixed_legs.json`, `fx_overlay.json`, `fx_params.json`, `fx_single.json`
- Create: `simulation/gen_animation_fixtures.py`
- Create: `animations/fixtures/expected.json` (generated, committed)
- Test: `simulation/test_animation_fixtures.py`

**Interfaces:**
- Produces: `expected.json` with the layout below, consumed by the firmware and app parity tests in plans 2 and 3.
- `gen_animation_fixtures.generate() -> dict` and `main()` writing the file.

`expected.json` layout:

```json
{
  "tolerance": 1e-4,
  "evaluate": [
    {"animation": "fx_mixed_legs", "params": {"SPEED": 1.0},
     "samples": [{"t": 0.0, "angles": [18 floats], "mask": 0}, ...]}
  ],
  "player": [
    {"animation": "fx_mixed_legs", "params": {}, "dt": 0.02,
     "live": {"body": [6 floats], "feet": [[3 floats] x 6]},
     "events": [{"step": 45, "action": "stop"}, {"step": 30, "action": "play", "animation": "fx_overlay", "params": {}}],
     "trace": [{"state": "ENTRY", "angles": [18 floats], "mask": 0}, ...]}
  ]
}
```

Angles are the 18 servo degrees after `pose_to_angles`, which is the only output every platform shares.
A player case starts from the given live pose, calls `update(dt)` once per step, applies an event before the update of the named step, and records every step.

- [ ] **Step 1: Write the fixture animations**

`animations/fixtures/fx_mixed_legs.json` (leg 0 goes foot, joints, foot; every ease appears):

```json
{
  "name": "fx_mixed_legs",
  "description": "Parity fixture: foot and joint targets on one leg, every ease",
  "schema": 1,
  "entryTime": 0.4,
  "exitTime": 0.6,
  "keyframes": [
    {"time": 0, "legs": [{"foot": {"y": 20, "z": 10}}, {"foot": {}}, {"foot": {}}, {"foot": {}}, {"foot": {}}, {"foot": {}}]},
    {"time": 0.5, "ease": "EASE_IN", "body": {"roll": 0.1, "z": 15},
     "legs": [{"joints": {"coxa": 5, "femur": 75, "tibia": -120}}, {"foot": {}}, {"foot": {}}, {"foot": {"z": 25}}, {"foot": {}}, {"foot": {}}]},
    {"time": 1.1, "ease": "EASE_OUT", "body": {"pitch": -0.08, "y": -10},
     "legs": [{"joints": {"coxa": -5, "femur": 60, "tibia": -90}}, {"foot": {}}, {"foot": {}}, {"foot": {"x": 10, "z": 25}}, {"foot": {}}, {"foot": {}}]},
    {"time": 1.8, "ease": "EASE_IN_OUT", "body": {"yaw": 0.15},
     "legs": [{"foot": {"y": -15, "z": 30}}, {"foot": {}}, {"foot": {}}, {"foot": {}}, {"foot": {}}, {"foot": {}}]},
    {"time": 2.2, "ease": "LINEAR"}
  ]
}
```

`animations/fixtures/fx_overlay.json` (looping, body and foot overlays with windows):

```json
{
  "name": "fx_overlay",
  "description": "Parity fixture: body and foot overlays with windows, looping",
  "schema": 1,
  "loop": true,
  "keyframes": [{"time": 0}, {"time": 1.2, "body": {"z": 8}}],
  "overlays": [
    {"bodyAxis": 0, "amplitude": 0.12, "frequency": 1.5, "phase": 0, "start": 0, "end": 1.2},
    {"bodyAxis": 5, "amplitude": 6, "frequency": 0.8333, "phase": 1.5708, "start": 0.2, "end": 1.0},
    {"footChannel": 5, "amplitude": 12, "frequency": 2, "phase": 0.3, "start": 0, "end": 0.9}
  ],
  "params": [{"id": "OVERLAY_AMPLITUDE", "min": 0, "defaultValue": 1, "max": 2}]
}
```

`animations/fixtures/fx_params.json` (declares all ten parameters):

```json
{
  "name": "fx_params",
  "description": "Parity fixture: every parameter declared",
  "schema": 1,
  "keyframes": [
    {"time": 0},
    {"time": 1, "body": {"roll": 0.1, "pitch": 0.1, "yaw": 0.1, "x": 10, "y": 10, "z": 10},
     "legs": [{"foot": {"x": 5, "y": 5, "z": 20}}, {"foot": {"z": 20}}, {"foot": {}}, {"foot": {}}, {"foot": {}}, {"foot": {"z": 15}}]}
  ],
  "overlays": [{"bodyAxis": 3, "amplitude": 4, "frequency": 1, "phase": 0, "start": 0, "end": 1}],
  "params": [
    {"id": "SPEED", "min": 0.25, "defaultValue": 1, "max": 3},
    {"id": "BODY_X", "min": 0, "defaultValue": 1, "max": 2},
    {"id": "BODY_Y", "min": 0, "defaultValue": 1, "max": 2},
    {"id": "BODY_Z", "min": 0, "defaultValue": 1, "max": 2},
    {"id": "BODY_ROLL", "min": 0, "defaultValue": 1, "max": 2},
    {"id": "BODY_PITCH", "min": 0, "defaultValue": 1, "max": 2},
    {"id": "BODY_YAW", "min": 0, "defaultValue": 1, "max": 2},
    {"id": "FOOT_LIFT", "min": 0, "defaultValue": 1, "max": 2},
    {"id": "OVERLAY_AMPLITUDE", "min": 0, "defaultValue": 1, "max": 2},
    {"id": "REPEAT", "min": 1, "defaultValue": 1, "max": 4}
  ]
}
```

`animations/fixtures/fx_single.json`:

```json
{
  "name": "fx_single",
  "description": "Parity fixture: one keyframe, hold at end",
  "schema": 1,
  "holdEnd": true,
  "entryTime": 0.3,
  "keyframes": [{"time": 0, "body": {"z": 25, "roll": 0.2},
                 "legs": [{"foot": {}}, {"foot": {}}, {"foot": {}}, {"joints": {"coxa": 0, "femur": 85, "tibia": -130}}, {"foot": {}}, {"foot": {}}]}]
}
```

- [ ] **Step 2: Write the generator**

`simulation/gen_animation_fixtures.py`:

```python
"""Writes animations/fixtures/expected.json from the reference implementation.

The firmware and app parity tests read that file; test_animation_fixtures.py fails when it is stale.
    uv run python gen_animation_fixtures.py
"""
import json
from pathlib import Path

import numpy as np

from src.robot import animation as an
from src.robot.animation_files import load_json
from src.robot.firmware_gait import BodyState, Kinematics

ROOT = Path(__file__).resolve().parents[1]
FIXTURE_DIR = ROOT / "animations" / "fixtures"
EXPECTED = FIXTURE_DIR / "expected.json"
TOLERANCE = 1e-4

EVALUATE_CASES = [
    ("fx_mixed_legs", {}),
    ("fx_overlay", {}),
    ("fx_overlay", {"OVERLAY_AMPLITUDE": 0.5}),
    ("fx_params", {}),
    ("fx_params", {"SPEED": 2.0, "BODY_X": 2.0, "BODY_Y": 0.5, "BODY_Z": 1.5, "BODY_ROLL": 0.0,
                   "BODY_PITCH": 2.0, "BODY_YAW": 0.5, "FOOT_LIFT": 2.0, "OVERLAY_AMPLITUDE": 0.25, "REPEAT": 3}),
    ("fx_single", {}),
]

DISPLACED_LIVE = {"body": [0.05, -0.03, 0.0, 8.0, -6.0, 12.0],
                  "feet": [[15.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, -10.0, 5.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [-5.0, 5.0, 0.0]]}

PLAYER_CASES = [
    {"animation": "fx_mixed_legs", "params": {}, "dt": 0.02, "live": DISPLACED_LIVE, "events": [], "steps": 170},
    {"animation": "fx_mixed_legs", "params": {}, "dt": 0.02, "live": DISPLACED_LIVE,
     "events": [{"step": 45, "action": "stop"}], "steps": 90},
    {"animation": "fx_mixed_legs", "params": {}, "dt": 0.02, "live": DISPLACED_LIVE,
     "events": [{"step": 40, "action": "play", "animation": "fx_overlay", "params": {"OVERLAY_AMPLITUDE": 1.5}}], "steps": 120},
    {"animation": "fx_single", "params": {}, "dt": 0.02, "live": DISPLACED_LIVE,
     "events": [{"step": 40, "action": "stop"}], "steps": 80},
    {"animation": "fx_params", "params": {"SPEED": 2.0, "REPEAT": 2}, "dt": 0.025, "live": DISPLACED_LIVE,
     "events": [], "steps": 100},
    {"animation": "fx_overlay", "params": {}, "dt": 0.02, "live": DISPLACED_LIVE,
     "events": [{"step": 110, "action": "stop"}], "steps": 150},
]


def sample_times(anim: an.Animation) -> list[float]:
    times = [k.time for k in anim.keyframes]
    mids = [(a + b) / 2 for a, b in zip(times, times[1:])]
    return sorted(set(times + mids + [-0.1, anim.duration + 0.5, 0.137, 0.731]))


def param_values(values: dict) -> dict[an.ParamId, float]:
    return {an.ParamId[k]: float(v) for k, v in values.items()}


def live_pose(live: dict) -> an.Pose:
    b = BodyState(omega=live["body"][0], phi=live["body"][1], psi=live["body"][2],
                  xm=live["body"][3], ym=live["body"][4], zm=live["body"][5])
    b.feet[:, :3] += np.array(live["feet"])
    return an.capture_pose(b)


def generate() -> dict:
    kin = Kinematics()
    anims = {p.stem: load_json(p) for p in sorted(FIXTURE_DIR.glob("fx_*.json"))}
    for name, a in anims.items():
        err = an.validate(a)
        if err:
            raise SystemExit(f"{name}: {err}")

    evaluate = []
    for name, values in EVALUATE_CASES:
        a = anims[name]
        params = an.resolve_params(a, param_values(values))
        samples = []
        for t in sample_times(a):
            angles, mask = an.pose_to_angles(an.evaluate(a, params, t, kin), kin)
            samples.append({"t": t, "angles": [round(float(x), 6) for x in angles], "mask": mask})
        evaluate.append({"animation": name, "params": values, "samples": samples})

    player = []
    for case in PLAYER_CASES:
        p = an.Player(kin)
        p.play(anims[case["animation"]], param_values(case["params"]), live_pose(case["live"]))
        events = {e["step"]: e for e in case["events"]}
        trace = []
        for step in range(case["steps"]):
            e = events.get(step)
            if e and e["action"] == "stop":
                p.stop()
            elif e and e["action"] == "play":
                p.play(anims[e["animation"]], param_values(e["params"]))
            angles, mask = an.pose_to_angles(p.update(case["dt"]), kin)
            trace.append({"state": p.state.name, "angles": [round(float(x), 6) for x in angles], "mask": mask})
        player.append({k: v for k, v in case.items() if k != "steps"} | {"trace": trace})

    return {"tolerance": TOLERANCE, "evaluate": evaluate, "player": player}


def main() -> None:
    EXPECTED.write_text(json.dumps(generate(), indent=1) + "\n")
    print(f"wrote {EXPECTED}")


if __name__ == "__main__":
    main()
```

- [ ] **Step 3: Write the test**

`simulation/test_animation_fixtures.py`:

```python
"""animations/fixtures/expected.json must be what the reference implementation produces today."""
import json

import numpy as np

import gen_animation_fixtures as gen
from src.robot import animation as an
from src.robot.animation_files import load_json


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
        for sc, sf in zip(c["samples"], f["samples"]):
            assert sc["t"] == sf["t"] and sc["mask"] == sf["mask"]
            assert np.allclose(sc["angles"], sf["angles"], atol=fresh["tolerance"])
    assert len(committed["player"]) == len(fresh["player"])
    for c, f in zip(committed["player"], fresh["player"]):
        assert c["animation"] == f["animation"] and c["events"] == f["events"]
        assert [s["state"] for s in c["trace"]] == [s["state"] for s in f["trace"]]
        for sc, sf in zip(c["trace"], f["trace"]):
            assert sc["mask"] == sf["mask"]
            assert np.allclose(sc["angles"], sf["angles"], atol=fresh["tolerance"])


def test_player_traces_visit_every_state():
    expected = json.loads(gen.EXPECTED.read_text())
    states = {s["state"] for case in expected["player"] for s in case["trace"]}
    assert states == {"ENTRY", "PLAYING", "HOLD", "EXIT", "IDLE"}
```

- [ ] **Step 4: Generate and run**

Run: `uv run python gen_animation_fixtures.py`
Expected: `wrote .../animations/fixtures/expected.json`; the file is under 400 KB.

Run: `uv run pytest test_animation_fixtures.py test_animation.py -q`
Expected: all pass.
If `test_player_traces_visit_every_state` fails on a missing state, adjust `steps` in the relevant `PLAYER_CASES` entry (IDLE needs the stop case to run past its exit; HOLD needs `fx_single` to finish its 0.3 s entry), regenerate, re-run.

- [ ] **Step 5: Commit**

```bash
git add animations/fixtures simulation/gen_animation_fixtures.py simulation/test_animation_fixtures.py
git commit -m "✅ Adds the animation parity fixtures and their generator"
```

---

### Task 6: Bundled library and the firmware pack script

**Files:**
- Create: `animations/wave.json`, `crouch.json`, `wiggle.json`, `stretch.json`, `spooked.json`, `play_dead.json`, `body_roll_test.json`
- Create: `firmware/scripts/pack_animations.py`
- Modify: `platformio.ini` (`[env]` `extra_scripts`)
- Modify: `.gitignore`
- Test: `simulation/test_animation_library.py`

**Interfaces:**
- Produces: `firmware/data/animations/<name>.pb` on every firmware build; `pack_animations.pack(src_dir, dst_dir) -> list[Path]`.

- [ ] **Step 1: Write the library**

Conventions: forward is `+y`; positive body `z` crouches; leg 0 is right front, 1 right middle, 2 right rear, 3 left front, 4 left middle, 5 left rear; a raised leg is authored in joint mode because the femur limit caps a foot lift at about 54 mm from the nominal stance.
The values below were checked against the IK on 2026-09-29: every keyframe is inside joint travel at the default parameters (a body yaw of 0.35 rad and a 40 mm crouch combined with a 0.15 rad roll were over, and were reduced).

`animations/wave.json`:

```json
{
  "name": "wave",
  "description": "Lean away from the right front leg, raise it, flick the foot twice",
  "schema": 1,
  "keyframes": [
    {"time": 0},
    {"time": 0.5, "ease": "EASE_IN_OUT", "body": {"x": -25, "y": -15, "z": -5, "pitch": 0.08},
     "legs": [{"joints": {"coxa": 10, "femur": 80, "tibia": -100}}, {"foot": {}}, {"foot": {}}, {"foot": {}}, {"foot": {}}, {"foot": {}}]},
    {"time": 0.8, "ease": "EASE_IN_OUT", "body": {"x": -25, "y": -15, "z": -5, "pitch": 0.08},
     "legs": [{"joints": {"coxa": 10, "femur": 80, "tibia": -60}}, {"foot": {}}, {"foot": {}}, {"foot": {}}, {"foot": {}}, {"foot": {}}]},
    {"time": 1.1, "ease": "EASE_IN_OUT", "body": {"x": -25, "y": -15, "z": -5, "pitch": 0.08},
     "legs": [{"joints": {"coxa": 10, "femur": 80, "tibia": -110}}, {"foot": {}}, {"foot": {}}, {"foot": {}}, {"foot": {}}, {"foot": {}}]},
    {"time": 1.4, "ease": "EASE_IN_OUT", "body": {"x": -25, "y": -15, "z": -5, "pitch": 0.08},
     "legs": [{"joints": {"coxa": 10, "femur": 80, "tibia": -60}}, {"foot": {}}, {"foot": {}}, {"foot": {}}, {"foot": {}}, {"foot": {}}]},
    {"time": 1.7, "ease": "EASE_IN_OUT", "body": {"x": -25, "y": -15, "z": -5, "pitch": 0.08},
     "legs": [{"joints": {"coxa": 10, "femur": 80, "tibia": -100}}, {"foot": {}}, {"foot": {}}, {"foot": {}}, {"foot": {}}, {"foot": {}}]},
    {"time": 2.3, "ease": "EASE_IN_OUT"}
  ],
  "params": [
    {"id": "SPEED", "min": 0.5, "defaultValue": 1, "max": 2},
    {"id": "REPEAT", "min": 1, "defaultValue": 1, "max": 3}
  ]
}
```

`animations/crouch.json`:

```json
{
  "name": "crouch",
  "description": "Squat down and back up",
  "schema": 1,
  "keyframes": [
    {"time": 0},
    {"time": 0.4, "ease": "EASE_IN_OUT", "body": {"z": 30}},
    {"time": 0.8, "ease": "EASE_IN_OUT"}
  ],
  "params": [{"id": "BODY_Z", "min": 0.3, "defaultValue": 1, "max": 1.3}, {"id": "SPEED", "min": 0.5, "defaultValue": 1, "max": 2}]
}
```

`animations/wiggle.json`:

```json
{
  "name": "wiggle",
  "description": "Roll, yaw and bounce on sines until stopped",
  "schema": 1,
  "loop": true,
  "keyframes": [{"time": 0}, {"time": 2.4}],
  "overlays": [
    {"bodyAxis": 0, "amplitude": 0.12, "frequency": 0.8333, "phase": 0, "start": 0, "end": 2.4},
    {"bodyAxis": 2, "amplitude": 0.1, "frequency": 0.8333, "phase": 1.5708, "start": 0, "end": 2.4},
    {"bodyAxis": 5, "amplitude": 8, "frequency": 0.8333, "phase": 1.5708, "start": 0, "end": 2.4}
  ],
  "params": [{"id": "OVERLAY_AMPLITUDE", "min": 0.2, "defaultValue": 1, "max": 1.5}, {"id": "SPEED", "min": 0.5, "defaultValue": 1, "max": 2.5}]
}
```

`animations/stretch.json`:

```json
{
  "name": "stretch",
  "description": "Front legs reach forward with the chest down, then the rear legs reach back",
  "schema": 1,
  "keyframes": [
    {"time": 0},
    {"time": 0.9, "ease": "EASE_IN_OUT", "body": {"pitch": 0.15, "z": 12, "y": -10},
     "legs": [{"foot": {"y": 45}}, {"foot": {}}, {"foot": {}}, {"foot": {"y": 45}}, {"foot": {}}, {"foot": {}}]},
    {"time": 1.6, "ease": "LINEAR", "body": {"pitch": 0.15, "z": 12, "y": -10},
     "legs": [{"foot": {"y": 45}}, {"foot": {}}, {"foot": {}}, {"foot": {"y": 45}}, {"foot": {}}, {"foot": {}}]},
    {"time": 2.4, "ease": "EASE_IN_OUT"},
    {"time": 3.3, "ease": "EASE_IN_OUT", "body": {"pitch": -0.15, "z": 12, "y": 10},
     "legs": [{"foot": {}}, {"foot": {}}, {"foot": {"y": -45}}, {"foot": {}}, {"foot": {}}, {"foot": {"y": -45}}]},
    {"time": 4.0, "ease": "LINEAR", "body": {"pitch": -0.15, "z": 12, "y": 10},
     "legs": [{"foot": {}}, {"foot": {}}, {"foot": {"y": -45}}, {"foot": {}}, {"foot": {}}, {"foot": {"y": -45}}]},
    {"time": 4.8, "ease": "EASE_IN_OUT"}
  ],
  "params": [{"id": "SPEED", "min": 0.5, "defaultValue": 1, "max": 2}, {"id": "BODY_PITCH", "min": 0, "defaultValue": 1, "max": 1.5}]
}
```

`animations/spooked.json`:

```json
{
  "name": "spooked",
  "description": "A startled hop up and back with a tremble, then a slow settle",
  "schema": 1,
  "keyframes": [
    {"time": 0},
    {"time": 0.15, "ease": "EASE_OUT", "body": {"z": -25, "y": -20, "pitch": 0.12}},
    {"time": 0.4, "ease": "LINEAR", "body": {"z": -20, "y": -25, "pitch": 0.1}},
    {"time": 1.6, "ease": "EASE_IN_OUT"}
  ],
  "overlays": [{"bodyAxis": 0, "amplitude": 0.03, "frequency": 12, "phase": 0, "start": 0.15, "end": 0.9}],
  "params": [{"id": "BODY_Z", "min": 0.3, "defaultValue": 1, "max": 1.2}, {"id": "OVERLAY_AMPLITUDE", "min": 0, "defaultValue": 1, "max": 2}]
}
```

`animations/play_dead.json`:

```json
{
  "name": "play_dead",
  "description": "Drop to the belly with all legs folded up in the air, and stay there",
  "schema": 1,
  "holdEnd": true,
  "entryTime": 0.6,
  "exitTime": 1.2,
  "keyframes": [
    {"time": 0},
    {"time": 0.5, "ease": "EASE_IN", "body": {"z": 35, "roll": 0.1}},
    {"time": 0.9, "ease": "EASE_OUT", "body": {"z": 58, "roll": 0},
     "legs": [{"joints": {"coxa": 0, "femur": 85, "tibia": -130}}, {"joints": {"coxa": 0, "femur": 85, "tibia": -130}}, {"joints": {"coxa": 0, "femur": 85, "tibia": -130}},
              {"joints": {"coxa": 0, "femur": 85, "tibia": -130}}, {"joints": {"coxa": 0, "femur": 85, "tibia": -130}}, {"joints": {"coxa": 0, "femur": 85, "tibia": -130}}]}
  ],
  "params": [{"id": "SPEED", "min": 0.5, "defaultValue": 1, "max": 2}]
}
```

`animations/body_roll_test.json`:

```json
{
  "name": "body_roll_test",
  "description": "Sweep roll, pitch and yaw one at a time to their usable limits",
  "schema": 1,
  "keyframes": [
    {"time": 0},
    {"time": 0.8, "ease": "EASE_IN_OUT", "body": {"roll": 0.3}},
    {"time": 2.4, "ease": "EASE_IN_OUT", "body": {"roll": -0.3}},
    {"time": 3.2, "ease": "EASE_IN_OUT"},
    {"time": 4.0, "ease": "EASE_IN_OUT", "body": {"pitch": 0.3}},
    {"time": 5.6, "ease": "EASE_IN_OUT", "body": {"pitch": -0.3}},
    {"time": 6.4, "ease": "EASE_IN_OUT"},
    {"time": 7.2, "ease": "EASE_IN_OUT", "body": {"yaw": 0.28}},
    {"time": 8.8, "ease": "EASE_IN_OUT", "body": {"yaw": -0.28}},
    {"time": 9.6, "ease": "EASE_IN_OUT"}
  ],
  "params": [
    {"id": "BODY_ROLL", "min": 0, "defaultValue": 1, "max": 1.2},
    {"id": "BODY_PITCH", "min": 0, "defaultValue": 1, "max": 1.2},
    {"id": "BODY_YAW", "min": 0, "defaultValue": 1, "max": 1.1},
    {"id": "SPEED", "min": 0.5, "defaultValue": 1, "max": 2}
  ]
}
```

- [ ] **Step 2: Write the library test**

`simulation/test_animation_library.py`:

```python
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
```

Run: `uv run pytest test_animation_library.py -q`
Expected: all pass.
If a keyframe clamps, reduce that keyframe's offsets (the femur is the usual limit: lower a foot `z`, shrink `body.z` when combined with a reach) until the mask is 0.
Do not loosen the assertion.

- [ ] **Step 3: Write the pack script**

`firmware/scripts/pack_animations.py`:

```python
#!/usr/bin/env python3
"""Packs animations/*.json into firmware/data/animations/*.pb for the LittleFS image.

Runs as a PlatformIO pre-script on every build and standalone. Uses the PlatformIO python, which
pre_build.py already provisions with protobuf and grpcio-tools; the generated animation_pb2 module
lands under .pio so nothing is written into the source tree.
"""
import subprocess
import sys
from pathlib import Path


def project_root() -> Path:
    return Path(__file__).resolve().parents[2]


def generated_module(root: Path):
    # Inside the simulation (its tests import this script) the generated module already exists and a
    # second copy would collide in the protobuf descriptor pool, so that one wins when importable.
    try:
        from src.platform_shared import animation_pb2
        return animation_pb2
    except ImportError:
        pass
    out = root / ".pio" / "animation_proto"
    out.mkdir(parents=True, exist_ok=True)
    proto = root / "platform_shared" / "animation.proto"
    stamp = out / "animation_pb2.py"
    if not stamp.exists() or stamp.stat().st_mtime < proto.stat().st_mtime:
        subprocess.run([sys.executable, "-m", "grpc_tools.protoc", f"-I{proto.parent}",
                        f"--python_out={out}", str(proto)], check=True)
    sys.path.insert(0, str(out))
    import animation_pb2  # noqa: E402
    return animation_pb2


def pack(src_dir: Path, dst_dir: Path) -> list[Path]:
    pb = generated_module(project_root())
    from google.protobuf import json_format
    dst_dir.mkdir(parents=True, exist_ok=True)
    for stale in dst_dir.glob("*.pb"):
        stale.unlink()
    written = []
    for src in sorted(src_dir.glob("*.json")):
        msg = json_format.Parse(src.read_text(), pb.Animation())
        if msg.schema != 1 or not msg.keyframes:
            raise SystemExit(f"{src.name}: schema must be 1 and keyframes non-empty")
        if msg.name != src.stem:
            raise SystemExit(f"{src.name}: name '{msg.name}' must equal the file stem")
        dst = dst_dir / f"{src.stem}.pb"
        dst.write_bytes(msg.SerializeToString())
        written.append(dst)
    print(f"animations: packed {len(written)} file(s) -> {dst_dir}")
    return written


def main() -> None:
    root = project_root()
    pack(root / "animations", root / "firmware" / "data" / "animations")


main()
```

The trailing `main()` call rather than an `if __name__` guard is deliberate: PlatformIO executes pre-scripts with `exec`, where `__name__` is not `"__main__"`.

Add to `platformio.ini` under `[env]`:

```ini
extra_scripts =
	pre:firmware/scripts/pre_build.py
	pre:firmware/scripts/pack_animations.py
	pre:firmware/scripts/build_app.py
```

Add to `.gitignore`:

```
# packed from animations/*.json by firmware/scripts/pack_animations.py
firmware/data/animations/
```

- [ ] **Step 4: Write the pack test**

Append to `simulation/test_animation_library.py`:

```python
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
```

Run: `uv run pytest test_animation_library.py -q`
Expected: all pass.

- [ ] **Step 5: Prove the firmware build runs the pack step**

Run from the repo root in PowerShell: `~/.platformio/penv/Scripts/pio run -e esp32-wroom-camera`
Expected: the log contains `animations: packed 7 file(s)` and `firmware/data/animations/` holds seven `.pb` files.
`git status` must not show them.

- [ ] **Step 6: Commit**

```bash
git add animations/*.json firmware/scripts/pack_animations.py platformio.ini .gitignore simulation/test_animation_library.py
git commit -m "✨ Adds the bundled animation library and packs it into the filesystem image"
```

---

### Task 7: Headless feasibility checker

**Files:**
- Create: `simulation/check_animation.py`
- Test: `simulation/test_check_animation.py`
- Modify: `simulation/README.md`, `CLAUDE.md`

**Interfaces:**
- Produces: `check(anim, sim, kin, dt=CONTROL_DT, settle=1.0) -> Report` with fields `name, error, clamped_mask, peak_joint_speed, tilt_max_deg, knock_fraction, fell, steps` and `ok()`; `main(argv)` printing a table and exiting 1 on any error or fall.
  `FALL_TILT_DEG = 45.0` is module level so tests can lower it.

- [ ] **Step 1: Write the failing tests**

`simulation/test_check_animation.py`:

```python
"""check_animation.py runs an animation through the servo model and reports what the preview cannot."""
from pathlib import Path

import numpy as np

import check_animation as ca
from src.robot import animation as an
from src.robot.animation_files import load_json, save_json
from src.robot.firmware_gait import Kinematics
from src.sim.mj_runtime import HexapodSim

LIBRARY = Path(__file__).resolve().parents[1] / "animations"


def hard_roll():
    """A 1.3 rad body roll: the legs clamp and the sim robot tilts about 38 deg (measured 2026-09-29)
    without tipping over. A statically stable hexapod rarely falls, so the verdict plumbing is tested
    by lowering the threshold below that measured tilt."""
    return an.Animation(name="tip", keyframes=[
        an.Keyframe(0.0), an.Keyframe(0.15, body=np.array([1.3, 0, 0, 0, 0, 0])),
        an.Keyframe(2.0, body=np.array([1.3, 0, 0, 0, 0, 0]))])


def test_crouch_is_feasible_with_no_clamp_knock_or_fall():
    r = ca.check(load_json(LIBRARY / "crouch.json"), HexapodSim(), Kinematics())
    assert r.error is None and not r.fell and r.clamped_mask == 0
    assert r.knock_fraction == 0.0
    assert 0.0 < r.peak_joint_speed < ca.SERVO_NOLOAD
    assert r.tilt_max_deg < 5.0


def test_play_dead_touches_the_ground_on_purpose_and_is_not_a_fall():
    r = ca.check(load_json(LIBRARY / "play_dead.json"), HexapodSim(), Kinematics())
    assert r.error is None and not r.fell
    assert r.knock_fraction > 0.2  # the belly rests on the ground during the hold


def test_hard_roll_reports_a_large_tilt_and_clamped_joints_but_no_fall_at_the_default_threshold():
    r = ca.check(hard_roll(), HexapodSim(), Kinematics())
    assert r.tilt_max_deg > 30.0 and r.clamped_mask != 0
    assert not r.fell


def test_fall_verdict_follows_the_tilt_threshold(monkeypatch):
    monkeypatch.setattr(ca, "FALL_TILT_DEG", 30.0)
    r = ca.check(hard_roll(), HexapodSim(), Kinematics())
    assert r.fell and not r.ok()


def test_an_impossible_snap_exceeds_the_servo_speed():
    a = an.Animation(name="snap", entry_time=0.05, keyframes=[
        an.Keyframe(0.0), an.Keyframe(0.02, body=np.array([0, 0, 0, 0, 0, 30.0])), an.Keyframe(0.5)])
    r = ca.check(a, HexapodSim(), Kinematics())
    assert r.peak_joint_speed > ca.SERVO_NOLOAD


def test_an_invalid_animation_reports_the_error_without_simulating():
    a = an.Animation(name="Bad", keyframes=[an.Keyframe(0.0)])
    r = ca.check(a, HexapodSim(), Kinematics())
    assert r.error is not None and r.steps == 0


def test_main_exits_nonzero_on_a_fall_and_prints_the_row(tmp_path, capsys, monkeypatch):
    monkeypatch.setattr(ca, "FALL_TILT_DEG", 30.0)
    save_json(hard_roll(), tmp_path / "tip.json")
    assert ca.main([str(tmp_path / "tip.json")]) == 1
    out = capsys.readouterr().out
    assert "tip" in out and "FELL" in out and "0/1 passed" in out
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `uv run pytest test_check_animation.py -q`
Expected: ModuleNotFoundError for `check_animation`.

- [ ] **Step 3: Write the checker**

`simulation/check_animation.py`:

```python
"""Runs animations through the calibrated servo model and reports what a kinematic preview cannot:
clamped joints, commanded joint speed against the servo's no-load speed, body tilt, how much of the
time something other than a foot touches the ground, and whether the robot tipped over.

    uv run python check_animation.py                  # every file in ../animations
    uv run python check_animation.py ../animations/play_dead.json --dt 0.02

Exits 1 when any animation is invalid or falls, so it can gate CI. A statically stable hexapod
rarely tips; the tilt and knock columns are the numbers to read for everything short of that.
"""
import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from src.robot import animation as an
from src.robot.animation_files import load_json
from src.robot.firmware_gait import Kinematics
from src.sim.mj_runtime import CONTROL_DT, SERVO_NOLOAD, HexapodSim

LIBRARY = Path(__file__).resolve().parents[1] / "animations"
FALL_TILT_DEG = 45.0
LOOP_PLAYS = 2
MAX_SECONDS = 60.0


@dataclass
class Report:
    name: str
    error: str | None = None
    clamped_mask: int = 0
    peak_joint_speed: float = 0.0  # rad/s, commanded
    tilt_max_deg: float = 0.0
    knock_fraction: float = 0.0    # fraction of steps with a non-foot ground contact
    fell: bool = False
    steps: int = 0

    def ok(self) -> bool:
        return self.error is None and not self.fell


def check(anim: an.Animation, sim: HexapodSim, kin: Kinematics, dt: float = CONTROL_DT, settle: float = 1.0) -> Report:
    report = Report(anim.name)
    report.error = an.validate(anim)
    if report.error:
        return report
    sim.reset_to_stand()
    player = an.Player(kin)
    player.play(anim)
    previous = None
    stop_at = anim.duration * LOOP_PLAYS + anim.entry_seconds() if anim.loop or anim.hold_end else None
    settle_left = None
    elapsed = 0.0
    knock_steps = 0
    while elapsed < MAX_SECONDS:
        if stop_at is not None and elapsed >= stop_at and player.state in (an.State.PLAYING, an.State.HOLD):
            player.stop()
        angles_deg, mask = an.pose_to_angles(player.update(dt), kin)
        report.clamped_mask |= mask
        angles = np.radians(angles_deg)
        if previous is not None:
            report.peak_joint_speed = max(report.peak_joint_speed, float(np.max(np.abs(angles - previous)) / dt))
        previous = angles
        sim.set_joint_targets(angles)
        sim.step_physics()
        report.steps += 1
        elapsed += dt
        report.tilt_max_deg = max(report.tilt_max_deg, sim.body_tilt_deg())
        knock_steps += sim.contact_state()[1] > 0.0
        if player.state == an.State.IDLE:
            settle_left = settle if settle_left is None else settle_left - dt
            if settle_left <= 0.0:
                break
    report.knock_fraction = knock_steps / report.steps
    report.fell = report.tilt_max_deg > FALL_TILT_DEG
    return report


def format_row(r: Report) -> str:
    if r.error:
        return f"{r.name:16s} INVALID  {r.error}"
    verdict = "FELL" if r.fell else "ok"
    clamps = "none" if r.clamped_mask == 0 else f"{r.clamped_mask:018b}"
    return (f"{r.name:16s} {verdict:5s} tilt {r.tilt_max_deg:5.1f} deg  peak joint {r.peak_joint_speed:5.1f} rad/s "
            f"(servo {SERVO_NOLOAD:.0f})  knock {r.knock_fraction:4.2f}  clamped {clamps}  {r.steps} steps")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("files", nargs="*", help="animation JSON files (default: every file in the library)")
    ap.add_argument("--dt", type=float, default=CONTROL_DT, help="control step in seconds")
    ap.add_argument("--settle", type=float, default=1.0, help="seconds to keep simulating after the exit")
    args = ap.parse_args(argv)
    files = [Path(f) for f in args.files] or sorted(LIBRARY.glob("*.json"))
    sim, kin = HexapodSim(), Kinematics()
    failed = 0
    for path in files:
        r = check(load_json(path), sim, kin, dt=args.dt, settle=args.settle)
        print(format_row(r))
        failed += 0 if r.ok() else 1
    print(f"{len(files) - failed}/{len(files)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 4: Run the tests and the library**

Run: `uv run pytest test_check_animation.py -q`
Expected: all pass.

Run: `uv run python check_animation.py`
Expected: seven rows, `7/7 passed`.
`play_dead` legitimately reports `knock` well above 0 while it lies on its belly (measured tilt 0 deg at `body.z` 58 on 2026-09-29).
If any animation reports `peak joint` above the servo no-load speed, lengthen the fastest segment, since the robot will lag it.
Keep the printed table for the PR description; commit messages have no body.

- [ ] **Step 5: Document the commands**

Add to `simulation/README.md` (a new section at the end, one sentence per line):

```markdown
## Animations

`animations/*.json` at the repository root are the bundled animations; the schema is `platform_shared/animation.proto`.
`src/robot/animation.py` is the reference evaluator and player that the firmware and app ports mirror.
`uv run python check_animation.py` runs every animation through the servo model and reports clamped joints, peak joint speed, tilt, and falls.
`uv run python gen_animation_fixtures.py` regenerates `animations/fixtures/expected.json`; `uv run pytest` fails while it is stale.
`uv run python sim_sandbox.py` has an Animate mode for playing and scrubbing an animation.
```

Add to the Simulation command block in `CLAUDE.md`:

```sh
uv run python check_animation.py             # run every bundled animation through the servo model
uv run python gen_animation_fixtures.py      # regenerate the animation parity fixtures
```

- [ ] **Step 6: Commit**

```bash
git add simulation/check_animation.py simulation/test_check_animation.py simulation/README.md CLAUDE.md
git commit -m "✨ Adds the headless animation feasibility checker"
```

---

### Task 8: Sandbox Animate mode

**Files:**
- Modify: `simulation/sim_sandbox.py`

**Interfaces:**
- Consumes: `an.Player`, `an.evaluate`, `an.pose_to_angles`, `load_json`.

- [ ] **Step 1: Add the mode**

In `simulation/sim_sandbox.py`:

Imports, after the existing `from src.robot.firmware_gait import (...)` block:

```python
from pathlib import Path
from src.robot import animation as an
from src.robot.animation_files import load_json

ANIMATION_DIR = Path(__file__).resolve().parents[1] / "animations"
```

In `Sandbox.__init__`, after `self.gc = GaitController()`:

```python
        self.player = an.Player(self.kin)
        self.animation = None
        self.animation_params = {}   # slider name -> tk.DoubleVar, rebuilt per animation
```

Add the mode to the radio row in `build_ui`: change `for m in ("stand", "gait", "policy"):` to `for m in ("stand", "gait", "policy", "animate"):`.

Add an Animation frame in `build_ui`, after the "Kinematic jump" frame:

```python
        af = ttk.LabelFrame(panel, text="Animation (Animate mode)")
        af.pack(fill="x", **pad)
        files = sorted(p.stem for p in ANIMATION_DIR.glob("*.json"))
        self.animation_name = tk.StringVar(value=files[0] if files else "")
        row = ttk.Frame(af); row.pack(fill="x", padx=6)
        ttk.Label(row, text="file", width=11).pack(side="left")
        ttk.OptionMenu(row, self.animation_name, self.animation_name.get(), *files,
                       command=lambda _: self._load_animation()).pack(side="left")
        btns = ttk.Frame(af); btns.pack(fill="x", padx=6, pady=2)
        ttk.Button(btns, text="Play", command=self._play_animation).pack(side="left", expand=True, fill="x")
        ttk.Button(btns, text="Stop", command=self.player.stop).pack(side="left", expand=True, fill="x")
        self._slider(af, "scrub", 0.0, 1.0, 0.0, fmt="{:.2f}")   # animation time when idle
        self.param_frame = ttk.Frame(af)
        self.param_frame.pack(fill="x")
        self._load_animation()
```

Add these methods to `Sandbox`, after `do_jump`:

```python
    def _load_animation(self):
        """Load the selected file and rebuild its parameter sliders from the declared params."""
        name = self.animation_name.get()
        if not name:
            return
        self.animation = load_json(ANIMATION_DIR / f"{name}.json")
        err = an.validate(self.animation)
        if err:
            print(f"[animation] {name}: {err}")
            self.animation = None
            return
        for child in self.param_frame.winfo_children():
            child.destroy()
        self.animation_params = {}
        for spec in self.animation.params:
            self._slider(self.param_frame, spec.id.name, spec.min, spec.max, spec.default_value, fmt="{:.2f}")
            self.animation_params[spec.id] = self.vals[spec.id.name]
        self.vals["scrub"].set(0.0)

    def _animation_values(self):
        return {pid: var.get() for pid, var in self.animation_params.items()}

    def _play_animation(self):
        if self.animation is not None:
            self.player.play(self.animation, self._animation_values())  # entry starts from player.last_pose

    def _apply_animation(self):
        """Playing: the player drives the pose. Idle: the scrub slider evaluates the animation directly,
        and the scrubbed pose becomes the player's live pose so a following Play blends from it."""
        if self.player.state != an.State.IDLE:
            pose = self.player.update(CONTROL_DT)
        elif self.animation is not None:
            params = an.resolve_params(self.animation, self._animation_values())
            pose = an.evaluate(self.animation, params, self.v("scrub") * self.animation.duration, self.kin)
            self.player.last_pose = pose
        else:
            pose = an.Pose.stance()
        angles_deg, mask = an.pose_to_angles(pose, self.kin)
        if mask:
            self.status.set(f"animate  clamped {mask:018b}")
        self._animation_angles = np.radians(angles_deg)
```

In `tick`, replace the `else:` branch that handles stand, gait and jump with:

```python
        else:
            if self.jump_frames:                  # scripted kinematic jump in progress
                self._apply_stand(zm_override=self.jump_frames.pop(0))
                targets = self.kin.inverse_kinematics(self.body, degrees=False)
            elif self.mode == "gait":
                self._apply_gait()
                targets = self.kin.inverse_kinematics(self.body, degrees=False)
            elif self.mode == "animate":
                self._apply_animation()
                targets = self._animation_angles
            else:                                 # stand (and post-jump)
                self._apply_stand()
                targets = self.kin.inverse_kinematics(self.body, degrees=False)
            self.sim.set_joint_targets(targets)
            self.sim.step_physics()
```

Update the module docstring's mode list with:

```
  - Animate: play, stop or scrub a bundled animation (animations/*.json) with its parameter sliders.
```

- [ ] **Step 2: Verify by hand**

Run: `uv run python sim_sandbox.py`
Check, in order:
1. Select Animate; the file menu lists seven names and the parameter sliders match the selected file's `params`.
2. Scrub `crouch` from 0 to 1: the body sinks and rises in the viewer.
3. Play `wave`: the robot leans, raises the right front leg and flicks it, then steps home.
   The status bar never shows `clamped`.
4. Play `play_dead`: the robot lies down and stays; Stop brings it back to stance over 1.2 s.
5. Play `wiggle`, then Stop mid-way: it eases home rather than snapping.
6. Switch to Stand and back: nothing is left running.

- [ ] **Step 3: Run the whole suite**

Run from `simulation/`: `uv run pytest -q`
Expected: all pass, including the pre-existing gait parity tests.

- [ ] **Step 4: Commit**

```bash
git add simulation/sim_sandbox.py
git commit -m "✨ Adds an Animate mode to the sim sandbox"
```

---

## Self-review notes

- Spec coverage: schema (Task 1), validation rules (Task 2), evaluator including mixed legs, overlays, multipliers and clamp mask (Task 3), player with entry, exit, hold, loop, repeat, chaining and step arc (Task 4), fixtures with every ease, mixed legs, overlays, each parameter, displaced entry, stop and chain (Task 5), bundled library and pack step (Task 6), checker with clamps, joint speed, tilt and fall (Task 7), sandbox Animate mode (Task 8).
  Firmware and app sections of the spec are plans 2 and 3.
- The Review Focus items are pinned by `test_from_proto_keeps_a_three_leg_keyframe_for_validate_to_reject`, `test_evaluate_clamps_time_to_the_ends`, `test_pose_to_angles_overrides_joint_legs_and_clamps_with_a_mask`, `test_zero_dt_and_a_single_keyframe_reach_hold_or_exit` and `test_play_during_exit_enters_from_the_current_blend_without_a_jump`.
- Not covered on purpose: the firmware `AnimationValidate` sweep of intermediate times is a firmware behaviour; the fixtures' evaluate samples at midpoints and off-grid times give it something to match against.
