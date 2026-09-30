# Animation Firmware Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the ESP32 firmware play animation files: a C++ port of the reference evaluator and player proven against the parity fixtures on the host, an `ANIMATE` motion mode with puppeteering, the wire messages for play, stop, pose and status, chunked file transfer and validation on the request channel, and a Python serial tool to drive it all from a bench before the web app exists.

**Architecture:** `firmware/include/animation/animation.h` is a header-only, nanopb-free port of `simulation/src/robot/animation.py` (clip structs, validator, evaluator, pose-to-angles, player), in the style of `gait.h`.
`animation_codec.h` copies a decoded nanopb `animation_Animation` into a plain `anim::Clip`.
`animation_store.h` reads `/littlefs/animations/<name>.pb` into PSRAM and decodes it.
`animation_runner.h` owns two clip buffers, the player, the puppeteer target and the status publisher, and is driven by `MotionService` from the control task; play, stop and pose requests arrive on the EventBus from the adapters and are consumed on the control task, so no clip is ever read while it is being replaced.
The host `native` env compiles nanopb from the submodule so the parity test decodes the committed `.pb` fixtures exactly as the robot does.

**Tech Stack:** ESP-IDF 5.4 via PlatformIO (pioarduino), nanopb 0.4.9 from `submodules/nanopb`, Unity host tests in `firmware/test/`, Python 3 with protobuf and pyserial under `uv` for the fixture generator and the bench tool.

**Spec:** `docs/superpowers/specs/2026-09-29-animation-system-design.md`, sections 2, 3 and 6.
The reference implementation and its docstrings in `simulation/src/robot/animation.py` are the port source; where the spec text and the reference differ, the reference wins (the spec's section 2 step numbering is known to lag the reference's evaluate order).

This is plan 2 of 3.
Plan 1 (foundation, merged at `9065636`) produced the schema, the reference and the fixtures this plan consumes.
Plan 3 (web app) follows.

## Global Constraints

- All code, comments, docs and commit messages in English and ASCII.
- Commit messages are one line, `[Gitmoji] [Verb] ...`, no body and no trailers of any kind.
- C++ formatting per `firmware/.clang-format` (Google base, 4-space indent, 120 columns, left pointer alignment, include order intentional).
  Comments only for genuine documentation or a non-obvious why.
- No dead code.
  No new feature flag: animation support is always compiled in.
- Firmware builds with `~/.platformio/penv/Scripts/pio run -e esp32-wroom-camera` from the repo root in PowerShell; host tests with `~/.platformio/penv/Scripts/pio test -e native` (MSYS2 `g++` >= 10 on PATH, or `CXX` set).
  Set `PYTHONUTF8=1` when capturing PlatformIO output to a file.
- Python runs through `uv` from `simulation/`.
- Markdown: each sentence on its own line.
- The port mirrors the reference line for line.
  Rules a porter must honour, all stated in the reference's docstrings: file values are float32; the evaluate order is body interpolation, body overlays and per-leg foot overlay accumulation, body multipliers, then legs with mixed legs running IK against the output body with the foot endpoint's overlay and lift applied; an overlay on a joint leg is ignored; joint clamp is symmetric `+-JOINT_LIMIT_DEG` in IK-output space; an unreachable foot sets that leg's femur and tibia mask bits; `REPEAT` is `max(1, floor(x + 0.5))`; entry and exit default to 0.5 s and run on wall time while `SPEED` scales only the playing clock; step arc `45 mm * min(1, travel / 40 mm) * sin(pi * u)` for foot legs whose horizontal travel exceeds 2 mm; `play` records the live pose as `lastPose`; Exit starts from the final keyframe; stance feet are an input (`default_feet_pos`), never a constant.
- Wire tags chosen here and fixed for plan 3: `Message` oneof `animation_play = 280`, `animation_stop = 281`, `pose = 282`, `animation_status = 283`; `CorrelationRequest` `file_write_chunk = 100`, `file_read_chunk = 101`, `file_delete = 102`, `animation_validate = 110`, `animation_list_request = 111`; `CorrelationResponse` `empty = 5`, `file_chunk = 101`, `animation_report = 110`, `animation_list = 111`; `ModesEnum.ANIMATE = 6`.
  File chunks carry at most 512 bytes (BLE MTU sized; the 2048-byte serial frame ceiling then holds with room).
- Precondition for Task 6: the working tree carries another session's uncommitted edits to `firmware/include/motion.h`, `gait.h`, `communication/ble.h`, `peripherals/*`, `wifi/dns_server.h`, `src/communication/ble.cpp` and `test/test_gait/test_gait.cpp`.
  Task 6 edits `motion.h` on top of the working-tree version, so those edits must be committed (or stashed and re-applied) before Task 6 is dispatched, otherwise its commit would sweep them in.
  The controller stops and asks at that point.

## Review Focus

1. A play request naming a file that does not exist or fails validation: the robot must stay in its current mode and pose, log the reason, and report it on the next validate request; it must never enter `ANIMATE` with no clip.
   Test in Task 6 (runner unit test on host).
2. A pose or play request arriving while the control task is mid-tick: no torn read of a clip or pose.
   Task 6 makes every cross-thread hand-off either an atomic flag over a buffer the control task owns, or a copy under a critical section, and its test exercises a play during Playing.
3. A `FileWriteChunk` whose offset does not continue the previous chunk, or whose `offset + len` exceeds `total_size`, or whose path escapes the mount: refused with 400, nothing written.
   Test in Task 7.
4. `AnimationStatus` while playing: emitted at most 5 times per second, and on every state change even inside that window.
   Test in Task 6.
5. A float32 clock: the C++ player accumulates `t` in `float`; the fixtures were built to keep every transition at least 1e-3 s from a step boundary, so the parity trace must match state by state.
   Task 5's test compares all 819 steps, not a sample.

---

## File Structure

| Path | Responsibility |
| --- | --- |
| `platform_shared/message.proto`, `message.options` | animation messages, `ANIMATE` mode, correlation additions |
| `platform_shared/api.proto`, `api.options` | chunked file transfer messages |
| `firmware/include/communication/proto_helpers.h` | traits for the four new `Message` payloads |
| `firmware/include/message_types.h` | `MOTION_STATE::ANIMATE`, `PoseMsg`, `AnimationCommandMsg` |
| `firmware/include/kinematics.h` | `Kinematics::footReachable` |
| `firmware/include/animation/animation.h` | the port: clip structs, validator, evaluator, pose to angles, player (nanopb-free, host-clean) |
| `firmware/include/animation/animation_codec.h` | nanopb struct to `anim::Clip` |
| `firmware/include/animation/animation_store.h` | file to decoded, validated clip in PSRAM; directory listing |
| `firmware/include/animation/animation_runner.h` | play, stop, puppeteer and status, driven from the control task |
| `firmware/include/file_transfer.h` | chunked write and read over stdio (host-clean) |
| `firmware/include/motion.h` | `ANIMATE` branch and mode hand-back |
| `firmware/src/main.cpp` | handlers, correlation cases, status bridge |
| `firmware/test/test_animation/` | host parity and unit tests, plus `nanopb_sources.c` |
| `firmware/test/test_file_transfer/` | host tests for the chunk writer |
| `platformio.ini`, `.github/workflows/embedded-build.yml` | native env gains nanopb and proto generation; CI native job gains submodules and proto tooling |
| `app/scripts/compile_protos.js` | lists `animation.proto` so the app keeps building |
| `simulation/gen_animation_fixtures.py`, `test_animation_fixtures.py` | also emit `expected.txt` and `fx_*.pb` for the C++ test |
| `simulation/scripts/compile_protos.py` | generates all three protos with package-relative imports |
| `simulation/robot_animate.py` | bench tool over the USB serial link |

---

### Task 1: Fixture artifacts for the C++ test

**Files:**
- Modify: `simulation/gen_animation_fixtures.py`
- Modify: `simulation/test_animation_fixtures.py`
- Create (generated, committed): `animations/fixtures/expected.txt`, `animations/fixtures/fx_mixed_legs.pb`, `fx_overlay.pb`, `fx_params.pb`, `fx_single.pb`

**Interfaces:**
- Produces: `animations/fixtures/expected.txt`, a whitespace-token text file the C++ test parses, and one nanopb-decodable `.pb` per fixture animation.

Text format, one record per line, tokens separated by single spaces, `-` for an empty parameter list, parameters as `NAME=VALUE` joined by `;`:

```
T 0.0001
E <animation> <params> <t> <mask> <a0> ... <a17>
P <index> <animation> <params> <dt> <b0> ... <b5> <f0x> <f0y> <f0z> ... <f5z>
V <step> stop
V <step> play <animation> <params>
S <state> <mask> <a0> ... <a17>
```

`E` rows are the evaluate samples in file order.
A `P` row opens a player case with its live pose (six body offsets, then six foot offsets); the `V` rows that follow are its events; the `S` rows that follow are its trace in step order, one per step.
Angles are degrees in IK order, six decimals; `state` is the `State` name.

- [ ] **Step 1: Extend the generator**

In `simulation/gen_animation_fixtures.py` add after `EXPECTED = ...`:

```python
EXPECTED_TXT = FIXTURE_DIR / "expected.txt"
```

Add these functions before `main()`:

```python
def _params_token(values: dict) -> str:
    return "-" if not values else ";".join(f"{k}={v}" for k, v in values.items())


def _floats(xs) -> str:
    return " ".join(f"{float(x):.6f}" for x in xs)


def text_dump(expected: dict) -> str:
    """The same content as expected.json in the line format firmware/test/test_animation parses."""
    lines = [f"T {expected['tolerance']}"]
    for case in expected["evaluate"]:
        for s in case["samples"]:
            lines.append(f"E {case['animation']} {_params_token(case['params'])} {s['t']} {s['mask']} {_floats(s['angles'])}")
    for index, case in enumerate(expected["player"]):
        live = case["live"]
        feet = [v for foot in live["feet"] for v in foot]
        lines.append(f"P {index} {case['animation']} {_params_token(case['params'])} {case['dt']} "
                     f"{_floats(live['body'])} {_floats(feet)}")
        for e in case["events"]:
            if e["action"] == "stop":
                lines.append(f"V {e['step']} stop")
            else:
                lines.append(f"V {e['step']} play {e['animation']} {_params_token(e['params'])}")
        for s in case["trace"]:
            lines.append(f"S {s['state']} {s['mask']} {_floats(s['angles'])}")
    return "\n".join(lines) + "\n"


def write_fixture_binaries() -> None:
    """The C++ parity test decodes these with nanopb, exactly as the robot decodes an upload."""
    from src.robot.animation_files import load_json, save_binary
    for path in sorted(FIXTURE_DIR.glob("fx_*.json")):
        save_binary(load_json(path), path.with_suffix(".pb"))
```

Change `main()` to:

```python
def main() -> None:
    expected = generate()
    EXPECTED.write_text(json.dumps(expected, indent=1) + "\n", newline="\n")
    EXPECTED_TXT.write_text(text_dump(expected), newline="\n")
    write_fixture_binaries()
    print(f"wrote {EXPECTED}, {EXPECTED_TXT} and the fixture .pb files")
```

Extend the generator's docstring with the text format above (one sentence per line, keep the existing contract paragraph).

- [ ] **Step 2: Extend the currency test**

Append to `simulation/test_animation_fixtures.py`:

```python
def test_expected_txt_and_fixture_binaries_are_current(tmp_path):
    from src.robot.animation_files import load_json, to_proto
    assert gen.EXPECTED_TXT.read_bytes() == gen.text_dump(gen.generate()).encode()
    for path in sorted(gen.FIXTURE_DIR.glob("fx_*.json")):
        assert path.with_suffix(".pb").read_bytes() == to_proto(load_json(path)).SerializeToString(), path.name


def test_expected_txt_covers_every_sample_and_step():
    expected = json.loads(gen.EXPECTED.read_text())
    lines = gen.EXPECTED_TXT.read_text().splitlines()
    assert sum(1 for l in lines if l.startswith("E ")) == sum(len(c["samples"]) for c in expected["evaluate"])
    assert sum(1 for l in lines if l.startswith("S ")) == sum(len(c["trace"]) for c in expected["player"])
    assert sum(1 for l in lines if l.startswith("P ")) == len(expected["player"])
```

- [ ] **Step 3: Generate and test**

Run from `simulation/`: `uv run python gen_animation_fixtures.py` then `uv run pytest test_animation_fixtures.py -q`.
Expected: all pass; `expected.json` unchanged (`git status` shows it unmodified); `expected.txt` exists with 819 `S` rows and 7 `P` rows; four `.pb` files of roughly 0.3 to 1 KB each.

- [ ] **Step 4: Commit**

```bash
git add simulation/gen_animation_fixtures.py simulation/test_animation_fixtures.py animations/fixtures/expected.txt animations/fixtures/fx_mixed_legs.pb animations/fixtures/fx_overlay.pb animations/fixtures/fx_params.pb animations/fixtures/fx_single.pb
git commit -m "✅ Emits the animation fixtures in the form the firmware test reads"
```

---

### Task 2: Protocol additions and host test scaffolding

**Files:**
- Modify: `platform_shared/message.proto`, `platform_shared/message.options`, `platform_shared/api.proto`, `platform_shared/api.options`
- Modify: `firmware/include/communication/proto_helpers.h`
- Modify: `firmware/include/message_types.h` (`ANIMATE` only)
- Modify: `firmware/src/main.cpp` (mode guard only)
- Modify: `app/scripts/compile_protos.js`
- Modify: `platformio.ini` (`[env:native]`)
- Modify: `.github/workflows/embedded-build.yml`
- Modify: `simulation/scripts/compile_protos.py`
- Create: `firmware/test/test_animation/nanopb_sources.c`, `firmware/test/test_animation/test_animation.cpp` (decode smoke test only; later tasks append)

**Interfaces:**
- Produces: generated `socket_message_AnimationPlay`, `_AnimationStop`, `_PoseData`, `_AnimationStatus`, `_AnimationParam`, `_AnimationValidate`, `_AnimationReport`, `_AnimationList`, `_AnimationEntry`, `socket_message_AnimationState_ANIM_*`, `socket_message_ModesEnum_ANIMATE`, `api_FileWriteChunk`, `api_FileReadChunk`, `api_FileChunk`, and the correlation tags listed in Global Constraints; `MessageTraits` for the four `Message` payloads; a host test env that decodes `.pb` fixtures.

- [ ] **Step 1: Protos**

`platform_shared/message.proto`: add `import "animation.proto";` after the `api.proto` import; append `ANIMATE = 6;` to `ModesEnum` with the comment `// plays animation files; see animation.proto`; add after `SystemCommandData`:

```proto
// --- Animations (animation.proto holds the file schema) ---

message AnimationParam {
  animation.ParamId id = 1;
  float value = 2;
}
// Loads /littlefs/animations/<name>.pb and plays it. From any active mode the robot borrows ANIMATE
// and hands the mode back when the player finishes; a play while playing chains.
message AnimationPlay {
  string name = 1;
  repeated AnimationParam params = 2;
}
message AnimationStop {}
// Puppeteer target, applied only in ANIMATE while nothing is playing; held if the stream stops.
message PoseData {
  animation.BodyPose body = 1;
  repeated animation.LegTarget legs = 2;  // 0 or 6
}
enum AnimationState {
  ANIM_IDLE = 0;
  ANIM_ENTRY = 1;
  ANIM_PLAYING = 2;
  ANIM_HOLD = 3;
  ANIM_EXIT = 4;
}
// Observable: pushed on every state change and at 5 Hz while the player is not idle.
message AnimationStatus {
  string name = 1;
  AnimationState state = 2;
  float t = 3;
  uint32 clamped_mask = 4;  // bit leg*3+joint for joints pinned at a limit or an unreachable foot
}
message AnimationValidate { string name = 1; }
message AnimationReport {
  bool ok = 1;
  string error = 2;          // first structural error, empty when ok
  uint32 clamped_mask = 3;   // union over every keyframe and 32 intermediate times per segment
}
message AnimationListRequest {}
message AnimationEntry {
  string name = 1;
  uint32 size = 2;
}
message AnimationList { repeated AnimationEntry entries = 1; }
```

Add to the `CorrelationRequest` oneof:

```proto
    api.FileWriteChunk file_write_chunk = 100;
    api.FileReadChunk file_read_chunk = 101;
    api.FileDeleteRequest file_delete = 102;
    AnimationValidate animation_validate = 110;
    AnimationListRequest animation_list_request = 111;
```

Add to the `CorrelationResponse` oneof:

```proto
    api.EmptyMessage empty = 5;
    api.FileChunk file_chunk = 101;
    AnimationReport animation_report = 110;
    AnimationList animation_list = 111;
```

Add to the `Message` oneof:

```proto
    AnimationPlay animation_play = 280;
    AnimationStop animation_stop = 281;
    PoseData pose = 282;
    AnimationStatus animation_status = 283;  // observable status: pushed while an animation runs
```

`platform_shared/api.proto`, in the File System section after `FileMkdirRequest`:

```proto
// Chunked transfer over the correlation channel, for transports without HTTP. Offset 0 truncates;
// the chunk whose end equals total_size completes the file. At most 512 bytes per chunk.
message FileWriteChunk {
    string path = 1;
    uint32 offset = 2;
    uint32 total_size = 3;
    bytes content = 4;
}
message FileReadChunk {
    string path = 1;
    uint32 offset = 2;
    uint32 length = 3;  // at most 512
}
message FileChunk {
    bytes content = 1;
    uint32 total_size = 2;
}
```

`platform_shared/message.options`, append:

```
socket_message.AnimationPlay.name max_size:33
socket_message.AnimationPlay.params max_count:10
socket_message.PoseData.legs max_count:6
socket_message.AnimationStatus.name max_size:33
socket_message.AnimationValidate.name max_size:33
socket_message.AnimationReport.error max_size:96
socket_message.AnimationList.entries max_count:32
socket_message.AnimationEntry.name max_size:33
```

`platform_shared/api.options`, append:

```
api.FileWriteChunk.path max_size:128
api.FileWriteChunk.content max_size:512
api.FileReadChunk.path max_size:128
api.FileChunk.content max_size:512
```

- [ ] **Step 2: Traits, mode enum, guard, app file list**

`firmware/include/communication/proto_helpers.h`, add to the macro block:

```cpp
DEFINE_MESSAGE_TRAITS(AnimationPlay, animation_play)
DEFINE_MESSAGE_TRAITS(AnimationStop, animation_stop)
DEFINE_MESSAGE_TRAITS(PoseData, pose)
DEFINE_MESSAGE_TRAITS(AnimationStatus, animation_status)
```

`firmware/include/message_types.h`: `enum class MOTION_STATE { DEACTIVATED, IDLE, POSE, STAND, WALK, WALK_NN, ANIMATE };`

`firmware/src/main.cpp`, the `ModeData` handler: change the upper bound to `socket_message_ModesEnum_ANIMATE`.

`app/scripts/compile_protos.js`: `const protoFiles = ['message.proto', 'api.proto', 'animation.proto']`.

`simulation/scripts/compile_protos.py`: generate all three files and rewrite the absolute imports protoc emits so the package stays importable under `src.platform_shared`:

```python
PROTO_FILES = ["animation.proto", "api.proto", "message.proto"]
IMPORT = re.compile(r"^import (\w+_pb2) as (\w+)$", re.MULTILINE)


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    (OUT_DIR / "__init__.py").touch()
    cmd = [sys.executable, "-m", "grpc_tools.protoc", f"-I{PROTO_DIR}", f"--python_out={OUT_DIR}"]
    cmd += [str(PROTO_DIR / f) for f in PROTO_FILES]
    subprocess.run(cmd, check=True)
    for f in PROTO_FILES:
        module = OUT_DIR / f.replace(".proto", "_pb2.py")
        module.write_text(IMPORT.sub(r"from . import \1 as \2", module.read_text()))
    print(f"protoc: {', '.join(PROTO_FILES)} -> {OUT_DIR}")
```

(add `import re`; update the module docstring: all three schemas are generated and the imports are made package-relative).

- [ ] **Step 3: Native env and CI**

`platformio.ini` `[env:native]`:

```ini
[env:native]
platform = native
framework =
test_framework = unity
build_flags =
	-std=gnu++2a
	-Wall
	-I firmware/include
	-I firmware/src/platform_shared
	-I submodules/nanopb
build_unflags =
extra_scripts =
	pre:firmware/scripts/pre_build.py
```

Update the env comment: the animation tests decode nanopb fixtures, so the env compiles nanopb from the submodule and regenerates the protos first (needs `python` with `grpcio-tools`, as `pio run` does).

`.github/workflows/embedded-build.yml`, `native-tests` job: give the checkout `with: submodules: "recursive"` and change the install step to `pip install platformio==6.1.19 protobuf grpcio-tools`.

- [ ] **Step 4: Host test scaffolding**

`firmware/test/test_animation/nanopb_sources.c`:

```c
// The native env compiles only the test directory, so nanopb and the generated animation schema
// are pulled in here as C rather than listed as sources.
#include "pb_common.c"
#include "pb_decode.c"
#include "pb_encode.c"
#include "animation.pb.c"
```

`firmware/test/test_animation/test_animation.cpp` (initial content; later tasks add tests and `RUN_TEST` lines):

```cpp
// Host-side tests for the animation port. The parity fixtures under animations/fixtures/ are
// generated from the Python reference; matching them is what makes the robot play what the editor
// and the simulation showed.

#include <unity.h>

#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <pb_decode.h>
#include <animation.pb.h>

namespace {

constexpr const char *FIXTURE_DIR = "animations/fixtures/";

std::vector<uint8_t> readFile(const std::string &path) {
    std::ifstream in(path, std::ios::binary);
    TEST_ASSERT_TRUE_MESSAGE(in.good(), path.c_str());
    return std::vector<uint8_t>((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

bool decodeFixture(const char *name, animation_Animation &out) {
    const std::vector<uint8_t> bytes = readFile(std::string(FIXTURE_DIR) + name + ".pb");
    pb_istream_t stream = pb_istream_from_buffer(bytes.data(), bytes.size());
    out = animation_Animation_init_zero;
    return pb_decode(&stream, animation_Animation_fields, &out);
}

}  // namespace

void setUp() {}
void tearDown() {}

void test_fixture_binaries_decode_with_nanopb() {
    animation_Animation a;
    TEST_ASSERT_TRUE(decodeFixture("fx_single", a));
    TEST_ASSERT_EQUAL_STRING("fx_single", a.name);
    TEST_ASSERT_EQUAL_UINT32(1, a.schema);
    TEST_ASSERT_TRUE(a.hold_end);
    TEST_ASSERT_EQUAL(1, a.keyframes_count);
    TEST_ASSERT_EQUAL(6, a.keyframes[0].legs_count);
    TEST_ASSERT_EQUAL(animation_LegTarget_joints_tag, a.keyframes[0].legs[3].which_target);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 85.0f, a.keyframes[0].legs[3].target.joints.femur);
    TEST_ASSERT_TRUE(decodeFixture("fx_mixed_legs", a));
    TEST_ASSERT_EQUAL(5, a.keyframes_count);
    TEST_ASSERT_EQUAL(2, a.params_count);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_fixture_binaries_decode_with_nanopb);
    return UNITY_END();
}
```

- [ ] **Step 5: Build everything**

Run from the repo root in PowerShell:
- `~/.platformio/penv/Scripts/pio run -e esp32-wroom-camera`: expected SUCCESS (the new messages compile into the firmware; note the reported RAM and flash deltas).
- `~/.platformio/penv/Scripts/pio test -e native`: expected the gait tests and `test_fixture_binaries_decode_with_nanopb` pass.
  The test binary's working directory is the project root, which is what `FIXTURE_DIR` assumes; if the fixture open fails, print `getcwd` in the failure message and adjust the constant, do not hard-code an absolute path.
- From `app/`: `pnpm proto` generates `animation.ts`; `pnpm check` passes.
- From `simulation/`: `uv run python scripts/compile_protos.py` then `uv run pytest -q`: all pass (the animation module now sits beside `api_pb2` and `message_pb2` with relative imports; `animation_files` still imports it the same way).

- [ ] **Step 6: Commit**

```bash
git add platform_shared/message.proto platform_shared/message.options platform_shared/api.proto platform_shared/api.options firmware/include/communication/proto_helpers.h firmware/include/message_types.h firmware/src/main.cpp app/scripts/compile_protos.js platformio.ini .github/workflows/embedded-build.yml simulation/scripts/compile_protos.py firmware/test/test_animation/nanopb_sources.c firmware/test/test_animation/test_animation.cpp
git commit -m "✨ Adds the animation wire messages and a host env that decodes the fixtures"
```

---

### Task 3: Clip structs, codec and validator

**Files:**
- Create: `firmware/include/animation/animation.h` (part 1: constants, structs, `validate`)
- Create: `firmware/include/animation/animation_codec.h`
- Test: `firmware/test/test_animation/test_animation.cpp`

**Interfaces:**
- Produces in namespace `anim`: constants `KEYFRAME_MAX 32`, `OVERLAY_MAX 8`, `PARAM_MAX 10`, `NAME_MAX 32`, `DESCRIPTION_MAX 96`, `SCHEMA_VERSION 1`, `DEFAULT_ENTRY_S`, `DEFAULT_EXIT_S`, `STEP_ARC_MM`, `STEP_ARC_FULL_TRAVEL_MM`, `STEP_ARC_MIN_TRAVEL_MM`; enums `Ease`, `ParamId` (with `PARAM_COUNT`), `BodyAxis`; `BODY_PARAM_FOR_AXIS[6]`; structs `LegTarget{bool joints; float v[3]}`, `Keyframe`, `Overlay`, `ParamSpec`, `ParamValue{int id; float value}`, `Clip` with `duration()`, `entrySeconds()`, `exitSeconds()`; `legTarget(const Keyframe&, int)`; `const char *validate(const Clip&)`.
- Produces `anim::fromProto(const animation_Animation&, Clip&)`.

- [ ] **Step 1: Write the failing tests**

Append to `test_animation.cpp` (add `#include <animation/animation.h>` and `#include <animation/animation_codec.h>` after the nanopb includes; add the `RUN_TEST` lines to `main`):

```cpp
void test_from_proto_copies_every_fixture_and_validates() {
    const char *names[] = {"fx_mixed_legs", "fx_overlay", "fx_params", "fx_single"};
    for (const char *name : names) {
        animation_Animation a;
        TEST_ASSERT_TRUE(decodeFixture(name, a));
        anim::Clip clip;
        anim::fromProto(a, clip);
        TEST_ASSERT_NULL_MESSAGE(anim::validate(clip), name);
        TEST_ASSERT_EQUAL_STRING(name, clip.name);
    }
    animation_Animation a;
    decodeFixture("fx_mixed_legs", a);
    anim::Clip clip;
    anim::fromProto(a, clip);
    TEST_ASSERT_EQUAL(5, clip.keyframeCount);
    TEST_ASSERT_TRUE(clip.keyframes[1].legs[0].joints);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 75.0f, clip.keyframes[1].legs[0].v[1]);
    TEST_ASSERT_FALSE(clip.keyframes[1].legs[3].joints);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 25.0f, clip.keyframes[1].legs[3].v[2]);
    TEST_ASSERT_EQUAL(anim::EASE_IN, clip.keyframes[1].ease);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.37f, clip.entryTime);
    TEST_ASSERT_EQUAL(0, clip.keyframes[4].legCount);
    TEST_ASSERT_EQUAL(anim::FOOT_LIFT, clip.params[0].id);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.0f, clip.params[0].max);
    decodeFixture("fx_overlay", a);
    anim::fromProto(a, clip);
    TEST_ASSERT_TRUE(clip.loop);
    TEST_ASSERT_TRUE(clip.overlays[0].onBody);
    TEST_ASSERT_EQUAL(0, clip.overlays[0].channel);
    TEST_ASSERT_FALSE(clip.overlays[2].onBody);
    TEST_ASSERT_EQUAL(5, clip.overlays[2].channel);
}

anim::Clip twoKeyframes() {
    anim::Clip c;
    strcpy(c.name, "t");
    c.keyframeCount = 2;
    c.keyframes[0].time = 0.0f;
    c.keyframes[1].time = 1.0f;
    return c;
}

void expectError(anim::Clip &c, const char *fragment) {
    const char *err = anim::validate(c);
    TEST_ASSERT_NOT_NULL_MESSAGE(err, fragment);
    TEST_ASSERT_NOT_NULL_MESSAGE(strstr(err, fragment), err);
}

void test_validate_reports_each_structural_rule() {
    anim::Clip c = twoKeyframes();
    TEST_ASSERT_NULL(anim::validate(c));
    c = twoKeyframes(); c.schema = 2; expectError(c, "schema");
    c = twoKeyframes(); strcpy(c.name, "Bad Name"); expectError(c, "name");
    c = twoKeyframes(); memset(c.description, 'd', 97); c.description[97] = 0; expectError(c, "description");
    c = twoKeyframes(); c.keyframeCount = 0; expectError(c, "keyframe");
    c = twoKeyframes(); c.keyframes[0].time = 0.1f; expectError(c, "time 0");
    c = twoKeyframes(); c.keyframes[1].time = 0.0f; expectError(c, "increase");
    c = twoKeyframes(); c.keyframes[1].time = NAN; expectError(c, "finite");
    c = twoKeyframes(); c.keyframes[1].legCount = 3; expectError(c, "0 or 6");
    c = twoKeyframes(); c.keyframes[1].ease = 4; expectError(c, "ease");
    c = twoKeyframes(); c.loop = true; c.holdEnd = true; expectError(c, "loop");
    c = twoKeyframes(); c.entryTime = INFINITY; expectError(c, "finite");
    c = twoKeyframes(); c.overlayCount = 1; c.overlays[0] = {true, 6, 1, 1, 0, 0, 1}; expectError(c, "body_axis");
    c = twoKeyframes(); c.overlayCount = 1; c.overlays[0] = {false, 18, 1, 1, 0, 0, 1}; expectError(c, "foot_channel");
    c = twoKeyframes(); c.overlayCount = 1; c.overlays[0] = {true, 0, 1, 1, 0, 0.5f, 0.5f}; expectError(c, "start");
    c = twoKeyframes(); c.overlayCount = 1; c.overlays[0] = {true, 0, 1, 1, 0, 0, 1.5f}; expectError(c, "end");
    c = twoKeyframes(); c.overlayCount = 1; c.overlays[0] = {true, 0, NAN, 1, 0, 0, 1}; expectError(c, "finite");
    c = twoKeyframes(); c.paramCount = 2; c.params[0] = {anim::SPEED, 0.5f, 1, 2}; c.params[1] = {anim::SPEED, 0.5f, 1, 2}; expectError(c, "unique");
    c = twoKeyframes(); c.paramCount = 1; c.params[0] = {anim::BODY_Z, 0.5f, 3, 2}; expectError(c, "min <= default_value <= max");
    c = twoKeyframes(); c.paramCount = 1; c.params[0] = {anim::SPEED, 0.0f, 1, 2}; expectError(c, "SPEED");
    c = twoKeyframes(); c.paramCount = 1; c.params[0] = {anim::REPEAT, 0.0f, 1, 2}; expectError(c, "REPEAT");
    c = twoKeyframes(); c.paramCount = 1; c.params[0] = {10, 0.5f, 1, 2}; expectError(c, "param id");
    c = twoKeyframes(); c.keyframeCount = 33; expectError(c, "32");
    c = twoKeyframes(); c.overlayCount = 9; expectError(c, "8");
    c = twoKeyframes(); c.paramCount = 11; expectError(c, "10");
}
```

- [ ] **Step 2: Run to verify failure**

`pio test -e native` fails to compile on the missing headers.

- [ ] **Step 3: Write the header, part 1**

`firmware/include/animation/animation.h`:

```cpp
#pragma once

// Port of simulation/src/robot/animation.py: the animation clip model, its validator, the evaluator,
// the pose-to-servo-angle path and the player. The Python module is the reference and carries the
// rules in its docstrings; this file mirrors it line for line and is checked against the fixtures in
// animations/fixtures/ by firmware/test/test_animation. It knows nothing about nanopb or the ESP: the
// codec fills a Clip from the decoded file, and the runner drives the player from the control task.
//
// Units: body angles rad, lengths mm, joint angles deg in the IK output convention, time seconds.
// File values are 32-bit floats and so is every value here.

#include <cmath>
#include <cstdint>
#include <cstring>
#include <kinematics.h>
#include <message_types.h>
#include <utils/math_utils.h>

namespace anim {

constexpr int KEYFRAME_MAX = 32;
constexpr int OVERLAY_MAX = 8;
constexpr int PARAM_MAX = 10;
constexpr int NAME_MAX = 32;
constexpr int DESCRIPTION_MAX = 96;
constexpr uint32_t SCHEMA_VERSION = 1;
constexpr float DEFAULT_ENTRY_S = 0.5f;
constexpr float DEFAULT_EXIT_S = 0.5f;
constexpr float STEP_ARC_MM = 45.0f;
constexpr float STEP_ARC_FULL_TRAVEL_MM = 40.0f;
constexpr float STEP_ARC_MIN_TRAVEL_MM = 2.0f;

enum Ease : int { LINEAR = 0, EASE_IN = 1, EASE_OUT = 2, EASE_IN_OUT = 3 };

enum ParamId : int {
    SPEED = 0,
    BODY_X = 1,
    BODY_Y = 2,
    BODY_Z = 3,
    BODY_ROLL = 4,
    BODY_PITCH = 5,
    BODY_YAW = 6,
    FOOT_LIFT = 7,
    OVERLAY_AMPLITUDE = 8,
    REPEAT = 9,
    PARAM_COUNT = 10
};

enum BodyAxis : int { ROLL = 0, PITCH = 1, YAW = 2, X = 3, Y = 4, Z = 5 };

constexpr ParamId BODY_PARAM_FOR_AXIS[6] = {BODY_ROLL, BODY_PITCH, BODY_YAW, BODY_X, BODY_Y, BODY_Z};

// A leg is either a foot offset from the standing foot (mm, +z lifts) or three joint angles (deg).
struct LegTarget {
    bool joints = false;
    float v[3] = {0.0f, 0.0f, 0.0f};
};

struct Keyframe {
    float time = 0.0f;
    int ease = LINEAR;
    float body[6] = {0, 0, 0, 0, 0, 0};  // roll, pitch, yaw, x, y, z offsets
    int legCount = 0;                    // 0 = every foot holds stance, else 6
    LegTarget legs[6];
};

struct Overlay {
    bool onBody = true;  // channel is a BodyAxis, else leg*3 + axis
    int channel = 0;
    float amplitude = 0.0f;
    float frequency = 1.0f;
    float phase = 0.0f;
    float start = 0.0f;
    float end = 0.0f;
};

struct ParamSpec {
    int id = 0;
    float min = 0.0f;
    float defaultValue = 1.0f;
    float max = 1.0f;
};

struct ParamValue {
    int id;
    float value;
};

struct Clip {
    char name[NAME_MAX + 1] = {0};
    char description[DESCRIPTION_MAX + 1] = {0};
    uint32_t schema = SCHEMA_VERSION;
    bool loop = false;
    bool holdEnd = false;
    float entryTime = 0.0f;
    float exitTime = 0.0f;
    int keyframeCount = 0;
    Keyframe keyframes[KEYFRAME_MAX];
    int overlayCount = 0;
    Overlay overlays[OVERLAY_MAX];
    int paramCount = 0;
    ParamSpec params[PARAM_MAX];

    float duration() const { return keyframeCount > 0 ? keyframes[keyframeCount - 1].time : 0.0f; }
    float entrySeconds() const { return entryTime > 0.0f ? entryTime : DEFAULT_ENTRY_S; }
    float exitSeconds() const { return exitTime > 0.0f ? exitTime : DEFAULT_EXIT_S; }
};

inline LegTarget legTarget(const Keyframe &k, int leg) { return k.legCount > 0 ? k.legs[leg] : LegTarget{}; }

inline bool validName(const char *name) {
    const size_t n = strlen(name);
    if (n < 1 || n > NAME_MAX) return false;
    for (size_t i = 0; i < n; ++i) {
        const char c = name[i];
        const bool ok = (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_' || c == '-';
        if (!ok) return false;
    }
    return true;
}

// The first structural error as a static string, or nullptr. Same rules and order as the reference's
// validate(); the description limit is bytes because the file buffer is bytes.
inline const char *validate(const Clip &c) {
    if (c.schema != SCHEMA_VERSION) return "schema is not 1";
    if (!validName(c.name)) return "name must be 1-32 characters of [a-z0-9_-]";
    if (strlen(c.description) > (size_t)DESCRIPTION_MAX) return "description longer than 96 bytes";
    if (!std::isfinite(c.entryTime) || !std::isfinite(c.exitTime)) return "entry/exit time must be finite";
    if (c.loop && c.holdEnd) return "loop and hold_end may not both be set";
    if (c.keyframeCount < 1) return "at least one keyframe is required";
    if (c.keyframeCount > KEYFRAME_MAX) return "more than 32 keyframes";
    for (int i = 0; i < c.keyframeCount; ++i) {
        const Keyframe &k = c.keyframes[i];
        if (!std::isfinite(k.time)) return "keyframe time must be finite";
        for (float v : k.body)
            if (!std::isfinite(v)) return "keyframe body must be finite";
        for (int l = 0; l < k.legCount; ++l)
            for (float v : k.legs[l].v)
                if (!std::isfinite(v)) return "keyframe leg must be finite";
        if (k.ease < LINEAR || k.ease > EASE_IN_OUT) return "keyframe ease out of range";
        if (k.legCount != 0 && k.legCount != 6) return "keyframe must have 0 or 6 legs";
    }
    if (c.keyframes[0].time != 0.0f) return "first keyframe must be at time 0";
    for (int i = 1; i < c.keyframeCount; ++i)
        if (c.keyframes[i].time <= c.keyframes[i - 1].time) return "keyframe time must increase";
    if (c.overlayCount > OVERLAY_MAX) return "more than 8 overlays";
    for (int i = 0; i < c.overlayCount; ++i) {
        const Overlay &o = c.overlays[i];
        if (!std::isfinite(o.amplitude) || !std::isfinite(o.frequency) || !std::isfinite(o.phase) ||
            !std::isfinite(o.start) || !std::isfinite(o.end))
            return "overlay must be finite";
        if (o.onBody && (o.channel < 0 || o.channel > 5)) return "overlay body_axis out of range";
        if (!o.onBody && (o.channel < 0 || o.channel > 17)) return "overlay foot_channel out of range";
        if (o.start < 0.0f || o.start >= o.end) return "overlay window must have 0 <= start < end";
        if (o.end > c.duration()) return "overlay end is after the last keyframe";
    }
    if (c.paramCount > PARAM_MAX) return "more than 10 params";
    for (int i = 0; i < c.paramCount; ++i) {
        const ParamSpec &p = c.params[i];
        if (p.id < 0 || p.id >= PARAM_COUNT) return "param id out of range";
        for (int j = 0; j < i; ++j)
            if (c.params[j].id == p.id) return "param id is not unique";
        if (!std::isfinite(p.min) || !std::isfinite(p.defaultValue) || !std::isfinite(p.max))
            return "param must be finite";
        if (!(p.min <= p.defaultValue && p.defaultValue <= p.max)) return "param needs min <= default_value <= max";
        if (p.id == SPEED && p.min <= 0.0f) return "param SPEED needs a positive min";
        if (p.id == REPEAT && p.min < 1.0f) return "param REPEAT needs min >= 1";
    }
    return nullptr;
}

}  // namespace anim
```

Check the reference's `validate` for the exact rule order and messages before writing; the strings above must contain the fragments the test looks for and should read like the Python ones.

`firmware/include/animation/animation_codec.h`:

```cpp
#pragma once

// Copies a decoded animation.proto message into the plain Clip the evaluator reads. Enum values are
// kept as integers so validate() reports an unknown ease or param id instead of the decoder.

#include <animation/animation.h>
#include <platform_shared/animation.pb.h>

namespace anim {

inline void fromProto(const animation_Animation &m, Clip &c) {
    c = Clip{};
    strncpy(c.name, m.name, NAME_MAX);
    strncpy(c.description, m.description, DESCRIPTION_MAX);
    c.schema = m.schema;
    c.loop = m.loop;
    c.holdEnd = m.hold_end;
    c.entryTime = m.entry_time;
    c.exitTime = m.exit_time;
    c.keyframeCount = m.keyframes_count;
    for (int i = 0; i < c.keyframeCount && i < KEYFRAME_MAX; ++i) {
        const animation_Keyframe &src = m.keyframes[i];
        Keyframe &k = c.keyframes[i];
        k.time = src.time;
        k.ease = (int)src.ease;
        k.body[ROLL] = src.body.roll;
        k.body[PITCH] = src.body.pitch;
        k.body[YAW] = src.body.yaw;
        k.body[X] = src.body.x;
        k.body[Y] = src.body.y;
        k.body[Z] = src.body.z;
        k.legCount = src.legs_count;
        for (int l = 0; l < k.legCount && l < 6; ++l) {
            const animation_LegTarget &lt = src.legs[l];
            LegTarget &t = k.legs[l];
            t.joints = lt.which_target == animation_LegTarget_joints_tag;
            if (t.joints) {
                t.v[0] = lt.target.joints.coxa;
                t.v[1] = lt.target.joints.femur;
                t.v[2] = lt.target.joints.tibia;
            } else {
                t.v[0] = lt.target.foot.x;
                t.v[1] = lt.target.foot.y;
                t.v[2] = lt.target.foot.z;
            }
        }
    }
    c.overlayCount = m.overlays_count;
    for (int i = 0; i < c.overlayCount && i < OVERLAY_MAX; ++i) {
        const animation_Overlay &src = m.overlays[i];
        Overlay &o = c.overlays[i];
        o.onBody = src.which_channel != animation_Overlay_foot_channel_tag;
        o.channel = o.onBody ? (int)src.channel.body_axis : (int)src.channel.foot_channel;
        o.amplitude = src.amplitude;
        o.frequency = src.frequency;
        o.phase = src.phase;
        o.start = src.start;
        o.end = src.end;
    }
    c.paramCount = m.params_count;
    for (int i = 0; i < c.paramCount && i < PARAM_MAX; ++i) {
        c.params[i] = {(int)m.params[i].id, m.params[i].min, m.params[i].default_value, m.params[i].max};
    }
}

}  // namespace anim
```

An absent `body` decodes as zeros in nanopb (`has_body` false, the submessage zero-initialised), which is the zero offset the schema promises; a `LegTarget` with no oneof set decodes as a zero foot offset.

- [ ] **Step 4: Run the tests**

`pio test -e native`: expected the three animation tests pass.
If a `validate` message fragment differs from the test's expectation, fix the message, not the test, keeping the Python wording.

- [ ] **Step 5: Commit**

```bash
git add firmware/include/animation/animation.h firmware/include/animation/animation_codec.h firmware/test/test_animation/test_animation.cpp
git commit -m "✨ Ports the animation clip model, codec and validator to the firmware"
```

---

### Task 4: Evaluator and pose to angles, with reach detection

**Files:**
- Modify: `firmware/include/kinematics.h` (add `footReachable`)
- Modify: `firmware/include/animation/animation.h` (part 2)
- Test: `firmware/test/test_animation/test_animation.cpp`

**Interfaces:**
- Produces: `Kinematics::footReachable(const BodyStateMsg&, int leg) const`; in `anim`: `struct Pose{float body[6]; LegTarget legs[6]}`, `easeValue(int, float)`, `resolveParams(const Clip&, const ParamValue*, int, float out[PARAM_COUNT])`, `bodyState(const float body6[6], const float stance[6][4], BodyStateMsg&)`, `legJointsDeg(Kinematics&, const float body6[6], const float foot[3], int leg, const float stance[6][4], float out[3])`, `evaluate(const Clip&, const float params[PARAM_COUNT], float t, Kinematics&, const float stance[6][4], Pose&)`, `uint32_t poseToAngles(const Pose&, Kinematics&, const float stance[6][4], float angles[18])`, `capturePose(const BodyStateMsg&, const float stance[6][4], Pose&)`.

- [ ] **Step 1: Write the failing tests**

Append to `test_animation.cpp` (and `RUN_TEST` them).
The fixture reader is shared with Task 5, so it goes in now:

```cpp
constexpr float STANCE[6][4] = {{122, 152, -66, 1},  {171, 0, -66, 1},  {122, -152, -66, 1},
                                {-122, 152, -66, 1}, {-171, 0, -66, 1}, {-122, -152, -66, 1}};

struct ParamList {
    anim::ParamValue values[anim::PARAM_COUNT];
    int count = 0;
};

int paramIdByName(const std::string &name) {
    static const char *NAMES[] = {"SPEED",      "BODY_X",    "BODY_Y",            "BODY_Z", "BODY_ROLL",
                                  "BODY_PITCH", "BODY_YAW",  "FOOT_LIFT",         "OVERLAY_AMPLITUDE", "REPEAT"};
    for (int i = 0; i < anim::PARAM_COUNT; ++i)
        if (name == NAMES[i]) return i;
    TEST_FAIL_MESSAGE(name.c_str());
    return -1;
}

ParamList parseParams(const std::string &token) {
    ParamList out;
    if (token == "-") return out;
    std::stringstream ss(token);
    std::string item;
    while (std::getline(ss, item, ';')) {
        const size_t eq = item.find('=');
        out.values[out.count++] = {paramIdByName(item.substr(0, eq)), std::stof(item.substr(eq + 1))};
    }
    return out;
}

struct EvalRow {
    std::string animation;
    ParamList params;
    float t;
    uint32_t mask;
    float angles[18];
};

struct PlayerEvent {
    int step;
    bool play;
    std::string animation;
    ParamList params;
};

struct TraceRow {
    std::string state;
    uint32_t mask;
    float angles[18];
};

struct PlayerCase {
    std::string animation;
    ParamList params;
    float dt;
    float body[6];
    float feet[6][3];
    std::vector<PlayerEvent> events;
    std::vector<TraceRow> trace;
};

struct Fixtures {
    float tolerance = 1e-4f;
    std::vector<EvalRow> evaluate;
    std::vector<PlayerCase> player;
};

const Fixtures &fixtures() {
    static Fixtures f;
    static bool loaded = false;
    if (loaded) return f;
    std::ifstream in(std::string(FIXTURE_DIR) + "expected.txt");
    TEST_ASSERT_TRUE_MESSAGE(in.good(), "animations/fixtures/expected.txt (run from the repo root)");
    std::string line;
    while (std::getline(in, line)) {
        std::stringstream ss(line);
        std::string kind;
        ss >> kind;
        if (kind == "T") {
            ss >> f.tolerance;
        } else if (kind == "E") {
            EvalRow r;
            std::string params;
            ss >> r.animation >> params >> r.t >> r.mask;
            r.params = parseParams(params);
            for (float &a : r.angles) ss >> a;
            f.evaluate.push_back(r);
        } else if (kind == "P") {
            PlayerCase c;
            int index;
            std::string params;
            ss >> index >> c.animation >> params >> c.dt;
            c.params = parseParams(params);
            for (float &b : c.body) ss >> b;
            for (auto &foot : c.feet)
                for (float &v : foot) ss >> v;
            f.player.push_back(c);
        } else if (kind == "V") {
            PlayerEvent e;
            std::string action;
            ss >> e.step >> action;
            e.play = action == "play";
            if (e.play) {
                std::string params;
                ss >> e.animation >> params;
                e.params = parseParams(params);
            }
            f.player.back().events.push_back(e);
        } else if (kind == "S") {
            TraceRow r;
            ss >> r.state >> r.mask;
            for (float &a : r.angles) ss >> a;
            f.player.back().trace.push_back(r);
        }
    }
    loaded = true;
    TEST_ASSERT_TRUE(f.evaluate.size() > 100);
    TEST_ASSERT_EQUAL(7, f.player.size());
    return f;
}

anim::Clip &clipByName(const std::string &name) {
    static std::map<std::string, anim::Clip> cache;
    auto it = cache.find(name);
    if (it != cache.end()) return it->second;
    animation_Animation a;
    TEST_ASSERT_TRUE_MESSAGE(decodeFixture(name.c_str(), a), name.c_str());
    anim::Clip &clip = cache[name];
    anim::fromProto(a, clip);
    TEST_ASSERT_NULL(anim::validate(clip));
    return clip;
}

// Servo resolution is about a tenth of a degree; a float32 port agreeing with the float64
// reference within a thousandth is parity.
constexpr float ANGLE_TOL_DEG = 1e-3f;

void test_evaluate_matches_every_fixture_sample() {
    Kinematics kin;
    int checked = 0;
    for (const EvalRow &row : fixtures().evaluate) {
        const anim::Clip &clip = clipByName(row.animation);
        float params[anim::PARAM_COUNT];
        anim::resolveParams(clip, row.params.values, row.params.count, params);
        anim::Pose pose;
        anim::evaluate(clip, params, row.t, kin, STANCE, pose);
        float angles[18];
        const uint32_t mask = anim::poseToAngles(pose, kin, STANCE, angles);
        char where[64];
        snprintf(where, sizeof(where), "%s t=%g", row.animation.c_str(), row.t);
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(row.mask, mask, where);
        for (int j = 0; j < 18; ++j) TEST_ASSERT_FLOAT_WITHIN_MESSAGE(ANGLE_TOL_DEG, row.angles[j], angles[j], where);
        ++checked;
    }
    TEST_ASSERT_TRUE(checked > 100);
}

void test_ease_curves_match_the_reference() {
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.3f, anim::easeValue(anim::LINEAR, 0.3f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.25f, anim::easeValue(anim::EASE_IN, 0.5f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.75f, anim::easeValue(anim::EASE_OUT, 0.5f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.125f, anim::easeValue(anim::EASE_IN_OUT, 0.25f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.875f, anim::easeValue(anim::EASE_IN_OUT, 0.75f));
}

void test_resolve_params_defaults_clamps_and_ignores_undeclared() {
    anim::Clip c = twoKeyframes();
    c.paramCount = 2;
    c.params[0] = {anim::SPEED, 0.5f, 1.0f, 2.0f};
    c.params[1] = {anim::FOOT_LIFT, 0.0f, 0.8f, 1.0f};
    float p[anim::PARAM_COUNT];
    anim::resolveParams(c, nullptr, 0, p);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, p[anim::SPEED]);
    TEST_ASSERT_EQUAL_FLOAT(0.8f, p[anim::FOOT_LIFT]);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, p[anim::BODY_Z]);
    const anim::ParamValue values[] = {{anim::SPEED, 9.0f}, {anim::BODY_Z, 0.1f}};
    anim::resolveParams(c, values, 2, p);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, p[anim::SPEED]);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, p[anim::BODY_Z]);
}

void test_pose_to_angles_clamps_overrides_and_flags_unreachable_feet() {
    Kinematics kin;
    anim::Pose pose;
    pose.legs[1] = {true, {40.0f, 95.0f, -100.0f}};
    pose.legs[5] = {false, {0.0f, 0.0f, 120.0f}};
    float angles[18];
    const uint32_t mask = anim::poseToAngles(pose, kin, STANCE, angles);
    TEST_ASSERT_EQUAL_FLOAT(31.5f, angles[3]);
    TEST_ASSERT_EQUAL_FLOAT(90.0f, angles[4]);
    TEST_ASSERT_EQUAL_FLOAT(-100.0f, angles[5]);
    TEST_ASSERT_TRUE(mask & (1u << 3));
    TEST_ASSERT_TRUE(mask & (1u << 4));
    TEST_ASSERT_FALSE(mask & (1u << 5));
    TEST_ASSERT_TRUE(mask & (1u << 16));
    TEST_ASSERT_FALSE(mask & 0x7u);
    anim::Pose reach;
    reach.legs[1] = {false, {100.0f, 0.0f, 0.0f}};
    const uint32_t reachMask = anim::poseToAngles(reach, kin, STANCE, angles);
    TEST_ASSERT_EQUAL_UINT32((1u << 4) | (1u << 5), reachMask);
    anim::Pose stance;
    TEST_ASSERT_EQUAL_UINT32(0, anim::poseToAngles(stance, kin, STANCE, angles));
    BodyStateMsg b{};
    b.updateFeet(STANCE);
    float expected[18];
    kin.inverseKinematics(b, expected);
    for (int j = 0; j < 18; ++j) TEST_ASSERT_FLOAT_WITHIN(1e-5f, expected[j], angles[j]);
}

void test_capture_pose_reads_offsets_from_a_body_state() {
    BodyStateMsg b{};
    b.updateFeet(STANCE);
    b.omega = 0.1f;
    b.zm = 15.0f;
    b.feet[2][0] += 1.0f;
    b.feet[2][1] += 2.0f;
    b.feet[2][2] += 3.0f;
    anim::Pose pose;
    anim::capturePose(b, STANCE, pose);
    TEST_ASSERT_EQUAL_FLOAT(0.1f, pose.body[anim::ROLL]);
    TEST_ASSERT_EQUAL_FLOAT(15.0f, pose.body[anim::Z]);
    TEST_ASSERT_FALSE(pose.legs[2].joints);
    TEST_ASSERT_EQUAL_FLOAT(3.0f, pose.legs[2].v[2]);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pose.legs[0].v[0]);
}
```

Add `#include <map>` to the test includes.

- [ ] **Step 2: Run to verify failure**

`pio test -e native`: fails to compile on the missing functions.

- [ ] **Step 3: Add reach detection to the kinematics**

In `firmware/include/kinematics.h`, inside `class Kinematics` after `inverseKinematics`:

```cpp
    // True when foot i of b lies inside the leg's reach annulus, which is exactly the condition under
    // which neither acos argument in inverseKinematics saturates. Mirrors foot_reachable() in
    // simulation/src/robot/animation.py.
    bool footReachable(const BodyStateMsg &b, int i) const {
        float T[4][4], w[4];
        get_transformation_matrix(b, T);
        MAT_MULT(T, b.feet[i], w, 4, 4, 1);
        const float wx = w[0] - mountPos[i][0];
        const float wy = w[1] - mountPos[i][1];
        const float lx = wx * ca[i] + wy * sa[i];
        const float ly = wx * sa[i] - wy * ca[i];
        const float dx = lx - rootJ1;
        const float radial = hypotf(dx, ly) - j1J2;
        const float lr = hypotf(radial, w[2] - mountPos[i][2]);
        return fabsf(j2J3 - j3Tip) <= lr && lr <= j2J3 + j3Tip;
    }
```

`MAT_MULT` on a `const` float array: check how `inverseKinematics` calls it and mirror the argument types; if the macro needs a non-const input, copy `b.feet[i]` into a local `float foot[4]` first.

- [ ] **Step 4: Write the evaluator**

Append inside `namespace anim` in `animation.h`, before the closing brace:

```cpp
struct Pose {
    float body[6] = {0, 0, 0, 0, 0, 0};
    LegTarget legs[6];
};

inline float easeValue(int kind, float t) {
    switch (kind) {
        case EASE_IN: return t * t;
        case EASE_OUT: return t * (2.0f - t);
        case EASE_IN_OUT: return t < 0.5f ? 2.0f * t * t : -1.0f + (4.0f - 2.0f * t) * t;
        default: return t;
    }
}

// Every id gets a value: a declared id takes the caller's value clamped to its range, else its
// default; an undeclared id is 1 (the neutral multiplier and a single play).
inline void resolveParams(const Clip &c, const ParamValue *values, int count, float out[PARAM_COUNT]) {
    for (int i = 0; i < PARAM_COUNT; ++i) out[i] = 1.0f;
    for (int i = 0; i < c.paramCount; ++i) {
        const ParamSpec &spec = c.params[i];
        float v = spec.defaultValue;
        for (int j = 0; j < count; ++j)
            if (values[j].id == spec.id) v = values[j].value;
        out[spec.id] = CLIP(v, spec.min, spec.max);
    }
}

inline void bodyState(const float body6[6], const float stance[6][4], BodyStateMsg &b) {
    b.omega = body6[ROLL];
    b.phi = body6[PITCH];
    b.psi = body6[YAW];
    b.xm = body6[X];
    b.ym = body6[Y];
    b.zm = body6[Z];
    b.updateFeet(stance);
}

inline void legJointsDeg(Kinematics &kin, const float body6[6], const float foot[3], int leg,
                         const float stance[6][4], float out[3]) {
    BodyStateMsg b;
    bodyState(body6, stance, b);
    for (int k = 0; k < 3; ++k) b.feet[leg][k] += foot[k];
    float angles[18];
    kin.inverseKinematics(b, angles);
    for (int k = 0; k < 3; ++k) out[k] = angles[leg * 3 + k];
}

// The keyframe pair bracketing t and the eased fraction between them, with t clamped.
inline void segment(const Clip &c, float t, const Keyframe *&k0, const Keyframe *&k1, float &u) {
    if (t <= 0.0f || c.keyframeCount == 1) {
        k0 = k1 = &c.keyframes[0];
        u = 0.0f;
        return;
    }
    if (t >= c.keyframes[c.keyframeCount - 1].time) {
        k0 = k1 = &c.keyframes[c.keyframeCount - 1];
        u = 0.0f;
        return;
    }
    int i = 1;
    while (c.keyframes[i].time < t) ++i;
    k0 = &c.keyframes[i - 1];
    k1 = &c.keyframes[i];
    u = easeValue(k1->ease, (t - k0->time) / (k1->time - k0->time));
}

// Order, matching the reference: interpolate the body; add body overlays and collect each foot
// overlay; apply the BODY_* multipliers (this is the output body); then resolve legs. A foot leg is
// lerped, gets its overlay and FOOT_LIFT on z. A joint leg is lerped raw. A mixed leg takes the foot
// endpoint with its overlay and lift, runs IK against the OUTPUT body, and lerps in joint space with
// the raw joint endpoint, so the servo command is continuous at the switching keyframe.
inline void evaluate(const Clip &c, const float params[PARAM_COUNT], float t, Kinematics &kin,
                     const float stance[6][4], Pose &out) {
    const Keyframe *k0;
    const Keyframe *k1;
    float u;
    segment(c, t, k0, k1, u);
    t = CLIP(t, 0.0f, c.duration());
    float body[6];
    for (int a = 0; a < 6; ++a) body[a] = k0->body[a] + (k1->body[a] - k0->body[a]) * u;
    float footOverlay[6][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    for (int i = 0; i < c.overlayCount; ++i) {
        const Overlay &o = c.overlays[i];
        if (!(o.start <= t && t <= o.end)) continue;
        const float v = o.amplitude * params[OVERLAY_AMPLITUDE] * sinf(2.0f * (float)M_PI * o.frequency * t + o.phase);
        if (o.onBody) body[o.channel] += v;
        else footOverlay[o.channel / 3][o.channel % 3] += v;
    }
    for (int a = 0; a < 6; ++a) body[a] *= params[BODY_PARAM_FOR_AXIS[a]];
    for (int a = 0; a < 6; ++a) out.body[a] = body[a];

    for (int i = 0; i < 6; ++i) {
        const LegTarget a = legTarget(*k0, i);
        const LegTarget b = legTarget(*k1, i);
        LegTarget &leg = out.legs[i];
        auto lifted = [&](const float foot[3], float dst[3]) {
            for (int k = 0; k < 3; ++k) dst[k] = foot[k] + footOverlay[i][k];
            dst[2] *= params[FOOT_LIFT];
        };
        if (!a.joints && !b.joints) {
            float lerp[3];
            for (int k = 0; k < 3; ++k) lerp[k] = a.v[k] + (b.v[k] - a.v[k]) * u;
            leg.joints = false;
            lifted(lerp, leg.v);
        } else if (a.joints && b.joints) {
            leg.joints = true;
            for (int k = 0; k < 3; ++k) leg.v[k] = a.v[k] + (b.v[k] - a.v[k]) * u;
        } else {
            float ja[3], jb[3], foot[3];
            if (a.joints) {
                for (int k = 0; k < 3; ++k) ja[k] = a.v[k];
            } else {
                lifted(a.v, foot);
                legJointsDeg(kin, body, foot, i, stance, ja);
            }
            if (b.joints) {
                for (int k = 0; k < 3; ++k) jb[k] = b.v[k];
            } else {
                lifted(b.v, foot);
                legJointsDeg(kin, body, foot, i, stance, jb);
            }
            leg.joints = true;
            for (int k = 0; k < 3; ++k) leg.v[k] = ja[k] + (jb[k] - ja[k]) * u;
        }
    }
}

// 18 servo angles (deg, IK order) and an 18-bit mask: bit leg*3+joint for a joint pinned at its
// limit, and the femur and tibia bits of a foot leg the IK cannot reach.
inline uint32_t poseToAngles(const Pose &p, Kinematics &kin, const float stance[6][4], float angles[18]) {
    BodyStateMsg b;
    bodyState(p.body, stance, b);
    for (int i = 0; i < 6; ++i)
        if (!p.legs[i].joints)
            for (int k = 0; k < 3; ++k) b.feet[i][k] += p.legs[i].v[k];
    kin.inverseKinematics(b, angles);
    uint32_t mask = 0;
    for (int i = 0; i < 6; ++i) {
        if (p.legs[i].joints) {
            for (int k = 0; k < 3; ++k) angles[i * 3 + k] = p.legs[i].v[k];
        } else if (!kin.footReachable(b, i)) {
            mask |= 0x6u << (i * 3);
        }
    }
    for (int j = 0; j < 18; ++j) {
        const float limit = JOINT_LIMIT_DEG[j % 3];
        const float clamped = CLIP(angles[j], -limit, limit);
        if (clamped != angles[j]) {
            mask |= 1u << j;
            angles[j] = clamped;
        }
    }
    return mask;
}

inline void capturePose(const BodyStateMsg &b, const float stance[6][4], Pose &out) {
    out.body[ROLL] = b.omega;
    out.body[PITCH] = b.phi;
    out.body[YAW] = b.psi;
    out.body[X] = b.xm;
    out.body[Y] = b.ym;
    out.body[Z] = b.zm;
    for (int i = 0; i < 6; ++i) {
        out.legs[i].joints = false;
        for (int k = 0; k < 3; ++k) out.legs[i].v[k] = b.feet[i][k] - stance[i][k];
    }
}
```

Compare `evaluate` and `poseToAngles` line by line with the reference before running; the reference's `_resolve_leg` and `pose_to_angles` are the source of truth for the foot-overlay and lift order and for which legs get reach bits.

- [ ] **Step 5: Run the tests**

`pio test -e native`: expected all pass, `test_evaluate_matches_every_fixture_sample` covering every `E` row.
If a row disagrees by more than the tolerance, print the row and the two angle vectors, find which rule diverged (segment end handling at exactly a keyframe time, overlay window inclusivity, the mixed-leg body, the reach rule), and fix the port.
Do not widen the tolerance beyond `1e-3` deg.

- [ ] **Step 6: Commit**

```bash
git add firmware/include/kinematics.h firmware/include/animation/animation.h firmware/test/test_animation/test_animation.cpp
git commit -m "✨ Ports the animation evaluator and pose to servo angle path with reach detection"
```

---

### Task 5: Player

**Files:**
- Modify: `firmware/include/animation/animation.h` (part 3)
- Test: `firmware/test/test_animation/test_animation.cpp`

**Interfaces:**
- Produces: `anim::State { IDLE, ENTRY, PLAYING, HOLD, EXIT }`, `class anim::Player` with `Player(Kinematics&)`, `setStance(const float (*stance)[4])`, `play(const Clip*, const ParamValue*, int, const Pose* live)`, `stop()`, `const Pose &update(float dt)`, `State state() const`, `float t() const`, `const Pose &lastPose() const`, `const Clip *clip() const`, `const float *params() const`.

- [ ] **Step 1: Write the failing test**

Append (and `RUN_TEST`):

```cpp
const char *stateName(anim::State s) {
    switch (s) {
        case anim::State::IDLE: return "IDLE";
        case anim::State::ENTRY: return "ENTRY";
        case anim::State::PLAYING: return "PLAYING";
        case anim::State::HOLD: return "HOLD";
        default: return "EXIT";
    }
}

void test_player_matches_every_fixture_trace() {
    Kinematics kin;
    int steps = 0;
    for (size_t ci = 0; ci < fixtures().player.size(); ++ci) {
        const PlayerCase &c = fixtures().player[ci];
        anim::Player player(kin);
        player.setStance(STANCE);
        BodyStateMsg live{};
        live.updateFeet(STANCE);
        live.omega = c.body[0];
        live.phi = c.body[1];
        live.psi = c.body[2];
        live.xm = c.body[3];
        live.ym = c.body[4];
        live.zm = c.body[5];
        for (int i = 0; i < 6; ++i)
            for (int k = 0; k < 3; ++k) live.feet[i][k] += c.feet[i][k];
        anim::Pose livePose;
        anim::capturePose(live, STANCE, livePose);
        player.play(&clipByName(c.animation), c.params.values, c.params.count, &livePose);
        for (size_t step = 0; step < c.trace.size(); ++step) {
            for (const PlayerEvent &e : c.events) {
                if (e.step != (int)step) continue;
                if (e.play) player.play(&clipByName(e.animation), e.params.values, e.params.count, nullptr);
                else player.stop();
            }
            const anim::Pose &pose = player.update(c.dt);
            float angles[18];
            const uint32_t mask = anim::poseToAngles(pose, kin, STANCE, angles);
            char where[96];
            snprintf(where, sizeof(where), "case %u (%s) step %u", (unsigned)ci, c.animation.c_str(), (unsigned)step);
            TEST_ASSERT_EQUAL_STRING_MESSAGE(c.trace[step].state.c_str(), stateName(player.state()), where);
            TEST_ASSERT_EQUAL_UINT32_MESSAGE(c.trace[step].mask, mask, where);
            for (int j = 0; j < 18; ++j)
                TEST_ASSERT_FLOAT_WITHIN_MESSAGE(ANGLE_TOL_DEG, c.trace[step].angles[j], angles[j], where);
            ++steps;
        }
    }
    TEST_ASSERT_EQUAL(819, steps);
}

void test_stop_before_the_first_update_blends_from_the_live_pose() {
    Kinematics kin;
    anim::Player player(kin);
    player.setStance(STANCE);
    anim::Pose live;
    live.body[anim::Z] = 12.0f;
    live.legs[0].v[1] = 15.0f;
    player.play(&clipByName("fx_single"), nullptr, 0, &live);
    player.stop();
    const anim::Pose &pose = player.update(0.0f);
    TEST_ASSERT_EQUAL(anim::State::EXIT, player.state());
    TEST_ASSERT_EQUAL_FLOAT(12.0f, pose.body[anim::Z]);
    TEST_ASSERT_EQUAL_FLOAT(15.0f, pose.legs[0].v[1]);
}
```

If the trace count in `expected.txt` is not 819 (it is the sum of the seven cases' steps: 170, 84, 120, 71, 105, 128, 141), use the actual sum and say so in the report.

- [ ] **Step 2: Run to verify failure**

`pio test -e native`: compile failure on `anim::Player`.

- [ ] **Step 3: Write the player**

Append inside `namespace anim`:

```cpp
enum class State : int { IDLE = 0, ENTRY = 1, PLAYING = 2, HOLD = 3, EXIT = 4 };

// Entry -> Playing -> Hold | Exit -> Idle around evaluate(). Mirrors the reference Player: REPEAT is
// max(1, floor(x + 0.5)); SPEED scales only the playing clock; Entry and Exit run on wall time; a
// play during any state starts Entry from the current pose; Exit after a finished play starts from
// the final keyframe; play() records the live pose so an immediate stop blends from it.
class Player {
  public:
    explicit Player(Kinematics &kin) : kin_(kin) {}

    // The standing feet every offset is relative to; the caller keeps them alive and current.
    void setStance(const float (*stance)[4]) { stance_ = stance; }

    void play(const Clip *clip, const ParamValue *values, int count, const Pose *live) {
        clip_ = clip;
        resolveParams(*clip, values, count, params_);
        t_ = 0.0f;
        playsDone_ = 0;
        const Pose start = live ? *live : lastPose_;
        lastPose_ = start;
        Pose first;
        evaluate(*clip, params_, 0.0f, kin_, stance_, first);
        startBlend(start, first, clip->entrySeconds(), State::ENTRY);
    }

    void stop() {
        if (state_ == State::IDLE) return;
        startBlend(lastPose_, Pose{}, clip_->exitSeconds(), State::EXIT);
    }

    const Pose &update(float dt) {
        if (state_ == State::IDLE) return lastPose_;
        if (state_ == State::ENTRY || state_ == State::EXIT) advanceBlend(dt);
        else if (state_ == State::HOLD) evaluate(*clip_, params_, clip_->duration(), kin_, stance_, lastPose_);
        else advancePlaying(dt);
        return lastPose_;
    }

    State state() const { return state_; }
    float t() const { return t_; }
    const Pose &lastPose() const { return lastPose_; }
    const Clip *clip() const { return clip_; }
    const float *params() const { return params_; }

  private:
    Kinematics &kin_;
    const float (*stance_)[4] = nullptr;
    State state_ = State::IDLE;
    const Clip *clip_ = nullptr;
    float params_[PARAM_COUNT] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    float t_ = 0.0f;
    Pose lastPose_;
    int playsDone_ = 0;
    float blendT_ = 0.0f;
    float blendSeconds_ = 1.0f;
    Pose blendFrom_;
    Pose blendTo_;

    // Legs where either side is a joint target are converted to joints on both sides once, so the
    // blend itself is a plain lerp; foot legs keep their offsets and get the step arc.
    void startBlend(const Pose &src, const Pose &dst, float seconds, State state) {
        blendFrom_ = src;
        blendTo_ = dst;
        for (int i = 0; i < 6; ++i) {
            if (!blendFrom_.legs[i].joints && !blendTo_.legs[i].joints) continue;
            if (!blendFrom_.legs[i].joints) {
                float j[3];
                legJointsDeg(kin_, blendFrom_.body, blendFrom_.legs[i].v, i, stance_, j);
                blendFrom_.legs[i] = {true, {j[0], j[1], j[2]}};
            }
            if (!blendTo_.legs[i].joints) {
                float j[3];
                legJointsDeg(kin_, blendTo_.body, blendTo_.legs[i].v, i, stance_, j);
                blendTo_.legs[i] = {true, {j[0], j[1], j[2]}};
            }
        }
        blendSeconds_ = seconds;
        blendT_ = 0.0f;
        state_ = state;
    }

    void advanceBlend(float dt) {
        blendT_ += dt;
        const float u = fminf(1.0f, blendT_ / blendSeconds_);
        const float e = easeValue(EASE_IN_OUT, u);
        for (int a = 0; a < 6; ++a) lastPose_.body[a] = blendFrom_.body[a] + (blendTo_.body[a] - blendFrom_.body[a]) * e;
        for (int i = 0; i < 6; ++i) {
            const LegTarget &la = blendFrom_.legs[i];
            const LegTarget &lb = blendTo_.legs[i];
            LegTarget &out = lastPose_.legs[i];
            out.joints = la.joints;
            for (int k = 0; k < 3; ++k) out.v[k] = la.v[k] + (lb.v[k] - la.v[k]) * e;
            if (la.joints) continue;
            const float travel = hypotf(lb.v[0] - la.v[0], lb.v[1] - la.v[1]);
            if (travel > STEP_ARC_MIN_TRAVEL_MM)
                out.v[2] += STEP_ARC_MM * fminf(1.0f, travel / STEP_ARC_FULL_TRAVEL_MM) * sinf((float)M_PI * u);
        }
        if (u >= 1.0f) {
            if (state_ == State::ENTRY) {
                state_ = State::PLAYING;
                t_ = 0.0f;
            } else {
                state_ = State::IDLE;
            }
        }
    }

    void advancePlaying(float dt) {
        const float duration = clip_->duration();
        t_ += dt * params_[SPEED];
        if (clip_->loop) {
            t_ = duration > 0.0f ? fmodf(t_, duration) : 0.0f;
            evaluate(*clip_, params_, t_, kin_, stance_, lastPose_);
            return;
        }
        if (t_ < duration) {
            evaluate(*clip_, params_, t_, kin_, stance_, lastPose_);
            return;
        }
        ++playsDone_;
        const int repeat = (int)fmaxf(1.0f, floorf(params_[REPEAT] + 0.5f));
        if (playsDone_ < repeat) {
            t_ = duration > 0.0f ? t_ - duration : 0.0f;
            evaluate(*clip_, params_, t_, kin_, stance_, lastPose_);
            return;
        }
        evaluate(*clip_, params_, duration, kin_, stance_, lastPose_);
        if (clip_->holdEnd) state_ = State::HOLD;
        else stop();
    }
};
```

Note on the finished play: the reference writes the final pose to `last_pose` and then calls `stop()`, which reads it; `evaluate(..., lastPose_)` followed by `stop()` does the same here.

Note on `fmodf`: Python's `%` on a positive clock equals `fmodf` for non-negative `t_`; `t_` never goes negative.

- [ ] **Step 4: Run the tests**

`pio test -e native`: expected all pass, the trace test covering every step of every case.
A state mismatch one step early or late means a boundary comparison differs from the reference (`>=` vs `>`, or where the clamp of `u` happens); fix the port, never the fixture.

- [ ] **Step 5: Commit**

```bash
git add firmware/include/animation/animation.h firmware/test/test_animation/test_animation.cpp
git commit -m "✨ Ports the animation player and proves parity against every fixture trace"
```

---

### Task 6: Store, runner, ANIMATE mode and the play, stop and pose messages

**Files:**
- Create: `firmware/include/animation/animation_store.h`
- Create: `firmware/include/animation/animation_runner.h`
- Modify: `firmware/include/message_types.h` (`PoseMsg`, `AnimationCommandMsg`)
- Modify: `firmware/include/motion.h`
- Modify: `firmware/src/main.cpp` (handlers and status bridge)
- Test: `firmware/test/test_animation/test_animation.cpp` (runner logic tested through a host-buildable core)

Precondition: the other session's working-tree edits are committed or stashed (see Global Constraints).
The controller confirms this before dispatch.

**Interfaces:**
- Consumes: `anim::Player`, `anim::validate`, `anim::fromProto`, `EventBus`, `MotionService` internals.
- Produces: `PoseMsg`, `AnimationCommandMsg`; `AnimationStore::load(name, Clip&, const char *&error)`, `AnimationStore::list(callback)`, `AnimationStore::path(name, buf)`; `AnimationRunner::begin(Kinematics&, const float (*stance)[4])`, `requestPlay(const AnimationCommandMsg&)`, `requestStop()`, `setPuppet(const PoseMsg&)`, `reset()`, `bool tick(float dt, BodyStateMsg&, float angles[18])` returning true on the tick the player returns to idle from a borrowed play; `MotionService` `ANIMATE` branch and the borrow and hand-back logic.

- [ ] **Step 1: Message structs**

Append to `firmware/include/message_types.h`:

```cpp
// Puppeteer target from the editor: body offsets and six leg targets, foot offsets or joint angles.
struct PoseMsg {
    float body[6];
    bool joints[6];
    float legs[6][3];
};

struct AnimationCommandMsg {
    bool play;  // false = stop
    char name[33];
    int paramCount;
    struct {
        int id;
        float value;
    } params[10];
};
```

- [ ] **Step 2: The store**

`firmware/include/animation/animation_store.h`:

```cpp
#pragma once

// Reads /littlefs/animations/<name>.pb into PSRAM, decodes it and validates it. The encoded and the
// decoded buffers are allocated once: a full clip decodes to about 5 KB and the encoded file is
// bounded by the schema, so neither belongs on a task stack or in internal RAM.

#include <dirent.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <sys/stat.h>
#include <cstdio>
#include <cstring>
#include <functional>
#include <pb_decode.h>
#include <animation/animation.h>
#include <animation/animation_codec.h>
#include <filesystem.h>

#define ANIMATION_DIRECTORY MOUNT_POINT "/animations"

class AnimationStore {
  public:
    static constexpr size_t ENCODED_MAX = animation_Animation_size;

    bool begin() {
        encoded_ = (uint8_t *)heap_caps_malloc(ENCODED_MAX, MALLOC_CAP_SPIRAM);
        decoded_ = (animation_Animation *)heap_caps_malloc(sizeof(animation_Animation), MALLOC_CAP_SPIRAM);
        if (!encoded_ || !decoded_) {
            ESP_LOGE(TAG, "no PSRAM for the animation buffers");
            return false;
        }
        FileSystem::mkdirRecursive(ANIMATION_DIRECTORY);
        return true;
    }

    static bool path(const char *name, char *out, size_t size) {
        if (!anim::validName(name)) return false;
        snprintf(out, size, ANIMATION_DIRECTORY "/%s.pb", name);
        return true;
    }

    // Decodes and validates; on failure `error` names the reason and `out` is unspecified.
    // Callers serialise: the adapter tasks and the event bus worker share the two buffers.
    bool load(const char *name, anim::Clip &out, const char *&error) {
        char file[96];
        if (!path(name, file, sizeof(file))) {
            error = "invalid name";
            return false;
        }
        FILE *f = fopen(file, "rb");
        if (!f) {
            error = "no such animation";
            return false;
        }
        const size_t n = fread(encoded_, 1, ENCODED_MAX, f);
        const bool more = fgetc(f) != EOF;
        fclose(f);
        if (more) {
            error = "file larger than the schema allows";
            return false;
        }
        pb_istream_t stream = pb_istream_from_buffer(encoded_, n);
        *decoded_ = animation_Animation_init_zero;
        if (!pb_decode(&stream, animation_Animation_fields, decoded_)) {
            error = PB_GET_ERROR(&stream);
            return false;
        }
        anim::fromProto(*decoded_, out);
        error = anim::validate(out);
        if (error) return false;
        if (strcmp(out.name, name) != 0) {
            error = "name does not match the file";
            return false;
        }
        return true;
    }

    // Calls fn(name, bytes) for every .pb in the directory.
    static void list(const std::function<void(const char *, uint32_t)> &fn) {
        DIR *dir = opendir(ANIMATION_DIRECTORY);
        if (!dir) return;
        for (struct dirent *e = readdir(dir); e; e = readdir(dir)) {
            const size_t len = strlen(e->d_name);
            if (len < 4 || strcmp(e->d_name + len - 3, ".pb") != 0) continue;
            char name[anim::NAME_MAX + 1] = {0};
            strncpy(name, e->d_name, len - 3 < anim::NAME_MAX ? len - 3 : anim::NAME_MAX);
            char full[96];
            snprintf(full, sizeof(full), ANIMATION_DIRECTORY "/%s", e->d_name);
            struct stat st;
            fn(name, stat(full, &st) == 0 ? (uint32_t)st.st_size : 0);
        }
        closedir(dir);
    }

  private:
    static constexpr const char *TAG = "AnimationStore";
    uint8_t *encoded_ = nullptr;
    animation_Animation *decoded_ = nullptr;
};
```

If nanopb does not emit `animation_Animation_size` (it does when every field is bounded; the options bound them all), report it rather than guessing a size.

- [ ] **Step 3: The runner**

The runner separates the thread-safe request surface from a host-testable core.
`firmware/include/animation/animation_runner.h`:

```cpp
#pragma once

// Owns the animation state the control task drives: two clip buffers (the player reads one while a
// request loads the other), the player, the puppeteer target and the status publisher. Requests arrive
// on the event bus worker and the adapter tasks; they either load into the inactive buffer and raise
// a flag the control task consumes, or copy a pose under a critical section. The control task is the
// only reader of the active clip, so a clip is never replaced while it is being evaluated.

#include <atomic>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <animation/animation.h>
#include <animation/animation_store.h>
#include <event_bus.h>
#include <message_types.h>
#include <platform_shared/message.pb.h>
#include <utils/timing.h>

class AnimationRunner {
  public:
    static constexpr uint32_t STATUS_PERIOD_MS = 200;

    bool begin(Kinematics &kin, const float (*stance)[4]) {
        kin_ = &kin;
        stance_ = stance;
        clips_[0] = (anim::Clip *)heap_caps_malloc(sizeof(anim::Clip), MALLOC_CAP_SPIRAM);
        clips_[1] = (anim::Clip *)heap_caps_malloc(sizeof(anim::Clip), MALLOC_CAP_SPIRAM);
        if (!clips_[0] || !clips_[1]) {
            ESP_LOGE(TAG, "no PSRAM for the clip buffers");
            return false;
        }
        new (clips_[0]) anim::Clip();
        new (clips_[1]) anim::Clip();
        player_ = new anim::Player(kin);
        player_->setStance(stance);
        loadMutex_ = xSemaphoreCreateMutex();
        return store_.begin();
    }

    // Adapter or worker task. Loads into the buffer the player is not reading; the control task
    // swaps and starts it on its next tick. A failed load leaves the player untouched and is logged;
    // the same file fails the same way on a validate request, which is how the app learns why.
    void requestPlay(const AnimationCommandMsg &cmd) {
        xSemaphoreTake(loadMutex_, portMAX_DELAY);
        const int idle = 1 - active_.load(std::memory_order_acquire);
        const char *error = nullptr;
        const bool ok = store_.load(cmd.name, *clips_[idle], error);
        if (ok) {
            pendingParams_ = cmd;
            pendingPlay_.store(true, std::memory_order_release);
        } else {
            ESP_LOGW(TAG, "play %s refused: %s", cmd.name, error);
        }
        xSemaphoreGive(loadMutex_);
    }

    void requestStop() { pendingStop_.store(true, std::memory_order_release); }

    void setPuppet(const PoseMsg &p) {
        portENTER_CRITICAL(&puppetMux_);
        puppet_ = p;
        puppetPending_ = true;
        portEXIT_CRITICAL(&puppetMux_);
    }

    // Validate a file for a request handler, using the inactive buffer under the load lock.
    void validate(const char *name, socket_message_AnimationReport &report) {
        xSemaphoreTake(loadMutex_, portMAX_DELAY);
        const int idle = 1 - active_.load(std::memory_order_acquire);
        const char *error = nullptr;
        report.ok = store_.load(name, *clips_[idle], error);
        if (!report.ok) {
            strncpy(report.error, error, sizeof(report.error) - 1);
        } else {
            report.clamped_mask = sweepClampMask(*clips_[idle]);
        }
        xSemaphoreGive(loadMutex_);
    }

    // Leaving ANIMATE: drop any play or pose and return the player to idle at stance.
    void reset() {
        pendingPlay_.store(false, std::memory_order_release);
        pendingStop_.store(false, std::memory_order_release);
        portENTER_CRITICAL(&puppetMux_);
        puppetPending_ = false;
        havePuppet_ = false;
        portEXIT_CRITICAL(&puppetMux_);
        player_->~Player();
        new (player_) anim::Player(*kin_);
        player_->setStance(stance_);
        current_ = anim::Pose{};
        publishStatus(true);
    }

    // Control task, every tick in ANIMATE. Fills body (offsets applied to the stance) and the 18
    // angles. Returns true on the tick the player returns to idle after a play, so the caller can
    // hand a borrowed mode back.
    bool tick(float dt, BodyStateMsg &body, float angles[18]) {
        consumeRequests();
        const anim::State before = player_->state();
        if (player_->state() != anim::State::IDLE) {
            current_ = player_->update(dt);
        } else if (havePuppet_) {
            approachPuppet();
        }
        clampMask_ = anim::poseToAngles(current_, *kin_, stance_, angles);
        anim::bodyState(current_.body, stance_, body);
        for (int i = 0; i < 6; ++i)
            if (!current_.legs[i].joints)
                for (int k = 0; k < 3; ++k) body.feet[i][k] += current_.legs[i].v[k];
        const bool finished = before != anim::State::IDLE && player_->state() == anim::State::IDLE;
        publishStatus(before != player_->state());
        return finished;
    }

    bool playing() const { return player_->state() != anim::State::IDLE; }

  private:
    static constexpr const char *TAG = "AnimationRunner";
    static constexpr float PUPPET_SMOOTHING = 0.06f;  // the STAND smoothing factor
    Kinematics *kin_ = nullptr;
    const float (*stance_)[4] = nullptr;
    AnimationStore store_;
    anim::Clip *clips_[2] = {nullptr, nullptr};
    std::atomic<int> active_{0};
    anim::Player *player_ = nullptr;
    SemaphoreHandle_t loadMutex_ = nullptr;
    std::atomic<bool> pendingPlay_{false};
    std::atomic<bool> pendingStop_{false};
    AnimationCommandMsg pendingParams_{};
    portMUX_TYPE puppetMux_ = portMUX_INITIALIZER_UNLOCKED;
    PoseMsg puppet_{};
    bool puppetPending_ = false;
    bool havePuppet_ = false;
    anim::Pose puppetTarget_;
    anim::Pose current_;
    uint32_t clampMask_ = 0;
    unsigned long lastStatusMs_ = 0;

    void consumeRequests() {
        if (pendingPlay_.exchange(false, std::memory_order_acq_rel)) {
            const int idle = 1 - active_.load(std::memory_order_acquire);
            active_.store(idle, std::memory_order_release);
            anim::ParamValue values[anim::PARAM_MAX];
            for (int i = 0; i < pendingParams_.paramCount; ++i)
                values[i] = {pendingParams_.params[i].id, pendingParams_.params[i].value};
            player_->play(clips_[idle], values, pendingParams_.paramCount, &current_);
            havePuppet_ = false;
        }
        if (pendingStop_.exchange(false, std::memory_order_acq_rel)) player_->stop();
        portENTER_CRITICAL(&puppetMux_);
        if (puppetPending_) {
            puppetPending_ = false;
            for (int a = 0; a < 6; ++a) puppetTarget_.body[a] = puppet_.body[a];
            for (int i = 0; i < 6; ++i) {
                puppetTarget_.legs[i].joints = puppet_.joints[i];
                for (int k = 0; k < 3; ++k) puppetTarget_.legs[i].v[k] = puppet_.legs[i][k];
            }
            havePuppet_ = true;
        }
        portEXIT_CRITICAL(&puppetMux_);
    }

    // Lerp toward the puppet target with the STAND smoothing; a leg whose representation changes
    // snaps to the new representation first, which is what the editor's toggle means.
    void approachPuppet() {
        for (int a = 0; a < 6; ++a) current_.body[a] = lerpf(current_.body[a], puppetTarget_.body[a], PUPPET_SMOOTHING);
        for (int i = 0; i < 6; ++i) {
            if (current_.legs[i].joints != puppetTarget_.legs[i].joints) current_.legs[i] = puppetTarget_.legs[i];
            for (int k = 0; k < 3; ++k)
                current_.legs[i].v[k] = lerpf(current_.legs[i].v[k], puppetTarget_.legs[i].v[k], PUPPET_SMOOTHING);
        }
    }

    // The validator's clamp sweep: every keyframe plus 32 evenly spaced times per segment.
    uint32_t sweepClampMask(const anim::Clip &clip) {
        float params[anim::PARAM_COUNT];
        anim::resolveParams(clip, nullptr, 0, params);
        uint32_t mask = 0;
        float angles[18];
        anim::Pose pose;
        for (int i = 0; i < clip.keyframeCount; ++i) {
            const float t0 = clip.keyframes[i].time;
            const float t1 = i + 1 < clip.keyframeCount ? clip.keyframes[i + 1].time : t0;
            const int samples = i + 1 < clip.keyframeCount ? 32 : 1;
            for (int s = 0; s < samples; ++s) {
                const float t = t0 + (t1 - t0) * (float)s / 32.0f;
                anim::evaluate(clip, params, t, *kin_, stance_, pose);
                mask |= anim::poseToAngles(pose, *kin_, stance_, angles);
            }
        }
        anim::evaluate(clip, params, clip.duration(), *kin_, stance_, pose);
        mask |= anim::poseToAngles(pose, *kin_, stance_, angles);
        return mask;
    }

    void publishStatus(bool changed) {
        const unsigned long now = millis();
        if (!changed && (player_->state() == anim::State::IDLE || now - lastStatusMs_ < STATUS_PERIOD_MS)) return;
        lastStatusMs_ = now;
        socket_message_AnimationStatus s = socket_message_AnimationStatus_init_zero;
        if (player_->clip()) strncpy(s.name, player_->clip()->name, sizeof(s.name) - 1);
        s.state = (socket_message_AnimationState)(int)player_->state();
        s.t = player_->t();
        s.clamped_mask = clampMask_;
        EventBus<socket_message_AnimationStatus>::publish(s);
    }
};
```

`millis()` is the inline helper `motion.h` defines from `esp_timer`; move that one-liner (and `micros()`) into `utils/timing.h` if it is not already there so both headers share it, rather than duplicating it.

The `AnimationState` cast relies on `anim::State` and `socket_message_AnimationState` sharing the values 0 to 4 in the same order; add a `static_assert` for each of the five pairs next to the enum in `animation_runner.h`.

- [ ] **Step 4: Motion service**

In `firmware/include/motion.h` (the working-tree version):

- add `#include <animation/animation_runner.h>`;
- in `begin()`, after the existing subscriptions, subscribe:

```cpp
        _animationSubHandle = EventBus<AnimationCommandMsg>::subscribe([&](AnimationCommandMsg const &c) {
            handleAnimationCommand(c);
        });
        _poseSubHandle = EventBus<PoseMsg>::subscribe([&](PoseMsg const &p) { _animation.setPuppet(p); });
        _animation.begin(kinematics, default_feet_pos);
```

- add the handler next to `handleInputMode`:

```cpp
    // A play from any active mode borrows ANIMATE and hands the mode back when the player finishes;
    // a play while already in ANIMATE (borrowed or sticky) chains. A stop only makes sense in ANIMATE.
    void handleAnimationCommand(AnimationCommandMsg const &c) {
        if (!c.play) {
            if (motionState == MOTION_STATE::ANIMATE) _animation.requestStop();
            return;
        }
        if (motionState != MOTION_STATE::ANIMATE) {
            if (!isActuatedMode(motionState)) return;
            _previousMode = motionState;
            _borrowedMode = true;
            EventBus<ModeMsg>::publish({MOTION_STATE::ANIMATE});
        }
        _animation.requestPlay(c);
    }
```

- in `handleAnimationCommand`, set `_expectingBorrow = true` immediately before the `EventBus<ModeMsg>::publish({MOTION_STATE::ANIMATE})` line;
- in `handleInputMode`, add at the top:

```cpp
        if (motionState == MOTION_STATE::ANIMATE && m.mode != MOTION_STATE::ANIMATE) _animation.reset();
        // The borrow's own mode publish arrives here too; any other mode change ends the borrow, so an
        // explicit ANIMATE from the app is sticky and an explicit STAND mid-play is final.
        if (_expectingBorrow) _expectingBorrow = false;
        else _borrowedMode = false;
```
- `isActuatedMode` gains `|| mode == MOTION_STATE::ANIMATE`;
- in `updateMotion`, add before the `WALK` case:

```cpp
            case MOTION_STATE::ANIMATE: {
                const bool finished = _animation.tick(dt, body_state, msgAngles.angles);
                if (finished && _borrowedMode) {
                    _borrowedMode = false;
                    EventBus<ModeMsg>::publish({_previousMode});
                }
                break;
            }
```

(`tick` already wrote the angles, so no IK call follows; `body_state` carries the absolute pose for telemetry.)

- members: `AnimationRunner _animation; EventBus<AnimationCommandMsg>::Handle _animationSubHandle; EventBus<PoseMsg>::Handle _poseSubHandle; MOTION_STATE _previousMode = MOTION_STATE::STAND; bool _borrowedMode = false; bool _expectingBorrow = false;`.

The command timeout: `resetCommandIfTimedOut` zeroes the STAND targets, which do not apply in ANIMATE; nothing to change.

- [ ] **Step 5: Handlers and bridge in `main.cpp`**

In `registerHandlers`, after the `ServoStateData` handler:

```cpp
    c.on<socket_message_AnimationPlay>([](const socket_message_AnimationPlay &p, int) {
        if (!anim::validName(p.name)) return;
        AnimationCommandMsg cmd{};
        cmd.play = true;
        strncpy(cmd.name, p.name, sizeof(cmd.name) - 1);
        cmd.paramCount = p.params_count < 10 ? p.params_count : 10;
        for (int i = 0; i < cmd.paramCount; ++i) {
            if (!std::isfinite(p.params[i].value)) return;
            cmd.params[i] = {(int)p.params[i].id, p.params[i].value};
        }
        EventBus<AnimationCommandMsg>::publish(cmd);
    });
    c.on<socket_message_AnimationStop>([](const socket_message_AnimationStop &, int) {
        AnimationCommandMsg cmd{};
        cmd.play = false;
        EventBus<AnimationCommandMsg>::publish(cmd);
    });
    c.on<socket_message_PoseData>([](const socket_message_PoseData &p, int) {
        if (p.legs_count != 0 && p.legs_count != 6) return;
        PoseMsg pose{};
        const float body[6] = {p.body.roll, p.body.pitch, p.body.yaw, p.body.x, p.body.y, p.body.z};
        for (int a = 0; a < 6; ++a) {
            if (!std::isfinite(body[a])) return;
            pose.body[a] = body[a];
        }
        for (int i = 0; i < (int)p.legs_count; ++i) {
            const animation_LegTarget &lt = p.legs[i];
            pose.joints[i] = lt.which_target == animation_LegTarget_joints_tag;
            const float *v = pose.joints[i] ? &lt.target.joints.coxa : &lt.target.foot.x;
            for (int k = 0; k < 3; ++k) {
                if (!std::isfinite(v[k])) return;
                pose.legs[i][k] = v[k];
            }
        }
        EventBus<PoseMsg>::publish(pose);
    });
```

(`&lt.target.joints.coxa` as a 3-float view relies on the nanopb struct laying `coxa, femur, tibia` out contiguously as floats, which it does; the same for `foot.x`.)

In `setupComm`, after `observeStatus<api_APStatus>();`: `observeStatus<socket_message_AnimationStatus>();`.

Add `#include <animation/animation.h>` to `main.cpp` for `anim::validName`.

- [ ] **Step 6: Host test of the runner's decisions**

The runner depends on FreeRTOS and PSRAM, so its logic is exercised where it lives, but the decisions Review Focus 1, 2 and 4 name are pinned on the host through the pieces that are host-clean: the `Player` (play during Playing, stop) is already covered; the status cadence is a pure function of times.
Add to `animation.h` (namespace `anim`) the tiny helper the runner uses:

```cpp
// Status is pushed on every state change, and otherwise at most once per period while not idle.
inline bool statusDue(bool changed, bool idle, unsigned long nowMs, unsigned long lastMs, unsigned long periodMs) {
    if (changed) return true;
    if (idle) return false;
    return nowMs - lastMs >= periodMs;
}
```

and use it in `publishStatus`.
Test in `test_animation.cpp`:

```cpp
void test_status_cadence_is_five_hertz_plus_every_change() {
    TEST_ASSERT_TRUE(anim::statusDue(true, true, 0, 0, 200));
    TEST_ASSERT_FALSE(anim::statusDue(false, true, 1000, 0, 200));
    TEST_ASSERT_FALSE(anim::statusDue(false, false, 150, 0, 200));
    TEST_ASSERT_TRUE(anim::statusDue(false, false, 200, 0, 200));
    TEST_ASSERT_TRUE(anim::statusDue(true, false, 150, 0, 200));
}
```

- [ ] **Step 7: Build and test**

`pio test -e native`: all pass.
`pio run -e esp32-wroom-camera`: SUCCESS.
Report the RAM and flash deltas against Task 2's build; the clip buffers must not appear in the static RAM figure (they are PSRAM heap).

- [ ] **Step 8: Commit**

```bash
git add firmware/include/animation/animation_store.h firmware/include/animation/animation_runner.h firmware/include/animation/animation.h firmware/include/message_types.h firmware/include/motion.h firmware/include/utils/timing.h firmware/src/main.cpp firmware/test/test_animation/test_animation.cpp
git commit -m "✨ Plays animations on the robot with an ANIMATE mode, puppeteering and status"
```

(Drop `utils/timing.h` from the list if it did not change.)

---

### Task 7: Chunked file transfer, validation and listing on the request channel

**Files:**
- Create: `firmware/include/file_transfer.h`
- Create: `firmware/test/test_file_transfer/test_file_transfer.cpp`
- Modify: `firmware/src/main.cpp` (correlation cases)

**Interfaces:**
- Produces: `file_transfer::CHUNK_MAX = 512`, `file_transfer::validPath(const char *rel)`, `int file_transfer::write(const char *full, uint32_t offset, uint32_t total, const uint8_t *data, size_t len)` returning an HTTP-style status, `int file_transfer::read(const char *full, uint32_t offset, uint32_t length, uint8_t *out, size_t &n, uint32_t &total)`; the five correlation cases.

- [ ] **Step 1: Write the failing tests**

`firmware/test/test_file_transfer/test_file_transfer.cpp`:

```cpp
// Host tests for the chunked file writer the app uses over BLE and serial, where there is no HTTP.

#include <unity.h>

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <file_transfer.h>

namespace {

std::string tmpFile() {
    static int n = 0;
    return ".pio/test_file_transfer_" + std::to_string(n++) + ".bin";
}

std::vector<uint8_t> contents(const std::string &path) {
    std::vector<uint8_t> out;
    FILE *f = fopen(path.c_str(), "rb");
    if (!f) return out;
    uint8_t buf[64];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0) out.insert(out.end(), buf, buf + n);
    fclose(f);
    return out;
}

}  // namespace

void setUp() {}
void tearDown() {}

void test_paths_must_be_absolute_inside_the_mount() {
    TEST_ASSERT_TRUE(file_transfer::validPath("/animations/wave.pb"));
    TEST_ASSERT_FALSE(file_transfer::validPath("animations/wave.pb"));
    TEST_ASSERT_FALSE(file_transfer::validPath("/animations/../config/x"));
    TEST_ASSERT_FALSE(file_transfer::validPath(""));
    TEST_ASSERT_FALSE(file_transfer::validPath(nullptr));
}

void test_chunks_assemble_the_file_in_order() {
    const std::string path = tmpFile();
    const uint8_t a[3] = {1, 2, 3}, b[2] = {4, 5};
    TEST_ASSERT_EQUAL(200, file_transfer::write(path.c_str(), 0, 5, a, 3));
    TEST_ASSERT_EQUAL(200, file_transfer::write(path.c_str(), 3, 5, b, 2));
    const std::vector<uint8_t> got = contents(path);
    TEST_ASSERT_EQUAL(5, got.size());
    TEST_ASSERT_EQUAL_UINT8_ARRAY(a, got.data(), 3);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(b, got.data() + 3, 2);
    remove(path.c_str());
}

void test_offset_zero_truncates_a_previous_file() {
    const std::string path = tmpFile();
    const uint8_t a[4] = {9, 9, 9, 9}, b[1] = {1};
    file_transfer::write(path.c_str(), 0, 4, a, 4);
    TEST_ASSERT_EQUAL(200, file_transfer::write(path.c_str(), 0, 1, b, 1));
    TEST_ASSERT_EQUAL(1, contents(path).size());
    remove(path.c_str());
}

void test_a_chunk_past_the_total_or_past_the_end_is_refused() {
    const std::string path = tmpFile();
    const uint8_t a[3] = {1, 2, 3};
    TEST_ASSERT_EQUAL(400, file_transfer::write(path.c_str(), 0, 2, a, 3));
    TEST_ASSERT_EQUAL(0, contents(path).size());
    TEST_ASSERT_EQUAL(200, file_transfer::write(path.c_str(), 0, 6, a, 3));
    TEST_ASSERT_EQUAL(400, file_transfer::write(path.c_str(), 4, 6, a, 2));  // gap after byte 3
    TEST_ASSERT_EQUAL(3, contents(path).size());
    static uint8_t big[file_transfer::CHUNK_MAX + 1];
    TEST_ASSERT_EQUAL(400, file_transfer::write(path.c_str(), 3, 1000, big, sizeof(big)));
    remove(path.c_str());
}

void test_read_returns_the_window_and_the_total() {
    const std::string path = tmpFile();
    uint8_t data[700];
    for (int i = 0; i < 700; ++i) data[i] = (uint8_t)i;
    file_transfer::write(path.c_str(), 0, 700, data, 512);
    file_transfer::write(path.c_str(), 512, 700, data + 512, 188);
    uint8_t out[512];
    size_t n = 0;
    uint32_t total = 0;
    TEST_ASSERT_EQUAL(200, file_transfer::read(path.c_str(), 512, 512, out, n, total));
    TEST_ASSERT_EQUAL(188, n);
    TEST_ASSERT_EQUAL_UINT32(700, total);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(data + 512, out, 188);
    TEST_ASSERT_EQUAL(404, file_transfer::read("/nonexistent/x.bin", 0, 10, out, n, total));
    TEST_ASSERT_EQUAL(400, file_transfer::read(path.c_str(), 0, 513, out, n, total));
    remove(path.c_str());
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_paths_must_be_absolute_inside_the_mount);
    RUN_TEST(test_chunks_assemble_the_file_in_order);
    RUN_TEST(test_offset_zero_truncates_a_previous_file);
    RUN_TEST(test_a_chunk_past_the_total_or_past_the_end_is_refused);
    RUN_TEST(test_read_returns_the_window_and_the_total);
    return UNITY_END();
}
```

- [ ] **Step 2: Run to verify failure**

`pio test -e native`: compile failure on the header.

- [ ] **Step 3: Write the header**

`firmware/include/file_transfer.h`:

```cpp
#pragma once

// Chunked file transfer for transports without HTTP: the app on its https origin only has BLE and
// Web Serial. Each chunk is a self-contained stdio operation, so a lost chunk leaves a file that is
// simply shorter than its total; the next write at the right offset continues it. Paths are checked
// by the caller against the mount root with validPath().

#include <sys/stat.h>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace file_transfer {

constexpr size_t CHUNK_MAX = 512;

inline bool validPath(const char *rel) { return rel && rel[0] == '/' && strstr(rel, "..") == nullptr; }

inline long fileSize(const char *full) {
    struct stat st;
    return stat(full, &st) == 0 ? (long)st.st_size : -1;
}

// 200 written; 400 the chunk does not fit the declared total, does not continue the file, or is
// too large; 500 the filesystem refused.
inline int write(const char *full, uint32_t offset, uint32_t total, const uint8_t *data, size_t len) {
    if (len > CHUNK_MAX || (uint64_t)offset + len > total) return 400;
    if (offset == 0) {
        FILE *f = fopen(full, "wb");
        if (!f) return 500;
        const bool ok = fwrite(data, 1, len, f) == len;
        fclose(f);
        return ok ? 200 : 500;
    }
    if (fileSize(full) != (long)offset) return 400;
    FILE *f = fopen(full, "ab");
    if (!f) return 500;
    const bool ok = fwrite(data, 1, len, f) == len;
    fclose(f);
    return ok ? 200 : 500;
}

// 200 with n bytes and the file's total; 404 no such file; 400 length over CHUNK_MAX.
inline int read(const char *full, uint32_t offset, uint32_t length, uint8_t *out, size_t &n, uint32_t &total) {
    n = 0;
    if (length > CHUNK_MAX) return 400;
    const long size = fileSize(full);
    if (size < 0) return 404;
    total = (uint32_t)size;
    FILE *f = fopen(full, "rb");
    if (!f) return 404;
    if (fseek(f, (long)offset, SEEK_SET) == 0) n = fread(out, 1, length, f);
    fclose(f);
    return 200;
}

}  // namespace file_transfer
```

- [ ] **Step 4: Correlation cases**

In `main.cpp`'s correlation handler `switch`, before `default:`:

```cpp
            case socket_message_CorrelationRequest_file_write_chunk_tag: {
                const api_FileWriteChunk &w = req.request.file_write_chunk;
                std::string full;
                res.which_response = socket_message_CorrelationResponse_empty_tag;
                res.status_code = fs_api::resolve(w.path, full)
                                      ? file_transfer::write(full.c_str(), w.offset, w.total_size, w.content.bytes,
                                                             w.content.size)
                                      : 400;
                break;
            }
            case socket_message_CorrelationRequest_file_read_chunk_tag: {
                const api_FileReadChunk &r = req.request.file_read_chunk;
                std::string full;
                auto &chunk = res.response.file_chunk;
                res.which_response = socket_message_CorrelationResponse_file_chunk_tag;
                size_t n = 0;
                res.status_code = fs_api::resolve(r.path, full)
                                      ? file_transfer::read(full.c_str(), r.offset, r.length, chunk.content.bytes, n,
                                                            chunk.total_size)
                                      : 400;
                chunk.content.size = n;
                break;
            }
            case socket_message_CorrelationRequest_file_delete_tag: {
                std::string full;
                res.which_response = socket_message_CorrelationResponse_empty_tag;
                res.status_code = fs_api::resolve(req.request.file_delete.path, full) && unlink(full.c_str()) == 0 ? 200 : 400;
                break;
            }
            case socket_message_CorrelationRequest_animation_validate_tag: {
                res.which_response = socket_message_CorrelationResponse_animation_report_tag;
                robot.validateAnimation(req.request.animation_validate.name, res.response.animation_report);
                if (!res.response.animation_report.ok) res.status_code = 422;
                break;
            }
            case socket_message_CorrelationRequest_animation_list_request_tag: {
                res.which_response = socket_message_CorrelationResponse_animation_list_tag;
                auto &list = res.response.animation_list;
                list.entries_count = 0;
                AnimationStore::list([&list](const char *name, uint32_t size) {
                    if (list.entries_count >= 32) return;
                    auto &e = list.entries[list.entries_count++];
                    strncpy(e.name, name, sizeof(e.name) - 1);
                    e.size = size;
                });
                break;
            }
```

`fs_api::resolve` already rejects `..` and requires a leading slash; `file_transfer::validPath` is the same rule for callers without the HTTP header (the test).
Add `#include <file_transfer.h>` and `#include <animation/animation_store.h>` to `main.cpp`.
`Hexapod` gains `void validateAnimation(const char *name, socket_message_AnimationReport &r) { _motionService.validateAnimation(name, r); }` and `MotionService` gains the one-line forward to `_animation.validate(name, r)`.

- [ ] **Step 5: Build and test**

`pio test -e native`: all pass in both test directories.
`pio run -e esp32-wroom-camera`: SUCCESS.

- [ ] **Step 6: Commit**

```bash
git add firmware/include/file_transfer.h firmware/test/test_file_transfer/test_file_transfer.cpp firmware/src/main.cpp firmware/include/hexapod.h firmware/include/motion.h
git commit -m "✨ Uploads, validates and lists animations over the request channel"
```

---

### Task 8: Bench tool over the serial link, docs, and the hardware checklist

**Files:**
- Create: `simulation/robot_animate.py`
- Modify: `simulation/README.md`, `CLAUDE.md`, `docs/animation.md`

**Interfaces:**
- Consumes: the serial framing `[uint16 LE length][payload]` of `SerialAdapter` (`firmware/src/communication/serial_adapter.cpp`), the generated `message_pb2` and `api_pb2`.
- Produces: `uv run python robot_animate.py --port COM5 <command>` with commands `list`, `upload <name>`, `validate <name>`, `play <name> [PARAM=VALUE ...]`, `stop`, `mode <name>`, `watch`.

- [ ] **Step 1: Write the tool**

`simulation/robot_animate.py`:

```python
"""Drives the robot's animation system over the native USB serial link, for bench tests before the web
app exists. The framing is SerialAdapter's: a little-endian uint16 length followed by one
socket_message.Message.

    uv run python robot_animate.py --port COM5 list
    uv run python robot_animate.py --port COM5 upload wave        # ../animations/wave.json -> /animations/wave.pb
    uv run python robot_animate.py --port COM5 validate wave
    uv run python robot_animate.py --port COM5 play wave SPEED=1.5
    uv run python robot_animate.py --port COM5 stop
    uv run python robot_animate.py --port COM5 mode ANIMATE       # sticky ANIMATE, e.g. before puppeteering
    uv run python robot_animate.py --port COM5 watch              # print AnimationStatus until Ctrl-C
"""
import argparse
import struct
import sys
import threading
import time
from pathlib import Path

import serial

from src.platform_shared import api_pb2, message_pb2
from src.robot.animation_files import load_json, to_proto

ROOT = Path(__file__).resolve().parents[1]
CHUNK = 512
REQUEST_TIMEOUT_S = 5.0
ANIMATION_STATUS_TAG = message_pb2.Message.DESCRIPTOR.fields_by_name["animation_status"].number


class Link:
    def __init__(self, port: str):
        self.ser = serial.Serial(port, 115200, timeout=0.05)
        self.rx = bytearray()
        self.responses: dict[int, message_pb2.CorrelationResponse] = {}
        self.next_id = 1
        self.on_status = None
        threading.Thread(target=self._reader, daemon=True).start()

    def send(self, msg: message_pb2.Message) -> None:
        payload = msg.SerializeToString()
        self.ser.write(struct.pack("<H", len(payload)) + payload)

    def request(self, fill) -> message_pb2.CorrelationResponse:
        msg = message_pb2.Message()
        msg.correlation_request.correlation_id = self.next_id
        self.next_id += 1
        fill(msg.correlation_request)
        self.send(msg)
        deadline = time.time() + REQUEST_TIMEOUT_S
        while time.time() < deadline:
            res = self.responses.pop(msg.correlation_request.correlation_id, None)
            if res is not None:
                return res
            time.sleep(0.01)
        raise SystemExit("no response from the robot (is the native USB port the one you opened?)")

    def subscribe(self, tag: int) -> None:
        msg = message_pb2.Message()
        msg.sub_notif.tag = tag
        self.send(msg)

    def _reader(self) -> None:
        while True:
            self.rx += self.ser.read(4096)
            while len(self.rx) >= 2:
                (length,) = struct.unpack_from("<H", self.rx, 0)
                if len(self.rx) < 2 + length:
                    break
                payload = bytes(self.rx[2:2 + length])
                del self.rx[:2 + length]
                msg = message_pb2.Message()
                try:
                    msg.ParseFromString(payload)
                except Exception:
                    continue
                kind = msg.WhichOneof("message")
                if kind == "correlation_response":
                    self.responses[msg.correlation_response.correlation_id] = msg.correlation_response
                elif kind == "animation_status" and self.on_status:
                    self.on_status(msg.animation_status)


def upload(link: Link, name: str) -> None:
    data = to_proto(load_json(ROOT / "animations" / f"{name}.json")).SerializeToString()
    path = f"/animations/{name}.pb"
    for offset in range(0, len(data), CHUNK):
        chunk = data[offset:offset + CHUNK]

        def fill(req, offset=offset, chunk=chunk):
            req.file_write_chunk.path = path
            req.file_write_chunk.offset = offset
            req.file_write_chunk.total_size = len(data)
            req.file_write_chunk.content = chunk

        res = link.request(fill)
        if res.status_code != 200:
            raise SystemExit(f"chunk at {offset} refused with {res.status_code}")
    print(f"uploaded {len(data)} bytes to {path}")
    validate(link, name)


def validate(link: Link, name: str) -> None:
    res = link.request(lambda req: setattr(req.animation_validate, "name", name))
    r = res.animation_report
    print(f"{name}: {'ok' if r.ok else 'INVALID ' + r.error}  clamped {r.clamped_mask:018b}")


def list_animations(link: Link) -> None:
    res = link.request(lambda req: req.animation_list_request.SetInParent())
    for e in res.animation_list.entries:
        print(f"{e.name:16s} {e.size} bytes")


def play(link: Link, name: str, params: list[str]) -> None:
    msg = message_pb2.Message()
    msg.animation_play.name = name
    for item in params:
        key, value = item.split("=")
        p = msg.animation_play.params.add()
        p.id = message_pb2.AnimationParam.DESCRIPTOR.fields_by_name["id"].enum_type.values_by_name[key].number
        p.value = float(value)
    link.send(msg)


def stop(link: Link) -> None:
    msg = message_pb2.Message()
    msg.animation_stop.SetInParent()
    link.send(msg)


def mode(link: Link, name: str) -> None:
    msg = message_pb2.Message()
    msg.mode.mode = message_pb2.ModesEnum.Value(name)
    link.send(msg)


def watch(link: Link) -> None:
    def show(s):
        state = message_pb2.AnimationState.Name(s.state)
        print(f"{s.name:16s} {state:12s} t={s.t:6.2f}  clamped {s.clamped_mask:018b}")

    link.on_status = show
    link.subscribe(ANIMATION_STATUS_TAG)
    print("watching AnimationStatus, Ctrl-C to stop")
    try:
        while True:
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True)
    sub = ap.add_subparsers(dest="command", required=True)
    sub.add_parser("list")
    sub.add_parser("upload").add_argument("name")
    sub.add_parser("validate").add_argument("name")
    p = sub.add_parser("play")
    p.add_argument("name")
    p.add_argument("params", nargs="*")
    sub.add_parser("stop")
    sub.add_parser("mode").add_argument("name")
    sub.add_parser("watch")
    args = ap.parse_args(argv)
    link = Link(args.port)
    time.sleep(0.3)
    if args.command == "list":
        list_animations(link)
    elif args.command == "upload":
        upload(link, args.name)
    elif args.command == "validate":
        validate(link, args.name)
    elif args.command == "play":
        play(link, args.name, args.params)
    elif args.command == "stop":
        stop(link)
    elif args.command == "mode":
        mode(link, args.name)
    else:
        watch(link)
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

Read `firmware/src/communication/serial_adapter.cpp` first and confirm the framing and the resync rule the reader mirrors; if the adapter expects anything beyond the two-byte length (a magic, a checksum), match it and say so in the report.
`message_pb2.Message` field access for `sub_notif` and the enum lookups must match the generated names; check `simulation/src/platform_shared/message_pb2.py` after `uv run python scripts/compile_protos.py`.

There is no automated test for this tool (it needs the robot); the acceptance list below is its test.

- [ ] **Step 2: Docs**

`simulation/README.md`, Animations section, add: `uv run python robot_animate.py --port <COMx> <command>` uploads, validates, plays, stops and watches animations over the robot's native USB port; run `--help` for the commands.

`CLAUDE.md`: add the same command line to the Simulation block; in "Further documentation" change the `docs/animation.md` line to say it describes the animation system as implemented (file format, evaluator rules, firmware mode, messages) and is kept current.

`docs/animation.md`: replace the shelved-system document with a current one, written from the spec's sections 1 to 3 and 6 as built (one sentence per line): what an animation file is and where the bundled ones live; the evaluator and player rules a porter must honour (copy the list from this plan's Global Constraints); the firmware mode, the borrow and hand-back, puppeteering, the messages and correlation requests with their tags, the file layout on LittleFS; the fixtures and how parity is tested on every platform; the bench tool.
Keep the old document's sign-convention warnings that still apply (positive body z crouches; positive x and y move the body toward negative x and y; +z lifts a foot) and drop everything about the stash.

- [ ] **Step 3: Hardware acceptance (owner, with the robot)**

Flash and upload the filesystem: `pio run -t upload` then `pio run -t uploadfs` (the seven bundled `.pb` files land in `/littlefs/animations`).
Then, on the native USB port:

1. `list` shows the seven bundled animations with sizes.
2. `validate wave` reports ok with an empty clamp mask; `validate play_dead` reports ok.
3. Put the robot in STAND from the controller or `mode STAND`, then `play wave`; in `watch`, the status runs ENTRY, PLAYING, EXIT, IDLE and the mode returns to STAND (the app or `watch` on the mode tag shows it).
   The robot leans left and back, raises the right front leg, flicks it twice, steps home.
4. `play crouch` while `wave` is playing: the second animation enters from wherever the first is, no jump.
5. `play wiggle` then `stop` mid-way: the robot eases home.
6. `play play_dead`: the robot lies down and holds; `stop` brings it back over 1.2 s.
7. `mode ANIMATE` then `play crouch`: after the play the robot stays in ANIMATE (sticky) at stance.
8. Edit `animations/wave.json` to an invalid file (a keyframe with three legs), `upload wave`: the upload succeeds but validation reports the error, and `play wave` does nothing while the robot stays in its mode; restore the file and upload again.
9. Servo speed: `play spooked SPEED=2`; the checker predicted a peak of 10 rad/s at speed 1, so at speed 2 the servos lag; confirm nothing worse than a softened hop.

Record the outcome of each step in `docs/superpowers/handoffs/2026-09-30-animation-firmware-acceptance.md`, including any step the robot fails, so plan 3 starts from measured behaviour.

- [ ] **Step 4: Commit**

```bash
git add simulation/robot_animate.py simulation/README.md CLAUDE.md docs/animation.md
git commit -m "✨ Adds a serial bench tool for animations and documents the system as built"
```

---

---

### Task 9: Ride height, stop on control loss, observable mode kinds, tick-cost log

Added after the whole-branch review from two owner decisions (2026-09-30) and two residuals of the final fix wave.
The spec commit `75e1912` carries the ride-height and control-loss rules.

**Files:**
- Modify: `platform_shared/animation.proto` (`optional float ride_height = 11`)
- Modify: `simulation/src/robot/animation.py`, `animation_files.py`, `simulation/test_animation.py` (field, validation, conversion)
- Modify: `simulation/sim_sandbox.py` (base height in Animate mode), `simulation/test_animation_library.py` (slider-extreme test)
- Modify: `animations/crouch.json`, `play_dead.json`, `stretch.json`, `spooked.json`, `body_roll_test.json` (`"rideHeight": 0`); regenerate nothing (the fixtures do not use the field)
- Modify: `firmware/include/animation/animation.h` (`Clip::hasRideHeight`, `rideHeight`, validation), `animation_codec.h`
- Modify: `firmware/include/animation/animation_runner.h` (base height, `controlLost()`, INFO tick-cost log)
- Modify: `firmware/include/message_types.h` (`ModeMsgKind`), `firmware/include/animation/mode_arbiter.h` (input takes the kind), `firmware/include/motion.h`, `firmware/include/hexapod.h`
- Modify: `firmware/include/communication/comm_base.hpp` (`hasClient()`, `onClientGone`), `websocket.h/.cpp`, `ble.h/.cpp`, `serial_adapter.h/.cpp`, `firmware/src/main.cpp` (bridge and control-loss wiring), `firmware/src/communication/espnow_adapter.cpp` (unchanged call sites compile)
- Test: `firmware/test/test_animation/test_animation.cpp`, `firmware/test/test_mode_arbiter/test_mode_arbiter.cpp`
- Modify: `docs/animation.md`

**Interfaces:**
- Produces: `Animation.ride_height` (optional, mm); `anim::Clip::hasRideHeight`, `rideHeight`; `AnimationRunner::tick(float dt, float sliderZm, BodyStateMsg&, float angles[18])`, `AnimationRunner::controlLost()`; `enum class ModeMsgKind { REQUEST, BORROW, HANDBACK, APPLIED }` replacing the two bools in `ModeMsg` (`{MOTION_STATE::X}` initialisers keep compiling with `kind = REQUEST`); `CommAdapterBase::hasClient() const` (virtual) and `onClientGone(std::function<void()>)`; `Hexapod::animationControlLost()`.

- [ ] **Step 1: Schema and reference**

`animation.proto`: after `params`, `optional float ride_height = 11;  // mm, body z base while playing; absent = the current ride-height slider`.
Python: `Animation.ride_height: float | None = None`; `from_proto` reads it only when `msg.HasField("ride_height")`; `to_proto` sets it only when not None; `validate` adds "ride_height must be finite" when present; tests: round trip with and without the field, the finiteness rule.
Sandbox: in `_apply_animation`, before `pose_to_angles`, add the base to `pose.body[an.BodyAxis.Z]`: the animation's `ride_height` when set, else `self._height_to_zm(self.v("height"))`; the same base is subtracted from `player.last_pose` on play so the captured live pose is an offset; state the rule in a comment.
Library: add `"rideHeight": 0` to crouch, play_dead, stretch, spooked and body_roll_test.
`test_animation_library.py`: for every animation without `ride_height`, evaluate every keyframe and midpoint with the body z offset shifted by the slider extremes (`-50` and `+50` mm, the STAND slider range `c.h * 50`) and assert the clamp mask is 0; narrow or set `rideHeight` on any file that fails, and report it.

- [ ] **Step 2: Firmware clip and runner base height**

`Clip` gains `bool hasRideHeight = false; float rideHeight = 0.0f;`; codec copies from `m.has_ride_height`; `validate` rejects a non-finite value.
`AnimationRunner::tick` takes the slider `sliderZm` (the STAND target `target_body_state.zm`).
Keep `baseZm_`, lerped every tick with the STAND smoothing factor toward the desired base: the clip's `rideHeight` while the player is not idle and the clip has one, else `sliderZm`.
Apply it after evaluation: `body.zm = current_.body[Z] + baseZm_` (that is, `bodyState` builds from the offsets and the base is added to `zm` before IK and before the joint overrides).
`enter()` captures the live pose with `live.zm - baseZm_` so the STAND height is not treated as an offset; initialise `baseZm_` to the slider on entry.
Puppeteer poses use the slider base.
Document in the runner header and in `docs/animation.md` (replace the neutral-stance wording written in the fix wave with the spec's ride-height paragraph).

- [ ] **Step 3: Mode message kinds and the observable bridge**

`ModeMsg { MOTION_STATE mode; ModeMsgKind kind = ModeMsgKind::REQUEST; }`; update the arbiter input to take the kind (borrow = BORROW, handback = HANDBACK) and every publish site (`motion.h` borrow, hand-back and restate; `begin()`; `main.cpp`; `espnow_adapter.cpp` untouched but must compile).
`handleInputMode` returns at once for `APPLIED`.
After executing an APPLY or RESTATE that came from a BORROW or HANDBACK, publish `{motionState, ModeMsgKind::APPLIED}`.
The bridge in `main.cpp` emits only `REQUEST` and `APPLIED` messages.
Arbiter tests: add a case that an APPLIED message is not passed to `decideMode` (test the `MotionService` guard by asserting `decideMode` is never given kind APPLIED: make `decideMode` return IGNORE for it and test that), and keep every existing case green with the kind mapping.

- [ ] **Step 4: Stop on control loss**

`CommAdapterBase`: `virtual bool hasClient() const = 0;` and `void onClientGone(std::function<void()> cb)`, called at the end of `removeClient`.
Websocket: true while any socket is open (track the count in `onWsClose` and the open path); BLE: `_deviceConnected`; serial: `hostPresent_`.
`main.cpp`: after the bridges, register on every adapter `onClientGone([] { if (!anyClientConnected()) robot.animationControlLost(); })` with `anyClientConnected()` beside `anyoneListening()`.
`Hexapod::animationControlLost()` forwards to `MotionService::animationControlLost()`, which calls `_animation.controlLost()` when `motionState == MOTION_STATE::ANIMATE`; the runner's `controlLost()` is `requestStop()` plus a log line.
The borrowed hand-back follows from the existing invariant; a sticky ANIMATE stays in ANIMATE at stance.
Host test: none possible for the adapters; add an arbiter-independent test in `test_animation.cpp` that a `Player` in HOLD goes to EXIT on `stop()` (already covered) is enough, and write the control-loss step into the acceptance checklist: disconnect the bench tool mid-`wiggle`, the robot eases home.

- [ ] **Step 5: Tick-cost log at INFO while animating**

Replace the compiled-out `ESP_LOGD` with `ESP_LOGI` emitted at most every 5 s and only while the player is not idle; the line names the maximum tick cost in microseconds.

- [ ] **Step 5b: Bench tool interactive shell and graceful leave from any posed state**

Added from the fix-wave re-review.
`robot_animate.py` gains a `shell` command that opens the port once and reads commands (`list`, `upload`, `validate`, `play`, `stop`, `mode`, `watch`, `quit`) from stdin, so a looping or holding animation can be stopped and a play can be chained while the port is held; `follow` returns cleanly on Ctrl-C (no traceback) and stops following on the first `ANIM_HOLD` status or after two seconds of a looping `ANIM_PLAYING` with no state change, printing why.
Acceptance steps 4, 5 and 6 in `docs/animation.md` use the shell.
`AnimationRunner::busy()` replaces the `playerBusy` input: true when the player is not idle, a play is pending, or the current pose is not at stance (any joint-mode leg, or a body or foot offset above 1 mm or 0.01 rad).
A graceful leave while the player is idle eases the pose home through the puppet path with a stance target instead of `requestStop()`; the hand-back invariant uses `!busy()`.
`decideMode` also returns IGNORE for a hand-back while busy, which closes the deferred chained-play window because the control task re-requests once the worker clears `_handbackSent`.
Arbiter tests: a sticky ANIMATE with a posed joint leg then explicit STAND gives GRACEFUL_LEAVE; a hand-back while busy is ignored.

- [ ] **Step 6: Verify, document, commit**

`pio test -e native` all pass; `pio run -e esp32-wroom-camera` SUCCESS (with the isolated package dirs if the shared framework is still replaced; report RAM and flash); `uv run pytest -q` and `uv run python check_animation.py` from `simulation/` pass; `pnpm proto && pnpm check` from `app/` still pass (the new optional field must not break the generated TypeScript).
`docs/animation.md`: the ride-height rule, the control-loss rule, the `ModeMsgKind` semantics and which kinds the app sees, the tick-cost log, and the new acceptance steps (control loss mid-wiggle; play_dead at a tall slider height returning to that height on exit).
Commits, one line each: `✨ Lets an animation fix its ride height or follow the slider`, `✨ Stops an animation when the last client is gone`, `🐛 Reports only applied mode changes to clients`, `🩹 Logs the animation tick cost at INFO while playing`.

## Self-review notes

- Spec coverage: section 2 (Tasks 3 to 5), section 3 mode and messages (Task 6), section 3 requests and storage (Task 7 and the store in Task 6), section 6 parity and firmware tests (Tasks 2, 4, 5, 6, 7), hardware acceptance (Task 8).
  The app side is plan 3.
- Review Focus 1 is pinned by the runner refusing a failed load (Task 6) and acceptance step 8; 2 by the double buffer and the critical section (Task 6) and the chained-play fixture trace (Task 5); 3 by `test_a_chunk_past_the_total_or_past_the_end_is_refused` and `test_paths_must_be_absolute_inside_the_mount`; 4 by `test_status_cadence_is_five_hertz_plus_every_change`; 5 by `test_player_matches_every_fixture_trace` over every step.
- Known deviation recorded: the spec's section 2 numbering of the evaluate steps lags the reference; this plan follows the reference and Task 8's rewrite of `docs/animation.md` states the order as built.
- The `ModesEnum.ANIMATE` value 6 changes nothing existing, but the app's `MotionModes` mirror does not yet know it; plan 3 adds it.
  Until then the app cannot select ANIMATE, which is fine because plan 3 is the first app consumer.
