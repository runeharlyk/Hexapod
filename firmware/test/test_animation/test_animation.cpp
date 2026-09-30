// Host-side tests for the animation port. The parity fixtures under animations/fixtures/ are
// generated from the Python reference; matching them is what makes the robot play what the editor
// and the simulation showed.

#include <unity.h>

#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include <pb_decode.h>
#include <animation.pb.h>
#include <animation/animation.h>
#include <animation/animation_codec.h>

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
    TEST_ASSERT_EQUAL(anim::BODY_ROLL, clip.params[0].id);
    TEST_ASSERT_EQUAL(anim::FOOT_LIFT, clip.params[1].id);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.0f, clip.params[1].max);
    decodeFixture("fx_overlay", a);
    anim::fromProto(a, clip);
    TEST_ASSERT_TRUE(clip.loop);
    TEST_ASSERT_EQUAL(anim::CHANNEL_BODY, clip.overlays[0].kind);
    TEST_ASSERT_EQUAL(0, clip.overlays[0].channel);
    TEST_ASSERT_EQUAL(anim::CHANNEL_FOOT, clip.overlays[2].kind);
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
    c = twoKeyframes(); c.keyframeCount = 0; expectError(c, "keyframe");
    c = twoKeyframes(); c.keyframes[0].time = 0.1f; expectError(c, "time 0");
    c = twoKeyframes(); c.keyframes[1].time = 0.0f; expectError(c, "increase");
    c = twoKeyframes(); c.keyframes[1].time = NAN; expectError(c, "finite");
    c = twoKeyframes(); c.keyframes[1].legCount = 3; expectError(c, "0 or 6");
    c = twoKeyframes(); c.keyframes[1].ease = 4; expectError(c, "ease");
    c = twoKeyframes(); c.loop = true; c.holdEnd = true; expectError(c, "loop");
    c = twoKeyframes(); c.entryTime = INFINITY; expectError(c, "finite");
    c = twoKeyframes(); c.overlayCount = 1; c.overlays[0].end = 1; expectError(c, "channel");
    c = twoKeyframes(); c.overlayCount = 1; c.overlays[0] = {anim::CHANNEL_BODY, 6, 1, 1, 0, 0, 1}; expectError(c, "body_axis");
    c = twoKeyframes(); c.overlayCount = 1; c.overlays[0] = {anim::CHANNEL_FOOT, 18, 1, 1, 0, 0, 1}; expectError(c, "foot_channel");
    c = twoKeyframes(); c.overlayCount = 1; c.overlays[0] = {anim::CHANNEL_BODY, 0, 1, 1, 0, 0.5f, 0.5f}; expectError(c, "start");
    c = twoKeyframes(); c.overlayCount = 1; c.overlays[0] = {anim::CHANNEL_BODY, 0, 1, 1, 0, 0, 1.5f}; expectError(c, "end");
    c = twoKeyframes(); c.overlayCount = 1; c.overlays[0] = {anim::CHANNEL_BODY, 0, NAN, 1, 0, 0, 1}; expectError(c, "finite");
    c = twoKeyframes(); c.paramCount = 2; c.params[0] = {anim::SPEED, 0.5f, 1, 2}; c.params[1] = {anim::SPEED, 0.5f, 1, 2}; expectError(c, "unique");
    c = twoKeyframes(); c.paramCount = 1; c.params[0] = {anim::BODY_Z, 0.5f, 3, 2}; expectError(c, "min <= default_value <= max");
    c = twoKeyframes(); c.paramCount = 1; c.params[0] = {anim::SPEED, 0.0f, 1, 2}; expectError(c, "SPEED");
    c = twoKeyframes(); c.paramCount = 1; c.params[0] = {anim::REPEAT, 0.0f, 1, 2}; expectError(c, "REPEAT");
    c = twoKeyframes(); c.paramCount = 1; c.params[0] = {10, 0.5f, 1, 2}; expectError(c, "param id");
    c = twoKeyframes(); c.keyframeCount = 33; expectError(c, "32");
    c = twoKeyframes(); c.overlayCount = 9; expectError(c, "8");
    c = twoKeyframes(); c.paramCount = 11; expectError(c, "10");
}

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
    TEST_ASSERT_TRUE(f.evaluate.size() >= 59);
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
    TEST_ASSERT_TRUE(checked >= 59);
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

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_fixture_binaries_decode_with_nanopb);
    RUN_TEST(test_from_proto_copies_every_fixture_and_validates);
    RUN_TEST(test_validate_reports_each_structural_rule);
    RUN_TEST(test_evaluate_matches_every_fixture_sample);
    RUN_TEST(test_ease_curves_match_the_reference);
    RUN_TEST(test_resolve_params_defaults_clamps_and_ignores_undeclared);
    RUN_TEST(test_pose_to_angles_clamps_overrides_and_flags_unreachable_feet);
    RUN_TEST(test_capture_pose_reads_offsets_from_a_body_state);
    RUN_TEST(test_player_matches_every_fixture_trace);
    RUN_TEST(test_stop_before_the_first_update_blends_from_the_live_pose);
    return UNITY_END();
}
