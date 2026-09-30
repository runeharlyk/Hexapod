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

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_fixture_binaries_decode_with_nanopb);
    RUN_TEST(test_from_proto_copies_every_fixture_and_validates);
    RUN_TEST(test_validate_reports_each_structural_rule);
    return UNITY_END();
}
