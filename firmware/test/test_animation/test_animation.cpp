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
