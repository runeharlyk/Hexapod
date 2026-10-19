// Host test that the exported policy header is the one PolicyRunner can drive. policy_runner.h itself needs the
// ESP-IDF headers, so the layout it depends on lives in policy/policy_layout.h, whose static_asserts already fail
// the build for a header of another shape; the tests below add what a compiler cannot see: the header's own
// description of itself, and its golden vector.

#include <unity.h>

#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <policy/policy_layout.h>

namespace hp = hexapod_policy;

namespace {

std::string headerLine(int index) {
    std::ifstream file("firmware/include/policy/hexapod_policy.h");
    TEST_ASSERT_TRUE_MESSAGE(file.good(), "run from the repository root");
    std::string line;
    for (int i = 0; i <= index; ++i) std::getline(file, line);
    return line;
}

} // namespace

void test_header_golden_vector_still_holds() { TEST_ASSERT_LESS_THAN_FLOAT(1e-4f, hp::selfCheck()); }

// The first comment line is `// run: <name> mode: <mode> hash: <hash>`; the mode it names must be the one the
// constants were generated for, so a hand-edited or stale line cannot misreport the deployed policy.
void test_first_comment_names_the_mode_the_constants_belong_to() {
    const std::string runLine = headerLine(1);
    char run[128], mode[64], hash[64];
    TEST_ASSERT_EQUAL_INT_MESSAGE(
        3, std::sscanf(runLine.c_str(), "// run: %127s mode: %63s hash: %63s", run, mode, hash), runLine.c_str());

    const std::string dimsLine = headerLine(3);
    char dimsMode[64];
    int obsDim, actDim;
    TEST_ASSERT_EQUAL_INT_MESSAGE(
        3, std::sscanf(dimsLine.c_str(), "// control_mode=%63s obs_dim=%d act_dim=%d", dimsMode, &obsDim, &actDim),
        dimsLine.c_str());
    TEST_ASSERT_EQUAL_STRING(dimsMode, mode);
    TEST_ASSERT_EQUAL_INT(hp::OBS_DIM, obsDim);
    TEST_ASSERT_EQUAL_INT(hp::ACT_DIM, actDim);
    TEST_ASSERT_EQUAL_INT(strcmp(mode, "residual_gait") == 0 ? 24 : 18, hp::ACT_DIM);
}

// The stick is converted to the policy's command frame (forward, rightward, counter-clockwise), which is where
// WALK_NN's command meaning lives: stick right drives right, and right stick right turns clockwise, a negative yaw.
void test_stick_forward_is_forward_at_the_trained_maximum() {
    const policy_layout::PolicyCommand c = policy_layout::stickToPolicyCommand(0.0f, 1.0f, 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(hp::CMD_VX_MAX, c.vx);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, c.vy);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, c.yawRate);
}

void test_stick_backward_reaches_the_asymmetric_backward_limit() {
    const policy_layout::PolicyCommand c = policy_layout::stickToPolicyCommand(0.0f, -1.0f, 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(hp::CMD_VX_MIN, c.vx);
    TEST_ASSERT_TRUE(c.vx < 0.0f);
}

void test_stick_right_is_rightward_velocity() {
    const policy_layout::PolicyCommand c = policy_layout::stickToPolicyCommand(1.0f, 0.0f, 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(hp::CMD_VY_MAX, c.vy);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, c.vx);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, c.yawRate);
}

void test_right_stick_right_is_clockwise_so_negative_counter_clockwise_yaw() {
    const policy_layout::PolicyCommand c = policy_layout::stickToPolicyCommand(0.0f, 0.0f, 1.0f);
    TEST_ASSERT_EQUAL_FLOAT(-hp::CMD_YAW_MAX, c.yawRate);
    TEST_ASSERT_EQUAL_FLOAT(hp::CMD_YAW_MAX, policy_layout::stickToPolicyCommand(0.0f, 0.0f, -1.0f).yawRate);
}

void test_stick_scales_linearly_and_the_axes_do_not_mix() {
    const policy_layout::PolicyCommand c = policy_layout::stickToPolicyCommand(0.5f, 0.5f, 0.5f);
    TEST_ASSERT_EQUAL_FLOAT(0.5f * hp::CMD_VX_MAX, c.vx);
    TEST_ASSERT_EQUAL_FLOAT(0.5f * hp::CMD_VY_MAX, c.vy);
    TEST_ASSERT_EQUAL_FLOAT(-0.5f * hp::CMD_YAW_MAX, c.yawRate);
    const policy_layout::PolicyCommand back = policy_layout::stickToPolicyCommand(-0.5f, -0.5f, -0.5f);
    TEST_ASSERT_EQUAL_FLOAT(0.5f * hp::CMD_VX_MIN, back.vx);
    TEST_ASSERT_EQUAL_FLOAT(-0.5f * hp::CMD_VY_MAX, back.vy);
    TEST_ASSERT_EQUAL_FLOAT(0.5f * hp::CMD_YAW_MAX, back.yawRate);
}

void test_released_stick_commands_zero() {
    const policy_layout::PolicyCommand c = policy_layout::stickToPolicyCommand(0.0f, 0.0f, 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, c.vx);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, c.vy);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, c.yawRate);
}

void setUp() {}
void tearDown() {}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_header_golden_vector_still_holds);
    RUN_TEST(test_first_comment_names_the_mode_the_constants_belong_to);
    RUN_TEST(test_stick_forward_is_forward_at_the_trained_maximum);
    RUN_TEST(test_stick_backward_reaches_the_asymmetric_backward_limit);
    RUN_TEST(test_stick_right_is_rightward_velocity);
    RUN_TEST(test_right_stick_right_is_clockwise_so_negative_counter_clockwise_yaw);
    RUN_TEST(test_stick_scales_linearly_and_the_axes_do_not_mix);
    RUN_TEST(test_released_stick_commands_zero);
    return UNITY_END();
}
