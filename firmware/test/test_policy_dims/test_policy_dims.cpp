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

void setUp() {}
void tearDown() {}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_header_golden_vector_still_holds);
    RUN_TEST(test_first_comment_names_the_mode_the_constants_belong_to);
    return UNITY_END();
}
