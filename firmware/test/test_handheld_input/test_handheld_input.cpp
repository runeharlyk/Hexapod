// The handheld controller's frames as the ESP-NOW adapter turns them into commands
// (communication/handheld_input.h): the sticks and the height trim, whether the frame moves the robot, and the mode its
// mode button asks for.

#include <unity.h>
#include <initializer_list>
#include <communication/handheld_input.h>

namespace {

controller_packet_t packet(int16_t lx, int16_t ly, int16_t rx, int16_t ry, uint8_t buttons = 0) {
    return {CONTROLLER_PACKET_VERSION, buttons, lx, ly, rx, ry, 0};
}

} // namespace

void setUp() {}
void tearDown() {}

void test_the_sticks_are_the_commands_axes_at_full_scale() {
    float height = 0;
    const CommandMsg c = handheldCommand(packet(500, -1000, 250, 1000), false, height);
    TEST_ASSERT_EQUAL_FLOAT(0.5f, c.lx);
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, c.ly);
    TEST_ASSERT_EQUAL_FLOAT(0.25f, c.rx);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, c.ry);
    TEST_ASSERT_EQUAL_FLOAT(0, c.s + c.s1 + c.fd + c.stride);
}

// Any axis off centre by HANDHELD_AXIS_THRESHOLD moves the robot, each one alone.
void test_a_frame_with_any_axis_off_centre_is_the_handheld_moving() {
    float height = 0;
    constexpr int16_t t = HANDHELD_AXIS_THRESHOLD;
    for (const controller_packet_t p :
         {packet(t, 0, 0, 0), packet(0, -t, 0, 0), packet(0, 0, t, 0), packet(0, 0, 0, -t)})
        TEST_ASSERT_EQUAL(CommandOrigin::HANDHELD_MOVING, handheldCommand(p, false, height).origin);
}

// A thumbstick that drifts just past the controller's own deadzone sends small axes at 50 Hz: under the threshold
// they are centred, so the frame neither walks, nor tilts, nor trims, nor keeps the handheld's window open.
void test_an_axis_drifting_under_the_threshold_is_centred() {
    float height = 0;
    constexpr int16_t d = HANDHELD_AXIS_THRESHOLD - 1;
    const CommandMsg drift = handheldCommand(packet(d, -d, d, -d), false, height);
    TEST_ASSERT_EQUAL(CommandOrigin::HANDHELD, drift.origin);
    TEST_ASSERT_EQUAL_FLOAT(0, drift.lx);
    TEST_ASSERT_EQUAL_FLOAT(0, drift.ly);
    TEST_ASSERT_EQUAL_FLOAT(0, drift.rx);
    TEST_ASSERT_EQUAL_FLOAT(0, drift.ry);
    const CommandMsg trim = handheldCommand(packet(0, 0, 0, d, BTN_RIGHT), true, height);
    TEST_ASSERT_EQUAL(CommandOrigin::HANDHELD, trim.origin);
    TEST_ASSERT_EQUAL_FLOAT(0, trim.h);
}

// The buttons travel as modes and gaits of their own, so a frame with only a button held is the handheld at rest.
void test_a_centred_frame_is_the_handheld_at_rest_whatever_its_buttons() {
    float height = 0;
    TEST_ASSERT_EQUAL(CommandOrigin::HANDHELD, handheldCommand(packet(0, 0, 0, 0), false, height).origin);
    TEST_ASSERT_EQUAL(CommandOrigin::HANDHELD,
                      handheldCommand(packet(0, 0, 0, 0, BTN_LEFT | BTN_RIGHT | BTN_A), true, height).origin);
}

// In STAND the right stick, with its button held, sets the body height every later command carries; the command's ry
// is then no tilt. The trim moves the robot though the command's sticks are centred.
void test_the_height_trim_in_a_stand_is_kept_and_moves_the_robot() {
    float height = 0;
    const CommandMsg trim = handheldCommand(packet(0, 0, 0, -400, BTN_RIGHT), true, height);
    TEST_ASSERT_EQUAL_FLOAT(0, trim.ry);
    TEST_ASSERT_EQUAL_FLOAT(-0.4f, trim.h);
    TEST_ASSERT_EQUAL(CommandOrigin::HANDHELD_MOVING, trim.origin);
    const CommandMsg after = handheldCommand(packet(0, 0, 0, 0), true, height);
    TEST_ASSERT_EQUAL_FLOAT(-0.4f, after.h);
    TEST_ASSERT_EQUAL(CommandOrigin::HANDHELD, after.origin);
    // Outside a stand the button is the gait's, and the right stick tilts the body.
    const CommandMsg walking = handheldCommand(packet(0, 0, 0, 300, BTN_RIGHT), false, height);
    TEST_ASSERT_EQUAL_FLOAT(0.3f, walking.ry);
    TEST_ASSERT_EQUAL_FLOAT(-0.4f, walking.h);
}

// Each press asks for the next mode of the cycle from the one the robot reports, WALK wrapping to DEACTIVATED.
void test_the_mode_button_steps_through_the_cycle() {
    TEST_ASSERT_EQUAL(MOTION_STATE::IDLE, nextHandheldMode(MOTION_STATE::DEACTIVATED));
    TEST_ASSERT_EQUAL(MOTION_STATE::STAND, nextHandheldMode(MOTION_STATE::IDLE));
    TEST_ASSERT_EQUAL(MOTION_STATE::WALK, nextHandheldMode(MOTION_STATE::STAND));
    TEST_ASSERT_EQUAL(MOTION_STATE::DEACTIVATED, nextHandheldMode(MOTION_STATE::WALK));
}

// ANIMATE has no place in the cycle: the robot is in it for an app's play or for the ease out of IDLE, both on their
// way to a stand or a walk. A press there asks for STAND, the ease's own target when the handheld began it from IDLE
// and a graceful leave of a play, so the legs ease home; never the IDLE the cycle's start would give, which stops the
// robot where it is, half-eased, until a further press.
void test_the_mode_button_in_animate_asks_for_the_stand() {
    TEST_ASSERT_EQUAL(MOTION_STATE::STAND, nextHandheldMode(MOTION_STATE::ANIMATE));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_the_sticks_are_the_commands_axes_at_full_scale);
    RUN_TEST(test_a_frame_with_any_axis_off_centre_is_the_handheld_moving);
    RUN_TEST(test_an_axis_drifting_under_the_threshold_is_centred);
    RUN_TEST(test_a_centred_frame_is_the_handheld_at_rest_whatever_its_buttons);
    RUN_TEST(test_the_height_trim_in_a_stand_is_kept_and_moves_the_robot);
    RUN_TEST(test_the_mode_button_steps_through_the_cycle);
    RUN_TEST(test_the_mode_button_in_animate_asks_for_the_stand);
    return UNITY_END();
}
