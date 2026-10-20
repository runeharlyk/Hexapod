// The mode as it crosses the wire (firmware/include/motion_state_wire.h): MOTION_STATE carries ModesEnum's values, the
// value POSE had is reserved, and a value the robot has no mode for is refused rather than cast.

#include <unity.h>
#include <motion_state_wire.h>

void setUp() {}
void tearDown() {}

void test_every_mode_crosses_the_wire_as_its_own_value() {
    const struct {
        socket_message_ModesEnum wire;
        MOTION_STATE mode;
    } pairs[] = {{socket_message_ModesEnum_DEACTIVATED, MOTION_STATE::DEACTIVATED},
                 {socket_message_ModesEnum_IDLE, MOTION_STATE::IDLE},
                 {socket_message_ModesEnum_STAND, MOTION_STATE::STAND},
                 {socket_message_ModesEnum_WALK, MOTION_STATE::WALK},
                 {socket_message_ModesEnum_WALK_NN, MOTION_STATE::WALK_NN},
                 {socket_message_ModesEnum_ANIMATE, MOTION_STATE::ANIMATE}};
    for (const auto &p : pairs) {
        const std::optional<MOTION_STATE> mode = motionStateFromWire(p.wire);
        TEST_ASSERT_TRUE(mode.has_value());
        TEST_ASSERT_EQUAL(p.mode, *mode);
        TEST_ASSERT_EQUAL_INT(p.wire, static_cast<int>(p.mode));
    }
}

// POSE was 2. The proto reserves it, so no mode takes its place, and a client that still sends it moves nothing.
void test_the_value_pose_had_is_no_mode() {
    TEST_ASSERT_FALSE(motionStateFromWire(2).has_value());
    TEST_ASSERT_EQUAL_INT(socket_message_ModesEnum_IDLE + 2, socket_message_ModesEnum_STAND);
}

void test_a_value_past_the_modes_is_no_mode() {
    TEST_ASSERT_FALSE(motionStateFromWire(-1).has_value());
    TEST_ASSERT_FALSE(motionStateFromWire(socket_message_ModesEnum_ANIMATE + 1).has_value());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_every_mode_crosses_the_wire_as_its_own_value);
    RUN_TEST(test_the_value_pose_had_is_no_mode);
    RUN_TEST(test_a_value_past_the_modes_is_no_mode);
    return UNITY_END();
}
