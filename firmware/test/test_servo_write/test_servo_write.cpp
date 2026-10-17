// Host tests of what the servo controller reports of each tick's write (TickSample's pwm, servo_write_us and the two
// boards' results), against the I2C register fake: a write that both boards take, a tick that writes nothing, a board
// that stops answering and is then backed off, one that lost power and is started again, and the time the bus took.
// The include path puts the host stand-in of the controller first, so the robot's own is included by its path.

#include <unity.h>

#include <memory>
#include "../../include/peripherals/servo_controller.h"

namespace {

constexpr uint8_t RIGHT_BOARD = 0x41;
constexpr uint8_t LEFT_BOARD = 0x40;
constexpr uint8_t LED0_ON_L = 0x06;
constexpr uint8_t PRE_SCALE = 0xFE;
constexpr uint8_t MODE1 = 0x00;
constexpr int64_t TICK_US = 5000;
// The PCA9685 channel each servo drives, in servo order (servo_controller.h's cfg).
constexpr uint8_t CHANNEL[NUM_SERVO] = {8, 9, 10, 2, 3, 4, 5, 6, 7, 8, 9, 10, 2, 3, 4, 5, 6, 7};

I2CBus& bus() { return I2CBus::instance(); }

void powerOnChip(uint8_t addr) {
    auto& chip = bus().registers[addr];
    chip = {};
    chip[MODE1] = 0x11;
    chip[PRE_SCALE] = 0x1E;
    for (int ch = 0; ch < 16; ch++) chip[LED0_ON_L + 4 * ch + 3] = 0x10;
}

uint16_t offCount(uint8_t addr, int channel) {
    const auto& chip = bus().registers[addr];
    return chip[LED0_ON_L + 4 * channel + 2] | (chip[LED0_ON_L + 4 * channel + 3] << 8);
}

// A pose with a different angle on every joint, so a servo reported under another's index shows.
void pose(float angles[NUM_SERVO]) {
    for (int i = 0; i < NUM_SERVO; i++) angles[i] = (i % 3 == 2 ? -90.0f : 0.0f) + 2.0f * (i % 9) - 8.0f;
}

// One control tick as main.cpp runs it while the robot stands: the tick starts, the motion's angles are written, and
// the telemetry is filled.
socket_message_TickSample angleTick(ServoController& controller) {
    host_clock::advance(TICK_US);
    controller.startTick();
    float angles[NUM_SERVO];
    pose(angles);
    controller.setAngles(angles);
    controller.updateServoState();
    socket_message_TickSample tick = socket_message_TickSample_init_zero;
    controller.fillTelemetry(tick);
    return tick;
}

// A tick in a mode that drives no servo: DEACTIVATED and IDLE write nothing.
socket_message_TickSample idleTick(ServoController& controller) {
    host_clock::advance(TICK_US);
    controller.startTick();
    socket_message_TickSample tick = socket_message_TickSample_init_zero;
    controller.fillTelemetry(tick);
    return tick;
}

// The controller is process-wide, as the robot's is, so its boards' backoff carries from test to test: each test
// begins on a fresh controller instead.
std::unique_ptr<ServoController> controller;

ServoController& started() {
    controller = std::make_unique<ServoController>();
    controller->begin();
    controller->activate();
    return *controller;
}

} // namespace

void setUp() {
    bus().reset();
    powerOnChip(RIGHT_BOARD);
    powerOnChip(LEFT_BOARD);
}

void tearDown() { controller.reset(); }

void test_a_written_tick_reports_both_boards_and_the_pulse_each_servo_got() {
    ServoController& c = started();
    const socket_message_TickSample tick = angleTick(c);
    TEST_ASSERT_EQUAL(socket_message_ServoWrite_SERVO_WRITE_OK, tick.servo_write_right);
    TEST_ASSERT_EQUAL(socket_message_ServoWrite_SERVO_WRITE_OK, tick.servo_write_left);
    for (int i = 0; i < NUM_SERVO; i++) {
        const uint8_t board = i < 9 ? RIGHT_BOARD : LEFT_BOARD;
        TEST_ASSERT_EQUAL_UINT16_MESSAGE(offCount(board, CHANNEL[i]), tick.pwm[i], "servo's pulse in the tick");
    }
    // Distinct pulses, so the comparison above cannot pass with the servos out of order.
    TEST_ASSERT_NOT_EQUAL(tick.pwm[0], tick.pwm[3]);
    TEST_ASSERT_NOT_EQUAL(tick.pwm[0], tick.pwm[9 + 3]);
}

// The calibration pose writes from the motion's own update, not from the angle path.
void test_the_centre_pose_reports_its_write() {
    ServoController& c = started();
    host_clock::advance(TICK_US);
    c.startTick();
    c.setCenterPwm();
    socket_message_TickSample tick = socket_message_TickSample_init_zero;
    c.fillTelemetry(tick);
    TEST_ASSERT_EQUAL(socket_message_ServoWrite_SERVO_WRITE_OK, tick.servo_write_right);
    TEST_ASSERT_EQUAL(socket_message_ServoWrite_SERVO_WRITE_OK, tick.servo_write_left);
    for (int i = 0; i < NUM_SERVO; i++) TEST_ASSERT_EQUAL_UINT16(306, tick.pwm[i]);
}

// A tick after a written one, in a mode that drives no servo, reports no write rather than the last one.
void test_a_tick_that_writes_nothing_reports_no_write() {
    ServoController& c = started();
    bus().transferUs = 100;
    angleTick(c);
    const socket_message_TickSample tick = idleTick(c);
    TEST_ASSERT_EQUAL(socket_message_ServoWrite_SERVO_WRITE_NONE, tick.servo_write_right);
    TEST_ASSERT_EQUAL(socket_message_ServoWrite_SERVO_WRITE_NONE, tick.servo_write_left);
    TEST_ASSERT_EQUAL_UINT32(0, tick.servo_write_us);
    for (int i = 0; i < NUM_SERVO; i++) TEST_ASSERT_EQUAL_UINT16(0, tick.pwm[i]);
}

// A board that stops answering fails three writes, then is skipped between its tries twice a second, each try failing
// again; the other board is written on every tick throughout. A skip is no failed write: it never reached the bus.
void test_a_board_that_stops_answering_reports_its_failures_apart_from_its_skips() {
    ServoController& c = started();
    angleTick(c);
    bus().absent.insert(RIGHT_BOARD);
    bus().attempts.clear();
    int failed = 0, skipped = 0, firstSkip = -1;
    for (int t = 0; t < 300; t++) {
        const socket_message_TickSample tick = angleTick(c);
        TEST_ASSERT_EQUAL(socket_message_ServoWrite_SERVO_WRITE_OK, tick.servo_write_left);
        if (tick.servo_write_right == socket_message_ServoWrite_SERVO_WRITE_FAILED) failed++;
        if (tick.servo_write_right == socket_message_ServoWrite_SERVO_WRITE_SKIPPED) {
            if (firstSkip < 0) firstSkip = t;
            skipped++;
        }
        // What the tick meant to write is reported for the board it could not write too.
        TEST_ASSERT_NOT_EQUAL(0, tick.pwm[0]);
    }
    TEST_ASSERT_EQUAL_INT(3, firstSkip);
    TEST_ASSERT_EQUAL_INT(5, failed);
    TEST_ASSERT_EQUAL_INT(295, skipped);
    TEST_ASSERT_EQUAL_INT(5, bus().attempts[RIGHT_BOARD]);
}

// A board that lost power is started again on the tick that finds it answering, and written as usual after.
void test_a_board_that_lost_power_reports_its_restart_once() {
    ServoController& c = started();
    angleTick(c);
    bus().absent.insert(LEFT_BOARD);
    for (int t = 0; t < 3; t++) angleTick(c);
    powerOnChip(LEFT_BOARD);
    bus().absent.clear();
    int restarts = 0, t = 0;
    socket_message_TickSample tick;
    do {
        tick = angleTick(c);
        restarts += tick.servo_write_left == socket_message_ServoWrite_SERVO_WRITE_RESTARTED;
    } while (tick.servo_write_left != socket_message_ServoWrite_SERVO_WRITE_RESTARTED && ++t < 200);
    TEST_ASSERT_EQUAL_INT(1, restarts);
    for (int i = 0; i < 10; i++)
        TEST_ASSERT_EQUAL(socket_message_ServoWrite_SERVO_WRITE_OK, angleTick(c).servo_write_left);
}

// The write time is the bus time of the tick's writes: one transfer per answering board, a failed one as long as any,
// a skipped board none, and none of the angle conversion before them.
void test_the_write_time_is_the_bus_time_of_the_tick() {
    ServoController& c = started();
    bus().transferUs = 150;
    TEST_ASSERT_EQUAL_UINT32(300, angleTick(c).servo_write_us);
    bus().absent.insert(RIGHT_BOARD);
    for (int t = 0; t < 3; t++) TEST_ASSERT_EQUAL_UINT32(300, angleTick(c).servo_write_us);
    const socket_message_TickSample skipping = angleTick(c);
    TEST_ASSERT_EQUAL(socket_message_ServoWrite_SERVO_WRITE_SKIPPED, skipping.servo_write_right);
    TEST_ASSERT_EQUAL_UINT32(150, skipping.servo_write_us);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_a_written_tick_reports_both_boards_and_the_pulse_each_servo_got);
    RUN_TEST(test_the_centre_pose_reports_its_write);
    RUN_TEST(test_a_tick_that_writes_nothing_reports_no_write);
    RUN_TEST(test_a_board_that_stops_answering_reports_its_failures_apart_from_its_skips);
    RUN_TEST(test_a_board_that_lost_power_reports_its_restart_once);
    RUN_TEST(test_the_write_time_is_the_bus_time_of_the_tick);
    return UNITY_END();
}
