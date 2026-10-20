// Host tests of what the servo controller reports of each tick's write (TickSample's pwm, servo_write_us and the two
// boards' results), against the I2C register fake: a write that both boards take, a tick that writes nothing, a board
// that stops answering and is then backed off, one that lost power and is started again, and the time the bus took.
// The include path puts the host stand-in of the controller first, so the robot's own is included by its path.

#include <unity.h>

#include <algorithm>
#include <array>
#include <memory>
#include "../../include/peripherals/servo_controller.h"

namespace {

constexpr uint8_t RIGHT_BOARD = 0x41;
constexpr uint8_t LEFT_BOARD = 0x40;
constexpr uint8_t LED0_ON_L = 0x06;
constexpr uint8_t PRE_SCALE = 0xFE;
constexpr uint8_t MODE1 = 0x00;
constexpr uint8_t ALL_LED_OFF_H = 0xFD;
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

// A tick in IDLE that writes the joints dragged in the app.
socket_message_TickSample jointTick(ServoController& controller, int servo, float degrees) {
    host_clock::advance(TICK_US);
    controller.startTick();
    controller.setJointAngle(servo, degrees);
    socket_message_TickSample tick = socket_message_TickSample_init_zero;
    controller.fillTelemetry(tick);
    return tick;
}

// A joint dragged in IDLE is a write of its own board, reported as the motion's are, with the pulse it got; the other
// board is not written.
void test_a_joint_write_reports_its_boards_write() {
    ServoController& c = started();
    bus().transferUs = 120;
    const socket_message_TickSample right = jointTick(c, 4, 20.0f);
    TEST_ASSERT_EQUAL(socket_message_ServoWrite_SERVO_WRITE_OK, right.servo_write_right);
    TEST_ASSERT_EQUAL(socket_message_ServoWrite_SERVO_WRITE_NONE, right.servo_write_left);
    TEST_ASSERT_EQUAL_UINT16(offCount(RIGHT_BOARD, CHANNEL[4]), right.pwm[4]);
    TEST_ASSERT_EQUAL_UINT16(306 + 20 * 2, right.pwm[4]);
    TEST_ASSERT_EQUAL_UINT32(120, right.servo_write_us);
    const socket_message_TickSample left = jointTick(c, 13, 10.0f);
    TEST_ASSERT_EQUAL(socket_message_ServoWrite_SERVO_WRITE_NONE, left.servo_write_right);
    TEST_ASSERT_EQUAL(socket_message_ServoWrite_SERVO_WRITE_OK, left.servo_write_left);
}

// A joint write reports the one pulse it wrote: the other servos of its board hold pulses from an earlier tick, which
// this tick did not write, and report none, as the servos of a board a tick does not write report none.
void test_a_joint_write_reports_only_its_servos_pulse() {
    ServoController& c = started();
    angleTick(c);
    const socket_message_TickSample tick = jointTick(c, 4, 20.0f);
    TEST_ASSERT_EQUAL_UINT16(306 + 20 * 2, tick.pwm[4]);
    for (int i = 0; i < NUM_SERVO; i++)
        if (i != 4) TEST_ASSERT_EQUAL_UINT16_MESSAGE(0, tick.pwm[i], "a servo the tick did not write");
}

// A joint dragged onto a board that stopped answering waits out its backoff as the motion's writes do: skipped, never
// sent to the bus, and reported so.
void test_a_joint_write_to_a_board_in_backoff_is_skipped() {
    ServoController& c = started();
    angleTick(c);
    bus().absent.insert(RIGHT_BOARD);
    for (int t = 0; t < 3; t++) angleTick(c);
    bus().attempts.clear();
    const socket_message_TickSample tick = jointTick(c, 4, 20.0f);
    TEST_ASSERT_EQUAL(socket_message_ServoWrite_SERVO_WRITE_SKIPPED, tick.servo_write_right);
    TEST_ASSERT_EQUAL_INT(0, bus().attempts[RIGHT_BOARD]);
    TEST_ASSERT_EQUAL(socket_message_ServoWrite_SERVO_WRITE_OK, jointTick(c, 13, 10.0f).servo_write_left);
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
    int t = 0;
    socket_message_TickSample tick;
    do {
        tick = angleTick(c);
    } while (tick.servo_write_left != socket_message_ServoWrite_SERVO_WRITE_RESTARTED && ++t < 200);
    TEST_ASSERT_EQUAL(socket_message_ServoWrite_SERVO_WRITE_RESTARTED, tick.servo_write_left);
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

// Which of a board's channels output a pulse, from every write the controller made to it, replayed in order on a chip
// that powered on as setUp powers it: a write to ALL_LED_OFF_H loads every channel's LEDn_OFF_H, as the PCA9685 does,
// and a sleeping chip outputs nothing.
std::array<bool, 16> pulsing(uint8_t addr) {
    constexpr uint8_t ALL_LED_ON_L = 0xFA, SLEEP = 0x10, FULL = 0x10;
    std::array<uint8_t, 256> chip {};
    chip[MODE1] = 0x11;
    for (int ch = 0; ch < 16; ch++) chip[LED0_ON_L + 4 * ch + 3] = FULL;
    for (const FakeWrite& w : bus().writes) {
        if (w.addr != addr) continue;
        for (size_t i = 0; i < w.bytes.size(); i++) {
            const uint8_t reg = (w.reg + i) & 0xFF;
            if (reg >= ALL_LED_ON_L && reg <= ALL_LED_OFF_H) {
                for (int ch = 0; ch < 16; ch++) chip[LED0_ON_L + 4 * ch + (reg - ALL_LED_ON_L)] = w.bytes[i];
            } else {
                chip[reg] = w.bytes[i];
            }
        }
    }
    std::array<bool, 16> out {};
    for (int ch = 0; ch < 16; ch++) {
        const uint8_t* led = &chip[LED0_ON_L + 4 * ch];
        const bool fullOff = led[3] & FULL, fullOn = led[1] & FULL;
        const uint16_t off = led[2] | ((led[3] & 0x0F) << 8);
        out[ch] = !(chip[MODE1] & SLEEP) && !fullOff && (fullOn || off != 0);
    }
    return out;
}

int pulsingChannels(uint8_t addr) {
    const std::array<bool, 16> out = pulsing(addr);
    return static_cast<int>(std::count(out.begin(), out.end(), true));
}

// A chip keeps its channel registers asleep and restarts them on waking, so the servos driven before a deactivation
// would all jump back to that pose the moment the boards are activated again. Activated, every servo is limp until it
// is written: a joint drives its own servo alone, and a board started again after losing power restarts none of the
// pulses from before the activation.
void test_an_activation_after_a_drive_leaves_every_servo_limp_until_written() {
    ServoController& c = started();
    angleTick(c);
    TEST_ASSERT_EQUAL_INT(9, pulsingChannels(RIGHT_BOARD));
    c.deactivate();
    TEST_ASSERT_EQUAL_INT(0, pulsingChannels(RIGHT_BOARD));
    c.activate();
    TEST_ASSERT_EQUAL_INT(0, pulsingChannels(RIGHT_BOARD));
    TEST_ASSERT_EQUAL_INT(0, pulsingChannels(LEFT_BOARD));

    jointTick(c, 4, 20.0f);
    TEST_ASSERT_EQUAL_INT(1, pulsingChannels(RIGHT_BOARD));
    TEST_ASSERT_TRUE(pulsing(RIGHT_BOARD)[CHANNEL[4]]);
    TEST_ASSERT_EQUAL_INT(0, pulsingChannels(LEFT_BOARD));

    bus().absent.insert(LEFT_BOARD);
    jointTick(c, 13, 10.0f);
    powerOnChip(LEFT_BOARD);
    bus().absent.clear();
    TEST_ASSERT_EQUAL(socket_message_ServoWrite_SERVO_WRITE_RESTARTED, jointTick(c, 13, 10.0f).servo_write_left);
    TEST_ASSERT_EQUAL_INT(1, pulsingChannels(LEFT_BOARD));
    TEST_ASSERT_TRUE(pulsing(LEFT_BOARD)[CHANNEL[13]]);
}

// An activation whose all-off write a glitch on the bus loses must not wake that board, whose channels still hold the
// pose from before the deactivation: it stays asleep, reported as a failed write is, and the next write that reaches it
// wakes it with every channel off before its own pulse.
void test_an_activation_whose_all_off_is_lost_leaves_its_board_asleep_and_reports_it() {
    ServoController& c = started();
    angleTick(c);
    c.deactivate();
    bool lost = false;
    bus().writeOverride = [&](uint8_t addr, uint8_t reg, const uint8_t*, size_t) -> std::optional<esp_err_t> {
        if (addr != RIGHT_BOARD || reg != ALL_LED_OFF_H || lost) return std::nullopt;
        lost = true;
        return ESP_FAIL;
    };
    host_clock::advance(TICK_US);
    c.startTick();
    c.activate();
    socket_message_TickSample tick = socket_message_TickSample_init_zero;
    c.fillTelemetry(tick);
    TEST_ASSERT_TRUE(lost);
    TEST_ASSERT_EQUAL_INT(0, pulsingChannels(RIGHT_BOARD));
    TEST_ASSERT_EQUAL_INT(0, pulsingChannels(LEFT_BOARD));
    TEST_ASSERT_EQUAL(socket_message_ServoWrite_SERVO_WRITE_FAILED, tick.servo_write_right);
    TEST_ASSERT_EQUAL(socket_message_ServoWrite_SERVO_WRITE_OK, tick.servo_write_left);

    TEST_ASSERT_EQUAL(socket_message_ServoWrite_SERVO_WRITE_OK, jointTick(c, 4, 20.0f).servo_write_right);
    TEST_ASSERT_EQUAL_INT(1, pulsingChannels(RIGHT_BOARD));
    TEST_ASSERT_TRUE(pulsing(RIGHT_BOARD)[CHANNEL[4]]);
}

// The servo page's switch is the control task's to apply (MotionService), as a mode is: delivered on the bus, it
// leaves the boards and the servos written as they were, so no bus worker writes them beside the control task.
void test_the_servo_pages_switch_on_the_bus_leaves_the_controller_alone() {
    ServoController& c = started();
    angleTick(c);
    EventBus<ServoStateMsg>::publish({false});
    host_bus::drainAll();
    TEST_ASSERT_EQUAL_HEX32((1u << NUM_SERVO) - 1, c.driven());
    TEST_ASSERT_EQUAL_INT(9, pulsingChannels(RIGHT_BOARD));
}

// A servo is driven only by a pulse an awake board outputs: a pose written while the boards sleep, as the servo page's
// switch can leave them in any mode, drives none, and every servo stays limp until written after they wake.
void test_a_pose_written_to_sleeping_boards_drives_no_servo() {
    ServoController& c = started();
    angleTick(c);
    TEST_ASSERT_EQUAL_HEX32((1u << NUM_SERVO) - 1, c.driven());
    c.deactivate();
    TEST_ASSERT_EQUAL_HEX32(0, c.driven());
    angleTick(c);
    jointTick(c, 4, 20.0f);
    TEST_ASSERT_EQUAL_HEX32(0, c.driven());
    c.activate();
    TEST_ASSERT_EQUAL_INT(0, pulsingChannels(RIGHT_BOARD));
    jointTick(c, 4, 20.0f);
    TEST_ASSERT_EQUAL_HEX32(1u << 4, c.driven());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_an_activation_after_a_drive_leaves_every_servo_limp_until_written);
    RUN_TEST(test_a_pose_written_to_sleeping_boards_drives_no_servo);
    RUN_TEST(test_the_servo_pages_switch_on_the_bus_leaves_the_controller_alone);
    RUN_TEST(test_an_activation_whose_all_off_is_lost_leaves_its_board_asleep_and_reports_it);
    RUN_TEST(test_a_written_tick_reports_both_boards_and_the_pulse_each_servo_got);
    RUN_TEST(test_the_centre_pose_reports_its_write);
    RUN_TEST(test_a_joint_write_reports_its_boards_write);
    RUN_TEST(test_a_joint_write_reports_only_its_servos_pulse);
    RUN_TEST(test_a_joint_write_to_a_board_in_backoff_is_skipped);
    RUN_TEST(test_a_tick_that_writes_nothing_reports_no_write);
    RUN_TEST(test_a_board_that_stops_answering_reports_its_failures_apart_from_its_skips);
    RUN_TEST(test_a_board_that_lost_power_reports_its_restart_once);
    RUN_TEST(test_the_write_time_is_the_bus_time_of_the_tick);
    return UNITY_END();
}
