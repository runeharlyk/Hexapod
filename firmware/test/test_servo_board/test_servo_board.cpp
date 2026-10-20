// Host tests of a servo board's write backoff and restart against the register fake in test/fakes: a
// board that stops answering, one that answers again with its registers, one that lost power, and one
// that was missing at start, each driven by 5 ms control ticks.

#include <unity.h>

#include <peripherals/servo_board.h>

namespace {

constexpr uint8_t ADDR = 0x40;
constexpr uint32_t OSCILLATOR_HZ = 25000000;
constexpr float PWM_HZ = 50;
constexpr int64_t TICK_US = 5000;

constexpr uint8_t MODE1 = 0x00;
constexpr uint8_t MODE1_SLEEP = 0x10;
constexpr uint8_t MODE1_AI = 0x20;
constexpr uint8_t LED0_ON_L = 0x06;
constexpr uint8_t ALL_LED_OFF_H = 0xFD;
constexpr uint8_t PRE_SCALE = 0xFE;
constexpr uint8_t PRESCALE_50HZ = 121;  // round(25 MHz / (4096 * 50 Hz)) - 1

constexpr uint8_t CHANNELS = 11;
const uint16_t POSE[CHANNELS] = {300, 310, 320, 330, 340, 350, 360, 370, 380, 390, 400};

int64_t now = 0;
I2CBus& bus() { return I2CBus::instance(); }

using Write = ServoBoard::Write;

Write tick(ServoBoard& board) {
    now += TICK_US;
    return board.write(POSE, CHANNELS, now);
}

// The PCA9685's power-on registers: asleep with all-call, no auto-increment, the default prescale, and
// every channel full off.
void powerOnChip() {
    auto& chip = bus().registers[ADDR];
    chip = {};
    chip[MODE1] = 0x11;
    chip[PRE_SCALE] = 0x1E;
    for (int ch = 0; ch < 16; ch++) chip[LED0_ON_L + 4 * ch + 3] = 0x10;
}

uint16_t offCount(int channel) {
    const auto& chip = bus().registers[ADDR];
    return chip[LED0_ON_L + 4 * channel + 2] | (chip[LED0_ON_L + 4 * channel + 3] << 8);
}

void assertThePoseIsInEveryChannel() {
    for (int ch = 0; ch < CHANNELS; ch++) TEST_ASSERT_EQUAL_UINT16(POSE[ch], offCount(ch));
}

// The index in the write log of the last write to `reg` that satisfies `match`, or -1.
template <typename Match>
int lastWrite(uint8_t reg, Match match) {
    const auto& writes = bus().writes;
    for (int i = static_cast<int>(writes.size()) - 1; i >= 0; i--) {
        if (writes[i].addr == ADDR && writes[i].reg == reg && match(writes[i].bytes)) return i;
    }
    return -1;
}

// A board started and woken, as the control loop has it when the robot is active, with one write made.
void startAwake(ServoBoard& board) {
    TEST_ASSERT_TRUE(board.begin(now));
    TEST_ASSERT_EQUAL(Write::OK, board.wakeup(now));
    TEST_ASSERT_EQUAL(Write::OK, tick(board));
}

void failThreeWrites(ServoBoard& board) {
    bus().absent.insert(ADDR);
    for (int i = 0; i < 3; i++) TEST_ASSERT_EQUAL(Write::FAILED, tick(board));
    TEST_ASSERT_FALSE(board.answering());
}

}  // namespace

void setUp() {
    bus().reset();
    powerOnChip();
    now = 1000000;
}
void tearDown() {}

void test_a_board_that_keeps_answering_is_written_once_a_tick_and_never_read() {
    ServoBoard board(ADDR, OSCILLATOR_HZ, PWM_HZ);
    startAwake(board);
    bus().attempts.clear();
    for (int i = 0; i < 100; i++) TEST_ASSERT_EQUAL(Write::OK, tick(board));
    TEST_ASSERT_EQUAL_INT(100, bus().attempts[ADDR]);
    assertThePoseIsInEveryChannel();
}

// Three failed writes in a row, then one 0.5 s after the last and one 0.5 s after that: five in 1.5 s
// of ticks instead of three hundred, and every other tick reports its write skipped, not failed.
void test_a_board_that_stops_answering_is_tried_twice_a_second() {
    ServoBoard board(ADDR, OSCILLATOR_HZ, PWM_HZ);
    startAwake(board);
    bus().absent.insert(ADDR);
    bus().attempts.clear();
    int failed = 0, skipped = 0;
    for (int i = 0; i < 300; i++) {
        const Write write = tick(board);
        failed += write == Write::FAILED;
        skipped += write == Write::SKIPPED;
    }
    TEST_ASSERT_EQUAL_INT(5, failed);
    TEST_ASSERT_EQUAL_INT(295, skipped);
    TEST_ASSERT_EQUAL_INT(5, bus().attempts[ADDR]);
    TEST_ASSERT_FALSE(board.answering());
}

// A loose connector: the chip kept its power and registers, so it is written again within one retry
// interval and on every tick after, without being started again.
void test_a_board_that_answers_again_is_written_every_tick() {
    ServoBoard board(ADDR, OSCILLATOR_HZ, PWM_HZ);
    startAwake(board);
    failThreeWrites(board);
    const size_t writesBefore = bus().writes.size();
    bus().absent.clear();

    int ticksUntilWritten = 1;
    while (tick(board) != Write::OK && ticksUntilWritten <= 200) ticksUntilWritten++;
    TEST_ASSERT_LESS_OR_EQUAL_INT(100, ticksUntilWritten);
    TEST_ASSERT_TRUE(board.answering());

    bus().attempts.clear();
    for (int i = 0; i < 10; i++) TEST_ASSERT_EQUAL(Write::OK, tick(board));
    TEST_ASSERT_EQUAL_INT(10, bus().attempts[ADDR]);
    for (size_t i = writesBefore; i < bus().writes.size(); i++) {
        TEST_ASSERT_NOT_EQUAL_UINT8(ALL_LED_OFF_H, bus().writes[i].reg);
        TEST_ASSERT_NOT_EQUAL_UINT8(MODE1, bus().writes[i].reg);
    }
}

// A board that lost power answers asleep, at its power-on prescale and without auto-increment: it is
// started again, the pose reaches every channel before the chip wakes, and the tick that found it
// reports the restart.
void test_a_board_that_lost_power_is_started_again_with_the_pose_before_it_wakes() {
    ServoBoard board(ADDR, OSCILLATOR_HZ, PWM_HZ);
    startAwake(board);
    failThreeWrites(board);
    powerOnChip();
    bus().writes.clear();
    bus().absent.clear();

    int ticks = 0;
    while (tick(board) != Write::RESTARTED && ++ticks < 200) {
    }
    const auto& chip = bus().registers[ADDR];
    TEST_ASSERT_EQUAL_UINT8(PRESCALE_50HZ, chip[PRE_SCALE]);
    TEST_ASSERT_TRUE(chip[MODE1] & MODE1_AI);
    TEST_ASSERT_FALSE(chip[MODE1] & MODE1_SLEEP);
    assertThePoseIsInEveryChannel();

    const int pose = lastWrite(LED0_ON_L, [](const std::vector<uint8_t>& b) { return b.size() == 4 * CHANNELS; });
    const int wake = lastWrite(MODE1, [](const std::vector<uint8_t>& b) { return !(b[0] & MODE1_SLEEP); });
    TEST_ASSERT_GREATER_OR_EQUAL_INT(0, pose);
    TEST_ASSERT_GREATER_THAN_INT(pose, wake);
}

// A deactivated robot's board is started again asleep: power returning must not move the legs.
void test_a_sleeping_board_that_lost_power_is_started_again_asleep() {
    ServoBoard board(ADDR, OSCILLATOR_HZ, PWM_HZ);
    startAwake(board);
    board.sleep();
    failThreeWrites(board);
    powerOnChip();
    bus().absent.clear();

    int ticks = 0;
    while (tick(board) != Write::RESTARTED && ++ticks < 200) {
    }
    const auto& chip = bus().registers[ADDR];
    TEST_ASSERT_TRUE(chip[MODE1] & MODE1_AI);
    TEST_ASSERT_TRUE(chip[MODE1] & MODE1_SLEEP);
    TEST_ASSERT_EQUAL_UINT8(PRESCALE_50HZ, chip[PRE_SCALE]);
    assertThePoseIsInEveryChannel();
}

// A board without power at boot is started when it first answers, even though no write ever failed
// while the robot was active.
void test_a_board_missing_at_start_is_started_when_it_first_answers() {
    ServoBoard board(ADDR, OSCILLATOR_HZ, PWM_HZ);
    bus().absent.insert(ADDR);
    TEST_ASSERT_FALSE(board.begin(now));
    TEST_ASSERT_EQUAL(Write::FAILED, board.wakeup(now));
    bus().absent.clear();

    TEST_ASSERT_EQUAL(Write::RESTARTED, tick(board));
    const auto& chip = bus().registers[ADDR];
    TEST_ASSERT_EQUAL_UINT8(PRESCALE_50HZ, chip[PRE_SCALE]);
    TEST_ASSERT_TRUE(chip[MODE1] & MODE1_AI);
    TEST_ASSERT_FALSE(chip[MODE1] & MODE1_SLEEP);
    assertThePoseIsInEveryChannel();
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_a_board_that_keeps_answering_is_written_once_a_tick_and_never_read);
    RUN_TEST(test_a_board_that_stops_answering_is_tried_twice_a_second);
    RUN_TEST(test_a_board_that_answers_again_is_written_every_tick);
    RUN_TEST(test_a_board_that_lost_power_is_started_again_with_the_pose_before_it_wakes);
    RUN_TEST(test_a_sleeping_board_that_lost_power_is_started_again_asleep);
    RUN_TEST(test_a_board_missing_at_start_is_started_when_it_first_answers);
    return UNITY_END();
}
