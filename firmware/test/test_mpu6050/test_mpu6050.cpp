// Host tests of how the MPU6050 driver drains the DMP's FIFO against the register fake in test/fakes. A failed
// or implausible FIFO count must end the drain: the control task calls update() every tick, and a drain that
// never ends there is a task watchdog reset.

#include <unity.h>

#include <peripherals/drivers/mpu6050.h>

namespace {

constexpr uint8_t ADDR = 0x68;
constexpr uint8_t REG_FIFO_COUNT_H = 0x72;
constexpr uint8_t REG_FIFO_R_W = 0x74;
constexpr uint8_t REG_MEM_R_W = 0x6F;
constexpr uint8_t REG_WHO_AM_I = 0x75;
constexpr size_t PACKET_SIZE = 28;

// A drain that has outlasted any sane bound; thrown so a driver that never ends fails the test instead of hanging it.
struct RunawayDrain {};
constexpr int RUNAWAY_COUNT_READS = 1000;

// What the chip's FIFO answers, and how often the driver asked.
struct Fifo {
    int countReads = 0;
    int packetReads = 0;
    // The count read that fails and every one after it; none while negative.
    int failCountsFrom = -1;
    bool packetFails = false;
    uint16_t count = 0;
    // The count read from which the FIFO holds exactly one packet; none while negative.
    int drainedAt = -1;
};
Fifo fifo;

void answerAsAFifo() {
    I2CBus::instance().readOverride = [](uint8_t, uint8_t reg, uint8_t *data, size_t len) -> std::optional<esp_err_t> {
        if (reg == REG_MEM_R_W) {
            data[0] = 0x00;
            data[1] = 0xF8;
            return ESP_OK;
        }
        if (reg == REG_FIFO_COUNT_H) {
            const int read = fifo.countReads++;
            if (read >= RUNAWAY_COUNT_READS) throw RunawayDrain {};
            // A transfer that timed out after clocking in part of the count leaves a plausible one in the buffer.
            const uint16_t count = fifo.drainedAt >= 0 && read >= fifo.drainedAt ? PACKET_SIZE : fifo.count;
            data[0] = count >> 8;
            data[1] = count & 0xFF;
            return fifo.failCountsFrom >= 0 && read >= fifo.failCountsFrom ? ESP_FAIL : ESP_OK;
        }
        if (reg == REG_FIFO_R_W && len == PACKET_SIZE) {
            ++fifo.packetReads;
            return fifo.packetFails ? ESP_FAIL : ESP_OK;
        }
        return std::nullopt;
    };
}

void startImu(MPU6050Driver &imu) {
    I2CBus &bus = I2CBus::instance();
    bus.reset();
    bus.registers[ADDR] = {};
    bus.registers[ADDR][REG_WHO_AM_I] = 0x68;
    answerAsAFifo();
    TEST_ASSERT_TRUE(imu.begin());
    fifo = Fifo {};
}

// False when update() outlasted the bound on count reads; otherwise `fresh` is what it returned.
bool updateEnds(MPU6050Driver &imu, bool &fresh) {
    try {
        fresh = imu.update();
    } catch (const RunawayDrain &) {
        return false;
    }
    return true;
}

} // namespace

void setUp() { fifo = Fifo {}; }
void tearDown() {}

void test_a_whole_packet_is_fresh() {
    MPU6050Driver imu(ADDR);
    startImu(imu);
    fifo.count = PACKET_SIZE;
    bool fresh = false;
    TEST_ASSERT_TRUE(updateEnds(imu, fresh));
    TEST_ASSERT_TRUE(fresh);
    TEST_ASSERT_EQUAL(1, fifo.packetReads);
}

void test_a_failed_count_read_is_not_fresh_and_ends_the_drain() {
    MPU6050Driver imu(ADDR);
    startImu(imu);
    fifo.count = 100;
    fifo.failCountsFrom = 0;
    bool fresh = true;
    TEST_ASSERT_TRUE(updateEnds(imu, fresh));
    TEST_ASSERT_FALSE(fresh);
    TEST_ASSERT_EQUAL(1, fifo.countReads);
    TEST_ASSERT_EQUAL(0, fifo.packetReads);
}

void test_a_count_read_that_fails_mid_drain_ends_it() {
    MPU6050Driver imu(ADDR);
    startImu(imu);
    fifo.count = 100;
    fifo.failCountsFrom = 1;
    bool fresh = true;
    TEST_ASSERT_TRUE(updateEnds(imu, fresh));
    TEST_ASSERT_FALSE(fresh);
    TEST_ASSERT_EQUAL(2, fifo.countReads);
    TEST_ASSERT_EQUAL(0, fifo.packetReads);
}

// The FIFO never drains: every count read after a discard finds it as full as before.
void test_a_count_that_stays_large_ends_within_a_few_reads() {
    MPU6050Driver imu(ADDR);
    startImu(imu);
    fifo.count = 100;
    bool fresh = true;
    TEST_ASSERT_TRUE(updateEnds(imu, fresh));
    TEST_ASSERT_FALSE(fresh);
    TEST_ASSERT_LESS_OR_EQUAL(5, fifo.countReads);
    TEST_ASSERT_EQUAL(0, fifo.packetReads);
}

// Three discards in a row is the most a drain makes; a packet found after the third still counts.
void test_a_packet_found_after_the_last_discard_is_fresh() {
    MPU6050Driver imu(ADDR);
    startImu(imu);
    fifo.count = 100;
    fifo.drainedAt = 3;
    bool fresh = false;
    TEST_ASSERT_TRUE(updateEnds(imu, fresh));
    TEST_ASSERT_TRUE(fresh);
    TEST_ASSERT_EQUAL(1, fifo.packetReads);
}

void test_a_count_that_stays_large_restarts_the_fifo() {
    MPU6050Driver imu(ADDR);
    startImu(imu);
    fifo.count = 100;
    const uint32_t resetsBefore = imu.fifoResets();
    bool fresh = true;
    TEST_ASSERT_TRUE(updateEnds(imu, fresh));
    TEST_ASSERT_EQUAL_UINT32(resetsBefore + 1, imu.fifoResets());
}

void test_an_overflowed_fifo_is_restarted() {
    MPU6050Driver imu(ADDR);
    startImu(imu);
    fifo.count = 1024;
    const uint32_t resetsBefore = imu.fifoResets();
    bool fresh = true;
    TEST_ASSERT_TRUE(updateEnds(imu, fresh));
    TEST_ASSERT_FALSE(fresh);
    TEST_ASSERT_EQUAL_UINT32(resetsBefore + 1, imu.fifoResets());
    TEST_ASSERT_EQUAL(1, fifo.countReads);
}

void test_a_failed_packet_read_is_not_fresh() {
    MPU6050Driver imu(ADDR);
    startImu(imu);
    fifo.count = PACKET_SIZE;
    fifo.packetFails = true;
    bool fresh = true;
    TEST_ASSERT_TRUE(updateEnds(imu, fresh));
    TEST_ASSERT_FALSE(fresh);
    TEST_ASSERT_EQUAL(1, fifo.packetReads);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_a_whole_packet_is_fresh);
    RUN_TEST(test_a_failed_count_read_is_not_fresh_and_ends_the_drain);
    RUN_TEST(test_a_count_read_that_fails_mid_drain_ends_it);
    RUN_TEST(test_a_count_that_stays_large_ends_within_a_few_reads);
    RUN_TEST(test_a_packet_found_after_the_last_discard_is_fresh);
    RUN_TEST(test_a_count_that_stays_large_restarts_the_fifo);
    RUN_TEST(test_an_overflowed_fifo_is_restarted);
    RUN_TEST(test_a_failed_packet_read_is_not_fresh);
    return UNITY_END();
}
