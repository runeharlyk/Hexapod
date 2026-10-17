// Host tests of the telemetry path from the control task to the service task: a recording gets the
// ticks in order and in whole batches, knows how many the ring refused, and never starts with ticks
// left over from an earlier one.

#include <unity.h>

#include <memory>
#include <pb_decode.h>
#include <pb_encode.h>
#include <thread>
#include <telemetry/telemetry.h>

namespace {

socket_message_TickSample tick(uint32_t seq) {
    socket_message_TickSample t = socket_message_TickSample_init_zero;
    t.seq = seq;
    return t;
}

}  // namespace

void setUp() {}
void tearDown() {}

void test_nothing_is_recorded_while_nobody_listens() {
    auto owned = std::make_unique<Telemetry>();  // about 24 KB, as on the robot where it lives in PSRAM
    Telemetry& telemetry = *owned;
    for (uint32_t i = 0; i < 20; ++i) telemetry.record(tick(i));
    socket_message_TelemetryBatch batch;
    TEST_ASSERT_FALSE(telemetry.takeBatch(batch));
}

void test_ticks_leave_in_order_in_whole_batches() {
    auto owned = std::make_unique<Telemetry>();  // about 24 KB, as on the robot where it lives in PSRAM
    Telemetry& telemetry = *owned;
    telemetry.setRecording(true);
    const uint32_t n = Telemetry::BATCH_TICKS;
    for (uint32_t i = 0; i < 2 * n + 1; ++i) telemetry.record(tick(i));

    socket_message_TelemetryBatch batch;
    for (uint32_t b = 0; b < 2; ++b) {
        TEST_ASSERT_TRUE(telemetry.takeBatch(batch));
        TEST_ASSERT_EQUAL_UINT32(b, batch.batch_seq);
        TEST_ASSERT_EQUAL(n, batch.ticks_count);
        for (uint32_t i = 0; i < n; ++i) TEST_ASSERT_EQUAL_UINT32(b * n + i, batch.ticks[i].seq);
    }
    // The odd tick waits for a full batch rather than going out alone.
    TEST_ASSERT_FALSE(telemetry.takeBatch(batch));
}

void test_a_full_ring_counts_what_it_refuses() {
    auto owned = std::make_unique<Telemetry>();  // about 24 KB, as on the robot where it lives in PSRAM
    Telemetry& telemetry = *owned;
    telemetry.setRecording(true);
    for (uint32_t i = 0; i < Telemetry::RING_TICKS + 7; ++i) telemetry.record(tick(i));
    TEST_ASSERT_EQUAL_UINT32(7, telemetry.droppedTicks());
    socket_message_TelemetryBatch batch;
    TEST_ASSERT_TRUE(telemetry.takeBatch(batch));
    TEST_ASSERT_EQUAL_UINT32(7, batch.dropped_ticks);
    TEST_ASSERT_EQUAL_UINT32(0, batch.ticks[0].seq);
}

void test_a_new_recording_starts_with_fresh_ticks() {
    auto owned = std::make_unique<Telemetry>();  // about 24 KB, as on the robot where it lives in PSRAM
    Telemetry& telemetry = *owned;
    telemetry.setRecording(true);
    for (uint32_t i = 0; i < 3; ++i) telemetry.record(tick(i));
    telemetry.setRecording(false);
    telemetry.discardUnsent();
    telemetry.setRecording(true);
    for (uint32_t i = 100; i < 100 + Telemetry::BATCH_TICKS; ++i) telemetry.record(tick(i));
    socket_message_TelemetryBatch batch;
    TEST_ASSERT_TRUE(telemetry.takeBatch(batch));
    TEST_ASSERT_EQUAL_UINT32(100, batch.ticks[0].seq);
}

// One producer and one consumer on separate threads, as the control and service tasks run: every
// tick arrives exactly once, in order, or is counted as dropped.
void test_two_tasks_lose_no_tick_unaccounted() {
    auto owned = std::make_unique<Telemetry>();  // about 24 KB, as on the robot where it lives in PSRAM
    Telemetry& telemetry = *owned;
    telemetry.setRecording(true);
    constexpr uint32_t TICKS = 200000;
    std::thread producer([&] {
        for (uint32_t i = 0; i < TICKS; ++i) telemetry.record(tick(i));
    });
    uint32_t received = 0, expected = 0;
    bool ordered = true;
    socket_message_TelemetryBatch batch;
    auto drain = [&] {
        while (telemetry.takeBatch(batch)) {
            for (pb_size_t i = 0; i < batch.ticks_count; ++i) {
                if (batch.ticks[i].seq < expected) ordered = false;
                expected = batch.ticks[i].seq + 1;
                received++;
            }
        }
    };
    while (received + telemetry.droppedTicks() + Telemetry::BATCH_TICKS <= TICKS) drain();
    producer.join();
    drain();
    TEST_ASSERT_TRUE(ordered);
    TEST_ASSERT_UINT32_WITHIN(Telemetry::BATCH_TICKS, TICKS, received + telemetry.droppedTicks());
}

// The raw compass reading is what a hard-iron fit is made from: it must survive the ring and the wire exactly, and
// a tick that did not read the compass must say so.
void test_the_compass_reading_survives_the_ring_and_the_wire() {
    auto owned = std::make_unique<Telemetry>();
    Telemetry& telemetry = *owned;
    telemetry.setRecording(true);
    for (uint32_t i = 0; i < Telemetry::BATCH_TICKS; ++i) {
        socket_message_TickSample t = tick(i);
        t.mag_fresh = i % 2 == 0;
        t.mag[0] = 10.5f + i;
        t.mag[1] = -85.25f;
        t.mag[2] = 30.0f;
        telemetry.record(t);
    }
    socket_message_TelemetryBatch batch;
    TEST_ASSERT_TRUE(telemetry.takeBatch(batch));

    uint8_t wire[socket_message_TelemetryBatch_size];
    pb_ostream_t out = pb_ostream_from_buffer(wire, sizeof wire);
    TEST_ASSERT_TRUE(pb_encode(&out, socket_message_TelemetryBatch_fields, &batch));
    socket_message_TelemetryBatch decoded = socket_message_TelemetryBatch_init_zero;
    pb_istream_t in = pb_istream_from_buffer(wire, out.bytes_written);
    TEST_ASSERT_TRUE(pb_decode(&in, socket_message_TelemetryBatch_fields, &decoded));

    TEST_ASSERT_EQUAL(Telemetry::BATCH_TICKS, decoded.ticks_count);
    for (uint32_t i = 0; i < Telemetry::BATCH_TICKS; ++i) {
        TEST_ASSERT_EQUAL(i % 2 == 0, decoded.ticks[i].mag_fresh);
        TEST_ASSERT_EQUAL_FLOAT(10.5f + i, decoded.ticks[i].mag[0]);
        TEST_ASSERT_EQUAL_FLOAT(-85.25f, decoded.ticks[i].mag[1]);
        TEST_ASSERT_EQUAL_FLOAT(30.0f, decoded.ticks[i].mag[2]);
    }
}

// The servo write's fields survive the ring and the wire: every pulse up to the PCA9685's 4095, both boards' results,
// the write time and the dead-man flag.
void test_the_servo_write_survives_the_ring_and_the_wire() {
    auto owned = std::make_unique<Telemetry>();
    Telemetry& telemetry = *owned;
    telemetry.setRecording(true);
    for (uint32_t i = 0; i < Telemetry::BATCH_TICKS; ++i) {
        socket_message_TickSample t = tick(i);
        for (int s = 0; s < 18; ++s) t.pwm[s] = static_cast<uint16_t>(4095 - 200 * s - i);
        t.servo_write_us = 1100 + i;
        t.servo_write_right = socket_message_ServoWrite_SERVO_WRITE_SKIPPED;
        t.servo_write_left =
            i % 2 ? socket_message_ServoWrite_SERVO_WRITE_RESTARTED : socket_message_ServoWrite_SERVO_WRITE_OK;
        t.link_lost = i >= 3;
        telemetry.record(t);
    }
    socket_message_TelemetryBatch batch;
    TEST_ASSERT_TRUE(telemetry.takeBatch(batch));

    uint8_t wire[socket_message_TelemetryBatch_size];
    pb_ostream_t out = pb_ostream_from_buffer(wire, sizeof wire);
    TEST_ASSERT_TRUE(pb_encode(&out, socket_message_TelemetryBatch_fields, &batch));
    socket_message_TelemetryBatch decoded = socket_message_TelemetryBatch_init_zero;
    pb_istream_t in = pb_istream_from_buffer(wire, out.bytes_written);
    TEST_ASSERT_TRUE(pb_decode(&in, socket_message_TelemetryBatch_fields, &decoded));

    for (uint32_t i = 0; i < Telemetry::BATCH_TICKS; ++i) {
        const socket_message_TickSample& t = decoded.ticks[i];
        for (int s = 0; s < 18; ++s) TEST_ASSERT_EQUAL_UINT16(4095 - 200 * s - i, t.pwm[s]);
        TEST_ASSERT_EQUAL_UINT32(1100 + i, t.servo_write_us);
        TEST_ASSERT_EQUAL(socket_message_ServoWrite_SERVO_WRITE_SKIPPED, t.servo_write_right);
        TEST_ASSERT_EQUAL(
            i % 2 ? socket_message_ServoWrite_SERVO_WRITE_RESTARTED : socket_message_ServoWrite_SERVO_WRITE_OK,
            t.servo_write_left);
        TEST_ASSERT_EQUAL(i >= 3, t.link_lost);
    }
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_nothing_is_recorded_while_nobody_listens);
    RUN_TEST(test_ticks_leave_in_order_in_whole_batches);
    RUN_TEST(test_a_full_ring_counts_what_it_refuses);
    RUN_TEST(test_a_new_recording_starts_with_fresh_ticks);
    RUN_TEST(test_two_tasks_lose_no_tick_unaccounted);
    RUN_TEST(test_the_compass_reading_survives_the_ring_and_the_wire);
    RUN_TEST(test_the_servo_write_survives_the_ring_and_the_wire);
    return UNITY_END();
}
