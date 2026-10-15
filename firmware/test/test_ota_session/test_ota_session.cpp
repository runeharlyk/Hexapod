// Host tests of the firmware update over the protobuf link: whatever goes wrong, the robot must
// keep booting the firmware it runs now, and only the client that started an update may drive it.

#include <unity.h>

#include <vector>
#include <ota_session.h>

namespace {

class FakeFlash : public OtaFlash {
  public:
    uint32_t slot = 4096;
    bool validates = true;
    bool begun = false, aborted = false, activated = false;
    std::vector<uint8_t> image;

    uint32_t capacity() override { return slot; }
    bool begin() override {
        begun = true;
        image.clear();
        return true;
    }
    bool write(const uint8_t* data, size_t size) override {
        image.insert(image.end(), data, data + size);
        return true;
    }
    bool end() override { return validates; }
    bool activate() override {
        activated = true;
        return true;
    }
    void abort() override { aborted = true; }
};

// The head of an app image (ota_image.h), padded to `size`.
std::vector<uint8_t> appImage(size_t size) {
    std::vector<uint8_t> bytes(size, 0x11);
    bytes[0] = 0xE9;
    const uint8_t magic[4] = {0x32, 0x54, 0xCD, 0xAB};
    for (int i = 0; i < 4; ++i) bytes[32 + i] = magic[i];
    return bytes;
}

int adapterA, adapterB;
const OtaSession::Client ALICE{&adapterA, 3};
const OtaSession::Client SERIAL_CLIENT{&adapterB, 3};  // the same id on another transport

}  // namespace

void setUp() {}
void tearDown() {}

void test_an_update_in_order_activates_the_image() {
    FakeFlash flash;
    std::atomic<bool> busy{false};
    OtaSession session(flash, busy);
    const auto image = appImage(1000);

    TEST_ASSERT_EQUAL(200, session.start(1000, ALICE, 0).status);
    TEST_ASSERT_EQUAL(200, session.chunk(0, image.data(), 512, ALICE, 10).status);
    TEST_ASSERT_EQUAL(200, session.chunk(1, image.data() + 512, 488, ALICE, 20).status);
    TEST_ASSERT_EQUAL(200, session.finish(ALICE).status);
    TEST_ASSERT_TRUE(flash.activated);
    TEST_ASSERT_TRUE(flash.image == image);
    // The robot restarts into the new image; no second update may start meanwhile.
    TEST_ASSERT_TRUE(busy);
}

void test_a_chunk_out_of_order_aborts_and_keeps_the_running_firmware() {
    FakeFlash flash;
    std::atomic<bool> busy{false};
    OtaSession session(flash, busy);
    const auto image = appImage(1024);

    session.start(1024, ALICE, 0);
    session.chunk(0, image.data(), 512, ALICE, 0);
    TEST_ASSERT_EQUAL(400, session.chunk(2, image.data(), 512, ALICE, 0).status);
    TEST_ASSERT_TRUE(flash.aborted);
    TEST_ASSERT_FALSE(flash.activated);
    TEST_ASSERT_FALSE(busy);
    TEST_ASSERT_EQUAL(409, session.finish(ALICE).status);
}

void test_more_data_than_announced_or_too_little_is_refused() {
    FakeFlash flash;
    std::atomic<bool> busy{false};
    OtaSession session(flash, busy);
    const auto image = appImage(600);

    session.start(512, ALICE, 0);
    TEST_ASSERT_EQUAL(400, session.chunk(0, image.data(), 600, ALICE, 0).status);

    session.start(1024, ALICE, 0);
    session.chunk(0, image.data(), 512, ALICE, 0);
    TEST_ASSERT_EQUAL(400, session.finish(ALICE).status);
    TEST_ASSERT_FALSE(flash.activated);
}

void test_a_factory_image_is_refused_at_its_first_chunk() {
    FakeFlash flash;
    std::atomic<bool> busy{false};
    OtaSession session(flash, busy);
    auto image = appImage(512);
    image[32] = 0x50;  // the bootloader's descriptor, as firmware.factory.bin starts with

    session.start(512, ALICE, 0);
    TEST_ASSERT_EQUAL(415, session.chunk(0, image.data(), 512, ALICE, 0).status);
    TEST_ASSERT_TRUE(flash.aborted);
}

void test_an_image_the_robot_rejects_is_not_activated() {
    FakeFlash flash;
    flash.validates = false;
    std::atomic<bool> busy{false};
    OtaSession session(flash, busy);
    const auto image = appImage(512);

    session.start(512, ALICE, 0);
    session.chunk(0, image.data(), 512, ALICE, 0);
    TEST_ASSERT_EQUAL(422, session.finish(ALICE).status);
    TEST_ASSERT_FALSE(flash.activated);
    TEST_ASSERT_FALSE(busy);
}

void test_only_the_starting_client_drives_an_update() {
    FakeFlash flash;
    std::atomic<bool> busy{false};
    OtaSession session(flash, busy);
    const auto image = appImage(512);

    session.start(512, ALICE, 0);
    TEST_ASSERT_EQUAL(409, session.start(512, SERIAL_CLIENT, 0).status);
    TEST_ASSERT_EQUAL(409, session.chunk(0, image.data(), 512, SERIAL_CLIENT, 0).status);
    TEST_ASSERT_EQUAL(409, session.finish(SERIAL_CLIENT).status);
    TEST_ASSERT_EQUAL(200, session.chunk(0, image.data(), 512, ALICE, 0).status);
}

void test_an_update_over_http_blocks_one_over_the_link() {
    FakeFlash flash;
    std::atomic<bool> busy{true};  // ota_service's upload holds the slot
    OtaSession session(flash, busy);
    TEST_ASSERT_EQUAL(409, session.start(512, ALICE, 0).status);
    TEST_ASSERT_FALSE(flash.begun);
    TEST_ASSERT_TRUE(busy);
}

void test_a_silent_update_expires_and_frees_the_slot() {
    FakeFlash flash;
    std::atomic<bool> busy{false};
    OtaSession session(flash, busy);
    session.start(512, ALICE, 1000);
    session.expire(1000 + OtaSession::TIMEOUT_MS);
    TEST_ASSERT_TRUE(session.running());
    session.expire(1001 + OtaSession::TIMEOUT_MS);
    TEST_ASSERT_FALSE(session.running());
    TEST_ASSERT_TRUE(flash.aborted);
    TEST_ASSERT_FALSE(busy);
}

void test_an_image_larger_than_the_slot_is_refused_up_front() {
    FakeFlash flash;
    std::atomic<bool> busy{false};
    OtaSession session(flash, busy);
    TEST_ASSERT_EQUAL(413, session.start(flash.slot + 1, ALICE, 0).status);
    TEST_ASSERT_FALSE(flash.begun);
    TEST_ASSERT_FALSE(busy);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_an_update_in_order_activates_the_image);
    RUN_TEST(test_a_chunk_out_of_order_aborts_and_keeps_the_running_firmware);
    RUN_TEST(test_more_data_than_announced_or_too_little_is_refused);
    RUN_TEST(test_a_factory_image_is_refused_at_its_first_chunk);
    RUN_TEST(test_an_image_the_robot_rejects_is_not_activated);
    RUN_TEST(test_only_the_starting_client_drives_an_update);
    RUN_TEST(test_an_update_over_http_blocks_one_over_the_link);
    RUN_TEST(test_a_silent_update_expires_and_frees_the_slot);
    RUN_TEST(test_an_image_larger_than_the_slot_is_refused_up_front);
    return UNITY_END();
}
