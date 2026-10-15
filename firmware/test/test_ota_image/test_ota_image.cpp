// Host tests of the check that keeps a factory image out of an app slot. The two images are built
// the way the release build lays them out: the app image starts with its own header, the factory
// image with the bootloader's.

#include <unity.h>

#include <vector>
#include <ota_image.h>

namespace {

std::vector<uint8_t> appImageHead() {
    std::vector<uint8_t> head(64, 0);
    head[0] = 0xE9;
    head[12] = 9;  // chip id ESP32-S3
    const uint32_t magic = 0xABCD5432;
    for (int i = 0; i < 4; ++i) head[32 + i] = (magic >> (8 * i)) & 0xFF;
    return head;
}

// The bootloader is an image too, so it opens with 0xE9, but its first segment holds
// esp_bootloader_desc_t (a 0x50 magic byte), not the app descriptor.
std::vector<uint8_t> factoryImageHead() {
    std::vector<uint8_t> head(64, 0);
    head[0] = 0xE9;
    head[12] = 9;
    head[32] = 0x50;
    return head;
}

}  // namespace

void setUp() {}
void tearDown() {}

void test_an_app_image_is_accepted() {
    const auto head = appImageHead();
    TEST_ASSERT_TRUE(ota_image::isAppImage(head.data(), head.size()));
}

void test_a_factory_image_is_refused() {
    const auto head = factoryImageHead();
    TEST_ASSERT_FALSE(ota_image::isAppImage(head.data(), head.size()));
}

void test_a_file_that_is_no_image_is_refused() {
    auto head = appImageHead();
    head[0] = 0x7F;  // an ELF file, say
    TEST_ASSERT_FALSE(ota_image::isAppImage(head.data(), head.size()));
}

void test_too_few_bytes_to_tell_are_refused() {
    const auto head = appImageHead();
    TEST_ASSERT_FALSE(ota_image::isAppImage(head.data(), ota_image::HEAD_SIZE - 1));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_an_app_image_is_accepted);
    RUN_TEST(test_a_factory_image_is_refused);
    RUN_TEST(test_a_file_that_is_no_image_is_refused);
    RUN_TEST(test_too_few_bytes_to_tell_are_refused);
    return UNITY_END();
}
