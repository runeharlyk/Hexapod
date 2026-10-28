// Host tests of the camera settings check (settings/camera_settings.h), which runs on the settings file read at boot
// and on every update. A value is refused when the esp32-camera driver would refuse it, or when it would not fit the
// frame buffers camera_service::init() allocates; a refused update leaves the stored settings as they were.

#include <unity.h>

#include <cstdint>
#include <cstring>
#include <functional>
#include <iterator>
#include <string>
#include <settings/camera_settings.h>

namespace {

constexpr uint32_t FRAMESIZE_VGA = 10;
constexpr uint32_t FRAMESIZE_SVGA = 11;

// One field and the range the driver accepts for it (managed_components/espressif__esp32-camera, sensors/ov2640.c,
// ov3660.c and ov5640.c, driver/include/esp_camera.h).
struct Field {
    const char* name;
    std::function<void(CameraSettings&, int64_t)> set;
    int64_t min;
    int64_t max;
};

const Field FIELDS[] = {
    {"framesize", [](CameraSettings& s, int64_t v) { s.framesize = static_cast<uint32_t>(v); }, 0, FRAMESIZE_VGA},
    {"quality", [](CameraSettings& s, int64_t v) { s.quality = static_cast<int32_t>(v); }, 0, 63},
    {"brightness", [](CameraSettings& s, int64_t v) { s.brightness = static_cast<int32_t>(v); }, -2, 2},
    {"contrast", [](CameraSettings& s, int64_t v) { s.contrast = static_cast<int32_t>(v); }, -2, 2},
    {"saturation", [](CameraSettings& s, int64_t v) { s.saturation = static_cast<int32_t>(v); }, -2, 2},
    {"sharpness", [](CameraSettings& s, int64_t v) { s.sharpness = static_cast<int32_t>(v); }, -3, 3},
    {"denoise", [](CameraSettings& s, int64_t v) { s.denoise = static_cast<int32_t>(v); }, 0, 8},
    {"special_effect", [](CameraSettings& s, int64_t v) { s.special_effect = static_cast<int32_t>(v); }, 0, 6},
    {"wb_mode", [](CameraSettings& s, int64_t v) { s.wb_mode = static_cast<int32_t>(v); }, 0, 4},
};

CameraSettings stored() {
    CameraSettings settings = CameraSettings_defaults();
    settings.brightness = 1;
    settings.vflip = true;
    return settings;
}

void assertSame(const CameraSettings& expected, const CameraSettings& actual, const std::string& context) {
    const char* message = context.c_str();
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(expected.framesize, actual.framesize, message);
    TEST_ASSERT_EQUAL_INT32_MESSAGE(expected.quality, actual.quality, message);
    TEST_ASSERT_EQUAL_INT32_MESSAGE(expected.brightness, actual.brightness, message);
    TEST_ASSERT_EQUAL_INT32_MESSAGE(expected.contrast, actual.contrast, message);
    TEST_ASSERT_EQUAL_INT32_MESSAGE(expected.saturation, actual.saturation, message);
    TEST_ASSERT_EQUAL_INT32_MESSAGE(expected.sharpness, actual.sharpness, message);
    TEST_ASSERT_EQUAL_INT32_MESSAGE(expected.denoise, actual.denoise, message);
    TEST_ASSERT_EQUAL_INT32_MESSAGE(expected.special_effect, actual.special_effect, message);
    TEST_ASSERT_EQUAL_INT32_MESSAGE(expected.wb_mode, actual.wb_mode, message);
    TEST_ASSERT_EQUAL_MESSAGE(expected.vflip, actual.vflip, message);
    TEST_ASSERT_EQUAL_MESSAGE(expected.hmirror, actual.hmirror, message);
}

} // namespace

void setUp() {}
void tearDown() {}

// What camera_service::init() set before the settings were stored: VGA at quality 12, the rest at the sensor's
// power-on values.
void test_the_defaults_are_what_the_camera_started_with_before() {
    const CameraSettings settings = CameraSettings_defaults();
    TEST_ASSERT_EQUAL_UINT32(FRAMESIZE_VGA, settings.framesize);
    TEST_ASSERT_EQUAL_INT32(12, settings.quality);
    TEST_ASSERT_EQUAL_INT32(0, settings.brightness);
    TEST_ASSERT_EQUAL_INT32(0, settings.contrast);
    TEST_ASSERT_EQUAL_INT32(0, settings.saturation);
    TEST_ASSERT_EQUAL_INT32(0, settings.sharpness);
    TEST_ASSERT_EQUAL_INT32(0, settings.denoise);
    TEST_ASSERT_EQUAL_INT32(0, settings.special_effect);
    TEST_ASSERT_EQUAL_INT32(0, settings.wb_mode);
    TEST_ASSERT_FALSE(settings.vflip);
    TEST_ASSERT_FALSE(settings.hmirror);
}

void test_each_value_at_the_ends_of_its_range_is_stored() {
    for (const Field& field : FIELDS) {
        for (const int64_t value : {field.min, field.max}) {
            const std::string context = std::string(field.name) + " = " + std::to_string(value);
            CameraSettings settings = stored();
            CameraSettings update = stored();
            field.set(update, value);

            const StateUpdateResult result = CameraSettings_update(update, settings);

            TEST_ASSERT_TRUE_MESSAGE(result != StateUpdateResult::ERROR, context.c_str());
            assertSame(update, settings, context);
        }
    }
}

void test_each_value_out_of_its_range_is_refused() {
    for (const Field& field : FIELDS) {
        for (const int64_t value : {field.min - 1, field.max + 1}) {
            const std::string context = std::string(field.name) + " = " + std::to_string(value);
            CameraSettings settings = stored();
            CameraSettings update = stored();
            update.hmirror = true;
            field.set(update, value);

            const StateUpdateResult result = CameraSettings_update(update, settings);

            TEST_ASSERT_TRUE_MESSAGE(result == StateUpdateResult::ERROR, context.c_str());
            assertSame(stored(), settings, context);
        }
    }
}

void test_an_equal_update_is_unchanged() {
    CameraSettings settings = stored();

    TEST_ASSERT_TRUE(CameraSettings_update(stored(), settings) == StateUpdateResult::UNCHANGED);
    assertSame(stored(), settings, "after an equal update");
}

void test_each_changed_value_is_a_change() {
    const std::function<void(CameraSettings&)> changes[] = {
        [](CameraSettings& s) { s.framesize = 5; },        [](CameraSettings& s) { s.quality = 30; },
        [](CameraSettings& s) { s.brightness = -1; },      [](CameraSettings& s) { s.contrast = 2; },
        [](CameraSettings& s) { s.saturation = -2; },      [](CameraSettings& s) { s.sharpness = 3; },
        [](CameraSettings& s) { s.denoise = 4; },          [](CameraSettings& s) { s.special_effect = 6; },
        [](CameraSettings& s) { s.wb_mode = 3; },          [](CameraSettings& s) { s.vflip = !s.vflip; },
        [](CameraSettings& s) { s.hmirror = !s.hmirror; },
    };
    for (size_t i = 0; i < std::size(changes); i++) {
        const std::string context = "change " + std::to_string(i);
        CameraSettings settings = stored();
        CameraSettings update = stored();
        changes[i](update);

        TEST_ASSERT_TRUE_MESSAGE(CameraSettings_update(update, settings) == StateUpdateResult::CHANGED,
                                 context.c_str());
        assertSame(update, settings, context);
    }
}

// The driver sizes each JPEG frame buffer once, at init, for the frame size init() asks for (width x height / 5 bytes,
// driver/cam_hal.c): a larger frame overflows it and is dropped, whatever the sensor itself could deliver.
void test_a_frame_size_too_large_for_the_buffers_is_refused() {
    for (const uint32_t framesize :
         {FRAMESIZE_SVGA, 15u /* UXGA, the OV2640's largest */, 25u /* FRAMESIZE_INVALID */}) {
        CameraSettings settings = stored();
        CameraSettings update = stored();
        update.framesize = framesize;

        TEST_ASSERT_TRUE(CameraSettings_update(update, settings) == StateUpdateResult::ERROR);
        TEST_ASSERT_EQUAL_UINT32(FRAMESIZE_VGA, settings.framesize);
    }
}

// The OV2640 and the MEGA_CCM have no sharpness or denoise: their setters refuse every value (sensors/ov2640.c,
// sensors/mega_ccm.c), so only the zero they report is taken. The OV3660 takes the whole range.
void test_a_sensor_without_sharpness_or_denoise_refuses_any_but_zero() {
    constexpr uint16_t OV2640 = 0x26, MEGA_CCM = 0x039E, OV3660 = 0x3660;
    CameraSettings sharp = CameraSettings_defaults();
    sharp.sharpness = -1;
    CameraSettings denoised = CameraSettings_defaults();
    denoised.denoise = 8;

    for (const uint16_t pid : {OV2640, MEGA_CCM}) {
        const std::string context = "pid " + std::to_string(pid);
        for (const CameraSettings& refused : {sharp, denoised}) {
            const char* reason = CameraSettings_refusalBy(pid, refused);
            TEST_ASSERT_NOT_NULL_MESSAGE(reason, context.c_str());
            // The reply's error_message holds 63 characters.
            TEST_ASSERT_LESS_OR_EQUAL_MESSAGE(63, std::strlen(reason), context.c_str());
        }
        TEST_ASSERT_NULL_MESSAGE(CameraSettings_refusalBy(pid, stored()), context.c_str());
    }
    TEST_ASSERT_NULL(CameraSettings_refusalBy(OV3660, sharp));
    TEST_ASSERT_NULL(CameraSettings_refusalBy(OV3660, denoised));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_the_defaults_are_what_the_camera_started_with_before);
    RUN_TEST(test_each_value_at_the_ends_of_its_range_is_stored);
    RUN_TEST(test_each_value_out_of_its_range_is_refused);
    RUN_TEST(test_an_equal_update_is_unchanged);
    RUN_TEST(test_each_changed_value_is_a_change);
    RUN_TEST(test_a_frame_size_too_large_for_the_buffers_is_refused);
    RUN_TEST(test_a_sensor_without_sharpness_or_denoise_refuses_any_but_zero);
    return UNITY_END();
}
