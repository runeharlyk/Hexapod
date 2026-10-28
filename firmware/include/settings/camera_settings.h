#pragma once

#include <cstdint>
#include <template/state_result.h>
#include <platform_shared/api.pb.h>

// The camera's settings, as the esp32-camera driver's sensor_t holds them. Stored by CameraSettingsService and applied
// to the sensor by camera_service::apply.
using CameraSettings = api_CameraSettings;

// framesize_t values (driver/include/sensor.h of the esp32-camera component); camera_service.h asserts them against
// the enum, which a host build does not have.
constexpr uint32_t CAMERA_FRAMESIZE_VGA = 10;

// camera_service::init() sizes the JPEG frame buffers for this frame size: the driver gives each width x height / 5
// bytes (driver/cam_hal.c, cam_config), so a larger frame overflows them and is dropped.
constexpr uint32_t CAMERA_BUFFER_FRAMESIZE = CAMERA_FRAMESIZE_VGA;
constexpr int32_t CAMERA_DEFAULT_QUALITY = 12;

// The ranges the driver's setters accept (managed_components/espressif__esp32-camera 2.1.7). The board's sensor is not
// fixed, so each range is the one every likely sensor accepts:
// - quality 0..63: driver/include/esp_camera.h (jpeg_quality); sensors/ov2640.c clamps to it, ov3660.c and ov5640.c
//   keep the low 6 bits;
// - brightness, contrast, saturation -2..2: sensors/ov2640.c (NUM_*_LEVELS 5 around 0); ov3660.c and ov5640.c take
//   -3..3 or -4..4;
// - special_effect 0..6 and wb_mode 0..4: sensors/ov2640.c (NUM_SPECIAL_EFFECTS 7, NUM_WB_MODES 4), ov3660.c, ov5640.c;
// - sharpness -3..3 and denoise 0..8: sensors/ov3660.c and ov5640.c; a sensor with neither is checked at the update,
//   while it is up (CameraSettings_refusalBy).
inline bool CameraSettings_valid(const CameraSettings& settings) {
    const auto within = [](int32_t value, int32_t min, int32_t max) { return value >= min && value <= max; };
    return settings.framesize <= CAMERA_BUFFER_FRAMESIZE && within(settings.quality, 0, 63) &&
           within(settings.brightness, -2, 2) && within(settings.contrast, -2, 2) &&
           within(settings.saturation, -2, 2) && within(settings.sharpness, -3, 3) && within(settings.denoise, 0, 8) &&
           within(settings.special_effect, 0, 6) && within(settings.wb_mode, 0, 4);
}

// camera_pid_t values (driver/include/sensor.h); camera_service.h asserts them against the enum.
constexpr uint16_t CAMERA_OV2640_PID = 0x26;
constexpr uint16_t CAMERA_MEGA_CCM_PID = 0x039E;

// Why the sensor with product id `pid` refuses `settings` whatever their range, or nullptr. Of the sensors the driver
// streams JPEG from, the OV2640 and the MEGA_CCM have no sharpness or denoise, and their setters refuse every value
// (sensors/ov2640.c set_sharpness and set_denoise, sensors/mega_ccm.c set_dummy); the zero they report is taken, as
// camera_service::apply never sets a value the sensor already holds.
inline const char* CameraSettings_refusalBy(uint16_t pid, const CameraSettings& settings) {
    if (pid != CAMERA_OV2640_PID && pid != CAMERA_MEGA_CCM_PID) return nullptr;
    if (settings.sharpness != 0) return "this camera has no sharpness setting; leave it at 0";
    if (settings.denoise != 0) return "this camera has no denoise setting; leave it at 0";
    return nullptr;
}

// What camera_service::init() set before the settings were stored: VGA at quality 12, the rest at the sensor's
// power-on values.
inline CameraSettings CameraSettings_defaults() {
    CameraSettings settings = api_CameraSettings_init_zero;
    settings.framesize = CAMERA_FRAMESIZE_VGA;
    settings.quality = CAMERA_DEFAULT_QUALITY;
    return settings;
}

inline void CameraSettings_read(const CameraSettings& settings, CameraSettings& proto) { proto = settings; }

inline bool CameraSettings_equals(const CameraSettings& a, const CameraSettings& b) {
    return a.framesize == b.framesize && a.quality == b.quality && a.brightness == b.brightness &&
           a.contrast == b.contrast && a.saturation == b.saturation && a.sharpness == b.sharpness &&
           a.denoise == b.denoise && a.special_effect == b.special_effect && a.wb_mode == b.wb_mode &&
           a.vflip == b.vflip && a.hmirror == b.hmirror;
}

// A group with any value out of range is refused whole, so nothing is stored that the sensor would not take.
inline StateUpdateResult CameraSettings_update(const CameraSettings& proto, CameraSettings& settings) {
    if (!CameraSettings_valid(proto)) return StateUpdateResult::ERROR;
    if (CameraSettings_equals(proto, settings)) return StateUpdateResult::UNCHANGED;
    settings = proto;
    return StateUpdateResult::CHANGED;
}
