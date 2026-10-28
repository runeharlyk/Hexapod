#pragma once

#if USE_CAMERA

#include <esp_camera.h>
#include <esp_heap_caps.h>
#include <esp_http_server.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <atomic>
#include <cstdio>

#include <communication/webserver.h>
#include <settings/camera_settings.h>
#include <template/stateful_service.h>

// ESP32-S3-EYE pin map, JPEG in PSRAM, streamed as MJPEG over HTTP. Init is non-fatal.
namespace camera_service {

inline const char *TAG = "Camera";

static_assert(CAMERA_FRAMESIZE_VGA == FRAMESIZE_VGA, "settings/camera_settings.h names framesize_t's values");
static_assert(CAMERA_OV2640_PID == OV2640_PID && CAMERA_MEGA_CCM_PID == MEGA_CCM_PID,
              "settings/camera_settings.h names camera_pid_t's values");

// Serialises the sensor's users: a settings update on a transport's task, and the release for an update on the OTA
// task, which frees the sensor. Taken before the settings service's lock, never after it.
inline SemaphoreHandle_t sensorMutex = xSemaphoreCreateMutex();

// Why the sensor up now would refuse `settings` (CameraSettings_refusalBy), or nullptr. With no sensor up nothing is
// refused: the settings are stored for init() to apply.
inline const char *refuses(const CameraSettings &settings) {
    xSemaphoreTake(sensorMutex, portMAX_DELAY);
    const sensor_t *s = esp_camera_sensor_get();
    const char *refusal = s ? CameraSettings_refusalBy(s->id.PID, settings) : nullptr;
    xSemaphoreGive(sensorMutex);
    return refusal;
}

// Sets each value the sensor does not hold already (sensor_t::status, which every setter updates when it succeeds), so
// an unchanged frame size does not reset the sensor's window. Without a sensor (none found, or released) it does
// nothing; a value the sensor refuses is logged, as the OV2640 refuses every sharpness and denoise.
inline void applyHoldingSensor(const CameraSettings &settings) {
    sensor_t *s = esp_camera_sensor_get();
    if (!s) return;
    const auto refused = [](const char *name, int value) { ESP_LOGW(TAG, "The sensor refused %s %d", name, value); };
    const auto set = [&](const char *name, int held, int wanted, int (*setter)(sensor_t *, int)) {
        if (held != wanted && setter(s, wanted) != 0) refused(name, wanted);
    };
    const framesize_t framesize = static_cast<framesize_t>(settings.framesize);
    if (s->status.framesize != framesize && s->set_framesize(s, framesize) != 0) refused("framesize", framesize);
    set("quality", s->status.quality, settings.quality, s->set_quality);
    set("brightness", s->status.brightness, settings.brightness, s->set_brightness);
    set("contrast", s->status.contrast, settings.contrast, s->set_contrast);
    set("saturation", s->status.saturation, settings.saturation, s->set_saturation);
    set("sharpness", s->status.sharpness, settings.sharpness, s->set_sharpness);
    set("denoise", s->status.denoise, settings.denoise, s->set_denoise);
    set("special_effect", s->status.special_effect, settings.special_effect, s->set_special_effect);
    set("wb_mode", s->status.wb_mode, settings.wb_mode, s->set_wb_mode);
    set("vflip", s->status.vflip, settings.vflip, s->set_vflip);
    set("hmirror", s->status.hmirror, settings.hmirror, s->set_hmirror);
}

// Applies the stored settings to the sensor; while the camera is not up they stay stored, for init() to apply. They
// are read once the sensor is held, so of two updates applying at once the later one's settings end on the sensor.
inline void apply(StatefulService<CameraSettings> &stored) {
    xSemaphoreTake(sensorMutex, portMAX_DELAY);
    stored.read([](const CameraSettings &settings) { applyHoldingSensor(settings); });
    xSemaphoreGive(sensorMutex);
}

inline bool init(StatefulService<CameraSettings> &stored) {
    camera_config_t config = {};
    config.pin_pwdn = -1;
    config.pin_reset = -1;
    config.pin_xclk = 15;
    config.pin_sccb_sda = 4;
    config.pin_sccb_scl = 5;
    config.pin_d7 = 16;
    config.pin_d6 = 17;
    config.pin_d5 = 18;
    config.pin_d4 = 12;
    config.pin_d3 = 10;
    config.pin_d2 = 8;
    config.pin_d1 = 9;
    config.pin_d0 = 11;
    config.pin_vsync = 6;
    config.pin_href = 7;
    config.pin_pclk = 13;
    config.xclk_freq_hz = 20000000;
    config.ledc_timer = LEDC_TIMER_0;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.pixel_format = PIXFORMAT_JPEG;
    // The frame buffers are sized for this frame size, which bounds every stored one (CameraSettings_valid).
    config.frame_size = static_cast<framesize_t>(CAMERA_BUFFER_FRAMESIZE);
    config.jpeg_quality = CAMERA_DEFAULT_QUALITY;
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.sccb_i2c_port = 1;  // main peripheral bus owns I2C_NUM_0; keep SCCB on port 1

    xSemaphoreTake(sensorMutex, portMAX_DELAY);
    const esp_err_t err = esp_camera_init(&config);
    if (err == ESP_OK) stored.read([](const CameraSettings &settings) { applyHoldingSensor(settings); });
    xSemaphoreGive(sensorMutex);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: 0x%x (video disabled)", err);
        return false;
    }
    ESP_LOGI(TAG, "Camera ready");
    return true;
}

// Gives back the camera's internal DMA buffer, the largest block of internal RAM the robot holds, to
// work that needs it more; the stream below finds no frame afterwards and says so, and apply() finds no sensor.
inline void release() {
    xSemaphoreTake(sensorMutex, portMAX_DELAY);
    const esp_err_t err = esp_camera_deinit();
    xSemaphoreGive(sensorMutex);
    if (err == ESP_OK) ESP_LOGI(TAG, "Camera released");
}

#define CAMERA_STREAM_BOUNDARY "hexapodframe"

// esp_http_server serves every socket from one task, so a stream looping in its handler would hold that task for as
// long as anyone watches and leave every other request, the WebSocket's commands among them, unread. The handler hands
// each viewer to one worker instead, which takes every frame once and writes it to all of them; a viewer beyond
// MAX_VIEWERS is refused. A viewer whose send fails, or stalls past the server's send timeout, is dropped, and holds
// the others back until then.
inline constexpr int MAX_VIEWERS = 3;
inline QueueHandle_t newViewers = nullptr;
inline std::atomic<int> viewers {0};

// httpd_resp_set_type keeps the pointer until the first chunk flushes the headers, so the type must outlive the call.
inline const char *const STREAM_TYPE = "multipart/x-mixed-replace;boundary=" CAMERA_STREAM_BOUNDARY;

inline esp_err_t sendFrame(httpd_req_t *req, const char *part, int partLen, const camera_fb_t *fb) {
    esp_err_t res = httpd_resp_send_chunk(req, part, partLen);
    return res == ESP_OK ? httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len) : res;
}

inline void streamWorker(void *) {
    httpd_req_t *watching[MAX_VIEWERS];
    int count = 0;
    auto drop = [&](int i) {
        httpd_req_async_handler_complete(watching[i]);
        watching[i] = watching[--count];
        viewers--;
    };
    char part[128];
    for (;;) {
        httpd_req_t *req = nullptr;
        // Asleep while nobody watches; a newcomer joins at the next frame.
        while (xQueueReceive(newViewers, &req, count == 0 ? portMAX_DELAY : 0) == pdTRUE) {
            watching[count++] = req;
            if (httpd_resp_set_type(req, STREAM_TYPE) != ESP_OK) drop(count - 1);
        }
        if (count == 0) continue;
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGW(TAG, "frame capture failed");
            while (count > 0) drop(count - 1);
            continue;
        }
        const int n =
            snprintf(part, sizeof(part),
                     "\r\n--" CAMERA_STREAM_BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                     (unsigned)fb->len);
        for (int i = 0; i < count;) {
            if (sendFrame(watching[i], part, n, fb) == ESP_OK)
                ++i;
            else
                drop(i);
        }
        esp_camera_fb_return(fb);
    }
}

inline void startStreamWorker() {
    newViewers = xQueueCreate(MAX_VIEWERS, sizeof(httpd_req_t *));
    // A PSRAM stack, as the loop only takes the camera's PSRAM frames and writes sockets, never flash.
    if (!newViewers || xTaskCreatePinnedToCoreWithCaps(streamWorker, "camstream", 4096, nullptr, 5, nullptr, 0,
                                                       MALLOC_CAP_SPIRAM) != pdPASS) {
        ESP_LOGE(TAG, "camera stream worker not started");
        newViewers = nullptr;
    }
}

inline esp_err_t stream(httpd_req_t *req) {
    if (!newViewers) return WebServer::sendError(req, 503, "the camera stream is not running");
    int count = viewers.load();
    while (count < MAX_VIEWERS && !viewers.compare_exchange_weak(count, count + 1)) {
    }
    if (count >= MAX_VIEWERS) return WebServer::sendError(req, 503, "the camera stream is at its viewer limit");
    httpd_req_t *async = nullptr;
    if (httpd_req_async_handler_begin(req, &async) != ESP_OK) {
        viewers--;
        return ESP_FAIL;
    }
    if (xQueueSend(newViewers, &async, 0) != pdTRUE) {
        httpd_req_async_handler_complete(async);
        viewers--;
        return ESP_FAIL;
    }
    return ESP_OK;
}

inline void registerRoutes(WebServer &server) {
    startStreamWorker();
    server.on("/api/camera/stream", HTTP_GET, stream);
}

} // namespace camera_service

#endif // USE_CAMERA
