#pragma once

#if USE_CAMERA

#include <esp_camera.h>
#include <esp_heap_caps.h>
#include <esp_http_server.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <cJSON.h>
#include <atomic>
#include <cstdio>
#include <cstring>
#include <string>

#include <communication/webserver.h>

// ESP32-S3-EYE pin map, JPEG in PSRAM, streamed as MJPEG over HTTP. Init is non-fatal.
namespace camera_service {

inline const char *TAG = "Camera";

inline bool init() {
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
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 12;
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.sccb_i2c_port = 1;  // main peripheral bus owns I2C_NUM_0; keep SCCB on port 1

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: 0x%x (video disabled)", err);
        return false;
    }
    ESP_LOGI(TAG, "Camera ready");
    return true;
}

// Gives back the camera's internal DMA buffer, the largest block of internal RAM the robot holds, to
// work that needs it more; the handlers below find no frame and no sensor afterwards and say so.
inline void release() {
    if (esp_camera_deinit() == ESP_OK) ESP_LOGI(TAG, "Camera released");
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

inline esp_err_t settingsGet(httpd_req_t *req) {
    sensor_t *s = esp_camera_sensor_get();
    if (!s) return WebServer::sendError(req, 500, "no sensor");
    cJSON *j = cJSON_CreateObject();
    cJSON_AddNumberToObject(j, "framesize", s->status.framesize);
    cJSON_AddNumberToObject(j, "quality", s->status.quality);
    cJSON_AddNumberToObject(j, "brightness", s->status.brightness);
    cJSON_AddNumberToObject(j, "contrast", s->status.contrast);
    cJSON_AddNumberToObject(j, "saturation", s->status.saturation);
    cJSON_AddNumberToObject(j, "sharpness", s->status.sharpness);
    cJSON_AddNumberToObject(j, "denoise", s->status.denoise);
    cJSON_AddNumberToObject(j, "special_effect", s->status.special_effect);
    cJSON_AddNumberToObject(j, "wb_mode", s->status.wb_mode);
    cJSON_AddBoolToObject(j, "vflip", s->status.vflip);
    cJSON_AddBoolToObject(j, "hmirror", s->status.hmirror);
    char *out = cJSON_PrintUnformatted(j);
    httpd_resp_set_type(req, "application/json");
    esp_err_t r = httpd_resp_sendstr(req, out ? out : "{}");
    cJSON_free(out);
    cJSON_Delete(j);
    return r;
}

inline esp_err_t settingsPost(httpd_req_t *req) {
    int len = req->content_len;
    if (len <= 0 || len > 1024) return WebServer::sendError(req, 400, "bad body");
    std::string raw(len, '\0');
    int got = 0;
    while (got < len) {
        int r = httpd_req_recv(req, &raw[got], len - got);
        if (r <= 0) return WebServer::sendError(req, 400, "recv failed");
        got += r;
    }
    cJSON *j = cJSON_Parse(raw.c_str());
    sensor_t *s = esp_camera_sensor_get();
    if (!j || !s) {
        if (j) cJSON_Delete(j);
        return WebServer::sendError(req, 400, "bad request");
    }
    auto num = [&](const char *k, auto setter) {
        cJSON *v = cJSON_GetObjectItem(j, k);
        if (cJSON_IsNumber(v)) setter(s, v->valueint);
        else if (cJSON_IsBool(v)) setter(s, cJSON_IsTrue(v) ? 1 : 0);
    };
    num("framesize", [](sensor_t *s, int v) { s->set_framesize(s, (framesize_t)v); });
    num("quality", [](sensor_t *s, int v) { s->set_quality(s, v); });
    num("brightness", [](sensor_t *s, int v) { s->set_brightness(s, v); });
    num("contrast", [](sensor_t *s, int v) { s->set_contrast(s, v); });
    num("saturation", [](sensor_t *s, int v) { s->set_saturation(s, v); });
    num("sharpness", [](sensor_t *s, int v) { s->set_sharpness(s, v); });
    num("denoise", [](sensor_t *s, int v) { s->set_denoise(s, v); });
    num("special_effect", [](sensor_t *s, int v) { s->set_special_effect(s, v); });
    num("wb_mode", [](sensor_t *s, int v) { s->set_wb_mode(s, v); });
    num("vflip", [](sensor_t *s, int v) { s->set_vflip(s, v); });
    num("hmirror", [](sensor_t *s, int v) { s->set_hmirror(s, v); });
    cJSON_Delete(j);
    return settingsGet(req);
}

inline void registerRoutes(WebServer &server) {
    startStreamWorker();
    server.on("/api/camera/stream", HTTP_GET, stream);
    server.on("/api/camera/settings", HTTP_GET, settingsGet);
    server.on("/api/camera/settings", HTTP_POST, settingsPost);
}

} // namespace camera_service

#endif // USE_CAMERA
