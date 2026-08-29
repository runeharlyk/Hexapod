#pragma once

#include <esp_http_server.h>
#include <esp_https_ota.h>
#include <esp_crt_bundle.h>
#include <esp_ota_ops.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_http_client.h>
#include <cJSON.h>
#include <cstring>
#include <memory>
#include <atomic>
#include <string>

#include <event_bus.h>
#include <message_types.h>
#include <communication/webserver.h>
#include <platform_shared/message.pb.h>

/*
 * Firmware update over both paths the app offers:
 *   POST /api/firmware           raw .bin body, streamed straight into the inactive OTA slot
 *   POST /api/firmware/download  {"download_url": "..."} — the robot fetches it over HTTPS itself
 *
 * Progress is published as socket_message_OtaStatusData on the EventBus, which main.cpp forwards
 * to every connected client, so the app's update dialog tracks a download it did not stream.
 *
 * The upload path takes the raw body rather than a multipart form: parsing multipart boundaries in
 * the request handler would buy nothing, and the app is the only client.
 */
namespace ota_service {

inline std::atomic<bool> inProgress{false};

inline void publish(socket_message_OtaState state, uint32_t progress, const char *error = nullptr) {
    socket_message_OtaStatusData msg = socket_message_OtaStatusData_init_zero;
    msg.state = state;
    msg.progress = progress;
    if (error) strncpy(msg.error, error, sizeof(msg.error) - 1);
    EventBus<socket_message_OtaStatusData>::publish(msg);
}

// Servos hold whatever pose they are in; walking while flashing risks a brownout mid-write.
inline void stopMotion() { EventBus<ModeMsg>::publish({MOTION_STATE::DEACTIVATED}); }

// Reboot from a detached task so the HTTP response reaches the client first.
inline void rebootSoon() {
    xTaskCreate(
        [](void *) {
            vTaskDelay(pdMS_TO_TICKS(1500));
            esp_restart();
        },
        "ota_reboot", 2048, nullptr, 5, nullptr);
}

inline esp_err_t upload(httpd_req_t *req) {
    static const char *TAG = "ota";

    if (inProgress.exchange(true)) return WebServer::sendError(req, 409, "update already running");

    const int total = req->content_len;
    if (total <= 0) {
        inProgress = false;
        return WebServer::sendError(req, 400, "empty body");
    }

    const esp_partition_t *target = esp_ota_get_next_update_partition(nullptr);
    if (!target) {
        inProgress = false;
        return WebServer::sendError(req, 500, "no OTA partition");
    }
    if ((int)target->size < total) {
        inProgress = false;
        return WebServer::sendError(req, 400, "image larger than the OTA partition");
    }

    esp_ota_handle_t handle = 0;
    esp_err_t err = esp_ota_begin(target, total, &handle);
    if (err != ESP_OK) {
        inProgress = false;
        publish(socket_message_OtaState_OTA_ERROR, 0, esp_err_to_name(err));
        return WebServer::sendError(req, 500, "esp_ota_begin failed");
    }

    ESP_LOGI(TAG, "upload -> %s, %d bytes", target->label, total);
    stopMotion();
    publish(socket_message_OtaState_OTA_PROGRESS, 0);

    char buf[1460]; // one TCP segment; larger buffers do not speed up a socket-bound write
    int received = 0;
    uint32_t lastPercent = 0;

    while (received < total) {
        int chunk = httpd_req_recv(req, buf, sizeof(buf) < (size_t)(total - received) ? sizeof(buf) : total - received);
        if (chunk == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (chunk <= 0) {
            esp_ota_abort(handle);
            inProgress = false;
            publish(socket_message_OtaState_OTA_ERROR, 0, "upload aborted");
            return WebServer::sendError(req, 400, "recv failed");
        }
        err = esp_ota_write(handle, buf, chunk);
        if (err != ESP_OK) {
            esp_ota_abort(handle);
            inProgress = false;
            publish(socket_message_OtaState_OTA_ERROR, 0, esp_err_to_name(err));
            return WebServer::sendError(req, 500, "flash write failed");
        }
        received += chunk;

        const uint32_t percent = (uint32_t)((int64_t)received * 100 / total);
        if (percent != lastPercent) {
            lastPercent = percent;
            publish(socket_message_OtaState_OTA_PROGRESS, percent);
        }
    }

    err = esp_ota_end(handle);
    if (err != ESP_OK) {
        inProgress = false;
        // ESP_ERR_OTA_VALIDATE_FAILED means the bytes arrived but are not a valid image for this chip.
        publish(socket_message_OtaState_OTA_ERROR, 0, esp_err_to_name(err));
        return WebServer::sendError(req, 400, "image validation failed");
    }

    err = esp_ota_set_boot_partition(target);
    if (err != ESP_OK) {
        inProgress = false;
        publish(socket_message_OtaState_OTA_ERROR, 0, esp_err_to_name(err));
        return WebServer::sendError(req, 500, "could not set boot partition");
    }

    ESP_LOGI(TAG, "upload complete, rebooting into %s", target->label);
    publish(socket_message_OtaState_OTA_FINISHED, 100);
    esp_err_t sent = WebServer::sendOk(req);
    rebootSoon();
    return sent;
}

struct DownloadTaskArgs {
    std::string url;
};

inline void downloadTask(void *arg) {
    static const char *TAG = "ota";
    std::unique_ptr<DownloadTaskArgs> args(static_cast<DownloadTaskArgs *>(arg));

    esp_http_client_config_t http = {};
    http.url = args->url.c_str();
    http.crt_bundle_attach = esp_crt_bundle_attach; // GitHub release assets redirect to a second host
    http.keep_alive_enable = true;
    http.timeout_ms = 15000;

    esp_https_ota_config_t cfg = {};
    cfg.http_config = &http;

    esp_https_ota_handle_t handle = nullptr;
    esp_err_t err = esp_https_ota_begin(&cfg, &handle);
    if (err != ESP_OK || handle == nullptr) {
        ESP_LOGE(TAG, "esp_https_ota_begin failed: %s", esp_err_to_name(err));
        publish(socket_message_OtaState_OTA_ERROR, 0, esp_err_to_name(err));
        inProgress = false;
        vTaskDelete(nullptr);
        return;
    }

    const int imageSize = esp_https_ota_get_image_size(handle);
    publish(socket_message_OtaState_OTA_PROGRESS, 0);

    uint32_t lastPercent = 0;
    while ((err = esp_https_ota_perform(handle)) == ESP_ERR_HTTPS_OTA_IN_PROGRESS) {
        if (imageSize > 0) {
            const uint32_t percent = (uint32_t)((int64_t)esp_https_ota_get_image_len_read(handle) * 100 / imageSize);
            if (percent != lastPercent) {
                lastPercent = percent;
                publish(socket_message_OtaState_OTA_PROGRESS, percent);
            }
        }
    }

    if (err != ESP_OK || !esp_https_ota_is_complete_data_received(handle)) {
        ESP_LOGE(TAG, "download failed: %s", esp_err_to_name(err));
        esp_https_ota_abort(handle);
        publish(socket_message_OtaState_OTA_ERROR, 0, err == ESP_OK ? "truncated download" : esp_err_to_name(err));
        inProgress = false;
        vTaskDelete(nullptr);
        return;
    }

    err = esp_https_ota_finish(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_https_ota_finish failed: %s", esp_err_to_name(err));
        publish(socket_message_OtaState_OTA_ERROR, 0, esp_err_to_name(err));
        inProgress = false;
        vTaskDelete(nullptr);
        return;
    }

    ESP_LOGI(TAG, "download complete, rebooting");
    publish(socket_message_OtaState_OTA_FINISHED, 100);
    rebootSoon();
    inProgress = false;
    vTaskDelete(nullptr);
}

inline esp_err_t download(httpd_req_t *req) {
    const int len = req->content_len;
    if (len <= 0 || len > 1024) return WebServer::sendError(req, 400, "bad body");

    std::string raw(len, '\0');
    int got = 0;
    while (got < len) {
        int r = httpd_req_recv(req, &raw[got], len - got);
        if (r == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (r <= 0) return WebServer::sendError(req, 400, "recv failed");
        got += r;
    }

    cJSON *body = cJSON_Parse(raw.c_str());
    cJSON *url = body ? cJSON_GetObjectItem(body, "download_url") : nullptr;
    if (!cJSON_IsString(url) || strncmp(url->valuestring, "https://", 8) != 0) {
        if (body) cJSON_Delete(body);
        // Plain http is refused: an unauthenticated image over a hostile network is a flash-write primitive.
        return WebServer::sendError(req, 400, "download_url must be an https URL");
    }

    if (inProgress.exchange(true)) {
        cJSON_Delete(body);
        return WebServer::sendError(req, 409, "update already running");
    }

    stopMotion();
    auto *args = new DownloadTaskArgs{url->valuestring};
    cJSON_Delete(body);

    // TLS handshake plus the OTA buffers need a deep stack; 8 KB matches IDF's own OTA examples.
    if (xTaskCreate(downloadTask, "ota_download", 8192, args, 4, nullptr) != pdPASS) {
        delete args;
        inProgress = false;
        return WebServer::sendError(req, 500, "could not start update task");
    }

    return WebServer::sendOk(req);
}

// Mark the running image valid on first boot, so a bricked update rolls back instead of looping.
inline void confirmRunningImage() {
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t state;
    if (esp_ota_get_state_partition(running, &state) != ESP_OK) return;
    if (state == ESP_OTA_IMG_PENDING_VERIFY) {
        esp_ota_mark_app_valid_cancel_rollback();
        ESP_LOGI("ota", "running image marked valid (%s)", running->label);
    }
}

inline void registerRoutes(WebServer &server) {
    server.on("/api/firmware", HTTP_POST, upload);
    server.on("/api/firmware/download", HTTP_POST, download);
}

} // namespace ota_service
