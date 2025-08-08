#include <Arduino.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <PsychicHttp.h>

#include <hexapod.h>
#include <wifi_service.h>
#include <ap_service.h>
#include <mdns_service.h>
#include <filesystem.h>
#include <features.h>
#include <communication/ble_adapter.h>
#include <communication/websocket_adapter.h>
#include <event_storage.h>

#include <WWWData.h>

// Variables
#define APP_NAME "Hexapod"
#define APP_VERSION "v0.0.1"

// Communication
BLE ble;
PsychicHttpServer server;
Websocket socket {server, "/api/ws"};
// TODO: Add a rest and sse option
// Rest rest {server, "/api/{topic_id}"}
// ServerSendEvent sse {server, "/api/sse"}
// TODO: EspNowAdapter now;
// TODO: Bluepad bluepad;

// Service
WiFiService wifiService;
APService apService;
EventStorage eventStorage;

DRAM_ATTR Hexapod robot;

void setupServer() {
    server.config.max_uri_handlers = 10;
    server.maxUploadSize = 1000000; // 1 MB;
    server.listen(80);
    server.serveStatic("/api/config/", ESP_FS, "/config/");
    WWWData::registerRoutes([](const char* uri, const char* contentType, const uint8_t* content, size_t len) {
        PsychicHttpRequestCallback requestHandler = [contentType, content, len](PsychicRequest* request) {
            PsychicResponse response(request);
            response.setCode(200);
            response.setContentType(contentType);
            response.addHeader("Content-Encoding", "gzip");
            response.addHeader("Vary", "Accept-Encoding");
            response.addHeader("Cache-Control", "public, immutable, max-age=31536000");
            response.setContent(content, len);
            return response.send();
        };
        auto* handler = new PsychicWebHandler();
        handler->onRequest(requestHandler);
        server.on(uri, HTTP_GET, handler);
        if (strcmp(uri, "/index.html") == 0) server.defaultEndpoint->setHandler(handler);
    });
    DefaultHeaders::Instance().addHeader("Server", APP_NAME);
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
}

void IRAM_ATTR controlLoopEntry(void*) {
    ESP_LOGI("main", "Control task starting");
    robot.initialize();
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = 5 / portTICK_PERIOD_MS;
    ESP_LOGI("main", "Control task started");
    for (;;) {
        robot.readSensors();
        robot.planMotion();
        robot.updateActuators();
        // robot.emitTelemetry();

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void IRAM_ATTR serviceLoopEntry(void*) {
    ESP_LOGI("main", "Service control task starting");
    eventStorage.begin();

    wifiService.begin();
    MDNS.begin(APP_NAME);
    MDNS.setInstanceName(APP_NAME);
    apService.begin();

    setupServer();

    ble.begin();
    socket.begin();

    ESP_LOGI("main", "Service control task started");
    for (;;) {
        wifiService.loop();
        apService.loop();

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void setup() {
    Serial.begin(115200);
    ESP_FS.begin(true); // TODO: Log failure
    ESP_LOGI("main", "Booting robot");

    feature_service::printFeatureConfiguration();

    xTaskCreate(serviceLoopEntry, "Service task", 4096, nullptr, 2, nullptr);

    xTaskCreatePinnedToCore(controlLoopEntry, "Control task", 4096, nullptr, 5, nullptr, 1);

    ESP_LOGI("main", "Setup finished");
}

void loop() { vTaskDelete(nullptr); }
