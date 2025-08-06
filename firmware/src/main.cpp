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

// Variables
#define APP_NAME "Hexapod"
#define APP_VERSION "v0.0.1"

// Communication
PsychicHttpServer server;
Websocket socket;
BLE ble;

// Service
WiFiService wifiService;
APService apService;
EventStorage eventStorage;

DRAM_ATTR Hexapod robot;

void setupServer() {
    server.config.max_uri_handlers = 10;
    server.maxUploadSize = 1000000; // 1 MB;
    server.listen(80);
    socket.attach(server, "/api/ws/events");
    server.serveStatic("/api/config/", ESPFS, "/config/");
    DefaultHeaders::Instance().addHeader("Server", APP_NAME);
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "Accept, Content-Type, Authorization");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Credentials", "true");
}

void IRAM_ATTR controlLoopEntry(void *) {
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

void IRAM_ATTR serviceLoopEntry(void *) {
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
    ESPFS.begin(true);
    ESP_LOGI("main", "Booting robot");

    feature_service::printFeatureConfiguration();

    xTaskCreate(serviceLoopEntry, "Service task", 4096, nullptr, 2, nullptr);

    xTaskCreatePinnedToCore(controlLoopEntry, "Control task", 4096, nullptr, 5, nullptr, 1);

    ESP_LOGI("main", "Setup finished");
}

void loop() { vTaskDelete(nullptr); }
