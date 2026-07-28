#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_chip_info.h>
#include <esp_heap_caps.h>
#include <esp_idf_version.h>
#include <esp_flash.h>
#include <esp_sleep.h>
#include <esp_littlefs.h>
#include <nvs_flash.h>
#include <cmath>
#include <cstring>

#include <features.h>
#include <filesystem.h>
#include <wifi/wifi_idf.h>
#include <wifi_service.h>
#include <ap_service.h>
#include <mdns_service.h>
#include <communication/webserver.h>
#include <communication/websocket.h>
#include <communication/ble.h>
#include <www_mount.hpp>
#include <hexapod.h>
#include <cpu_stats.h>
#include <filesystem_service.h>
#include <camera_service.h>
#include <servo_settings_service.h>
#include <event_bus.h>
#include <message_types.h>
#include <platform_shared/message.pb.h>

static const char *TAG = "main";

WiFiService wifiService;
APService apService;
MDNSService mdnsService;
ServoSettingsService servoSettingsService;
Websocket wsSocket{server, "/api/ws"};
BLE bleAdapter;
Hexapod robot;
CpuMonitor cpuMonitor;

static void fillAnalytics(socket_message_AnalyticsData &a) {
    a.free_heap = esp_get_free_heap_size();
    a.min_free_heap = esp_get_minimum_free_heap_size();
    a.total_heap = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
    a.max_alloc_heap = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    a.psram_size = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    a.free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    a.uptime = esp_timer_get_time() / 1000000;
    size_t total = 0, used = 0;
    if (esp_littlefs_info(FS_PARTITION_LABEL, &total, &used) == ESP_OK) {
        a.fs_total = (int)total;
        a.fs_used = (int)used;
    }
    CpuMonitor::Usage cpu = cpuMonitor.sample();
    a.cpu0_usage = cpu.core0;
    a.cpu1_usage = cpu.core1;
    a.cpu_usage = cpu.total;
}

static void fillStaticInfo(socket_message_StaticSystemInformation &s) {
    strncpy(s.esp_platform, CONFIG_IDF_TARGET, sizeof(s.esp_platform) - 1);
    strncpy(s.cpu_type, CONFIG_IDF_TARGET, sizeof(s.cpu_type) - 1);
    strncpy(s.firmware_version, APP_VERSION, sizeof(s.firmware_version) - 1);
    strncpy(s.sdk_version, esp_get_idf_version(), sizeof(s.sdk_version) - 1);
    s.cpu_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    esp_chip_info_t info;
    esp_chip_info(&info);
    s.cpu_cores = info.cores;
    uint32_t flash_size = 0;
    esp_flash_get_size(nullptr, &flash_size);
    s.flash_chip_size = flash_size;
}

static void registerHandlers(CommAdapterBase &c) {
    c.on<socket_message_ControllerInputData>([](const socket_message_ControllerInputData &in, int) {
        CommandMsg cmd{in.left.x, in.left.y, in.right.x, in.right.y, in.height, in.speed, in.s1, in.feet_distance};
        EventBus<CommandMsg>::publish(cmd);
    });
    c.on<socket_message_ModeData>(
        [](const socket_message_ModeData &m, int) { EventBus<ModeMsg>::publish({static_cast<MOTION_STATE>(m.mode)}); });
    c.on<socket_message_GaitData>(
        [](const socket_message_GaitData &g, int) { EventBus<GaitMsg>::publish({static_cast<GaitType>(g.gait)}); });
    c.on<socket_message_ServoPWMData>([](const socket_message_ServoPWMData &s, int) {
        EventBus<ServoSignalMsg>::publish({static_cast<int8_t>(s.servo_id), static_cast<uint16_t>(s.servo_pwm)});
    });
    c.on<socket_message_ServoStateData>(
        [](const socket_message_ServoStateData &s, int) { EventBus<ServoStateMsg>::publish({s.active}); });

    c.on<socket_message_SystemCommandData>([](const socket_message_SystemCommandData &cmd, int) {
        switch (cmd.command) {
            case socket_message_SystemCommand_SYS_RESTART: esp_restart(); break;
            case socket_message_SystemCommand_SYS_RESET:
                nvs_flash_erase();
                esp_restart();
                break;
            case socket_message_SystemCommand_SYS_SLEEP: esp_deep_sleep_start(); break;
            default: break;
        }
    });

    c.on<socket_message_CorrelationRequest>([adapter = &c](const socket_message_CorrelationRequest &req, int clientId) {
        socket_message_CorrelationResponse res = socket_message_CorrelationResponse_init_zero;
        res.correlation_id = req.correlation_id;
        res.status_code = 200;
        switch (req.which_request) {
            case socket_message_CorrelationRequest_features_data_request_tag: {
                res.which_response = socket_message_CorrelationResponse_features_data_response_tag;
                auto &f = res.response.features_data_response;
                strncpy(f.firmware_name, APP_NAME, sizeof(f.firmware_name) - 1);
                strncpy(f.firmware_version, APP_VERSION, sizeof(f.firmware_version) - 1);
                f.camera = USE_CAMERA;
                f.imu = USE_MPU6050;
                f.mag = USE_MAG;
                f.servo = true;
                f.mdns = USE_MDNS;
                break;
            }
            case socket_message_CorrelationRequest_system_information_request_tag: {
                res.which_response = socket_message_CorrelationResponse_system_information_response_tag;
                auto &sys = res.response.system_information_response;
                sys.has_analytics_data = true;
                sys.has_static_system_information = true;
                fillAnalytics(sys.analytics_data);
                fillStaticInfo(sys.static_system_information);
                break;
            }
            case socket_message_CorrelationRequest_i2c_scan_data_request_tag: {
                res.which_response = socket_message_CorrelationResponse_i2c_scan_data_tag;
                auto &scan = res.response.i2c_scan_data;
                scan.devices_count = 0;
                for (uint8_t addr : robot.scanI2C()) {
                    if (scan.devices_count >= 16) break;
                    scan.devices[scan.devices_count++].address = addr;
                }
                break;
            }
            case socket_message_CorrelationRequest_wifi_settings_get_tag: {
                res.which_response = socket_message_CorrelationResponse_wifi_settings_tag;
                res.response.wifi_settings = wifiService.state();
                break;
            }
            case socket_message_CorrelationRequest_wifi_settings_update_tag: {
                wifiService.update(
                    [&req](WiFiSettings &s) { return WiFiSettings_update(req.request.wifi_settings_update, s); },
                    "correlation");
                res.which_response = socket_message_CorrelationResponse_wifi_settings_tag;
                res.response.wifi_settings = wifiService.state();
                break;
            }
            case socket_message_CorrelationRequest_wifi_networks_get_tag: {
                // Static so the pointer survives adapter->emit; the evtbus worker is single-threaded.
                static api_WifiNetworkScan scanBuf[20];
                int count = WiFiService::networksProto(scanBuf, 20);
                res.which_response = socket_message_CorrelationResponse_wifi_network_list_tag;
                res.response.wifi_network_list.networks = scanBuf;
                res.response.wifi_network_list.networks_count = count < 0 ? 0 : count;
                if (count == -1) res.status_code = 202;       // scan running; client polls again
                else if (count == -2) res.status_code = 503;  // radio busy (STA connecting); give up
                break;
            }
            case socket_message_CorrelationRequest_wifi_status_get_tag: {
                res.which_response = socket_message_CorrelationResponse_wifi_status_tag;
                WiFiService::statusProto(res.response.wifi_status);
                break;
            }
            case socket_message_CorrelationRequest_ap_settings_get_tag: {
                res.which_response = socket_message_CorrelationResponse_ap_settings_tag;
                res.response.ap_settings = apService.state();
                break;
            }
            case socket_message_CorrelationRequest_ap_settings_update_tag: {
                apService.update(
                    [&req](APSettings &s) { return APSettings_update(req.request.ap_settings_update, s); },
                    "correlation");
                res.which_response = socket_message_CorrelationResponse_ap_settings_tag;
                res.response.ap_settings = apService.state();
                break;
            }
            case socket_message_CorrelationRequest_ap_status_get_tag: {
                res.which_response = socket_message_CorrelationResponse_ap_status_tag;
                apService.statusProto(res.response.ap_status);
                break;
            }
            case socket_message_CorrelationRequest_mdns_status_get_tag: {
                res.which_response = socket_message_CorrelationResponse_mdns_status_tag;
                mdnsService.statusProto(res.response.mdns_status);
                break;
            }
            case socket_message_CorrelationRequest_mdns_query_tag: {
                res.which_response = socket_message_CorrelationResponse_mdns_query_response_tag;
                mdnsService.queryProto(req.request.mdns_query, res.response.mdns_query_response);
                break;
            }
            case socket_message_CorrelationRequest_servo_settings_get_tag: {
                res.which_response = socket_message_CorrelationResponse_servo_settings_tag;
                res.response.servo_settings = servoSettingsService.state();
                break;
            }
            case socket_message_CorrelationRequest_servo_settings_update_tag: {
                servoSettingsService.update(
                    [&req](ServoSettings &s) { return ServoSettings_update(req.request.servo_settings_update, s); },
                    "correlation");
                res.which_response = socket_message_CorrelationResponse_servo_settings_tag;
                res.response.servo_settings = servoSettingsService.state();
                break;
            }
            default: res.status_code = 400; break;
        }
        adapter->emit(res, clientId);
    });
}

template <typename T>
static void emitAll(const T &msg) {
    wsSocket.emit(msg);
    bleAdapter.emit(msg);
}

template <typename ProtoT>
static void observeStatus() {
    EventBus<ProtoT>::subscribe([](const ProtoT &s) { emitAll(s); });
}

static void initNvs() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

static void setupServer() {
    server.config(50, 16384);
    server.listen(80);

    // Answer the browser's CORS preflight so the cross-origin dev app can reach the robot.
    server.addDefaultHeader("Access-Control-Allow-Origin", "*");
    server.addDefaultHeader("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS");
    server.addDefaultHeader("Access-Control-Allow-Headers", "Content-Type, Authorization");
    server.on("/*", HTTP_OPTIONS, [](httpd_req_t *r) {
        httpd_resp_set_status(r, "200 OK");
        return httpd_resp_send(r, nullptr, 0);
    });

    server.on("/api/features", HTTP_GET, [](httpd_req_t *r) {
        char buf[384];
        snprintf(buf, sizeof(buf),
                 "{\"camera\":%s,\"imu\":%s,\"mag\":%s,\"bmp\":false,\"mdns\":%s,\"servo\":true,"
                 "\"analytics\":true,\"sleep\":true,\"ota\":false,\"download_firmware\":false,"
                 "\"upload_firmware\":false,"
                 "\"firmware_version\":\"%s\",\"firmware_name\":\"%s\",\"firmware_built_target\":\"esp32-wroom-camera\"}",
                 USE_CAMERA ? "true" : "false", USE_MPU6050 ? "true" : "false", USE_MAG ? "true" : "false",
                 USE_MDNS ? "true" : "false", APP_VERSION, APP_NAME);
        httpd_resp_set_type(r, "application/json");
        return httpd_resp_sendstr(r, buf);
    });

    fs_api::registerRoutes(server);
#if USE_CAMERA
    camera_service::registerRoutes(server);
#endif

#if EMBED_WEBAPP
    mountStaticAssets(server);
    mountSpaFallback(server);
#endif
}

static void setupComm() {
    registerHandlers(wsSocket);
    registerHandlers(bleAdapter);

    EventBus<ServoAnglesMsg>::subscribe([](const ServoAnglesMsg &a) {
        socket_message_AnglesData out = socket_message_AnglesData_init_zero;
        out.angles_count = NUM_SERVO;
        for (int i = 0; i < NUM_SERVO; i++) out.angles[i] = static_cast<int32_t>(lroundf(a.angles[i]));
        emitAll(out);
    });

    EventBus<IMUAnglesMsg>::subscribe([](const IMUAnglesMsg &m) {
        socket_message_IMUData out = socket_message_IMUData_init_zero;
        out.x = m.rpy[0];
        out.y = m.rpy[1];
        out.z = m.rpy[2];
        emitAll(out);
    });

    observeStatus<api_WifiStatus>();
    observeStatus<api_APStatus>();

    wsSocket.begin();

    // BLE HCI needs contiguous internal DMA RAM; log what's left so an init failure is diagnosable.
    ESP_LOGI(TAG, "Before BLE: internal=%u DMA=%u largestDMA=%u", (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
    bleAdapter.begin();
}

static void controlLoop(void *) {
    robot.initialize();
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        robot.readSensors();
        robot.planMotion();
        robot.updateActuators();
        robot.emitTelemetry();
        vTaskDelayUntil(&last, pdMS_TO_TICKS(5));
    }
}

static void serviceLoop(void *) {
    WiFi.init();
    wifiService.begin();
    mdnsService.begin();
    apService.begin();
    setupServer();
    setupComm();

    // After BLE so the controller claims its internal DMA first; camera init is non-fatal.
#if USE_CAMERA
    camera_service::init();
#endif

    ESP_LOGI(TAG, "Networking up, free heap %lu bytes", (unsigned long)esp_get_free_heap_size());

    for (;;) {
        wifiService.loop();
        apService.loop();
        EXECUTE_EVERY_N_MS(2000, {
            socket_message_AnalyticsData a = socket_message_AnalyticsData_init_zero;
            fillAnalytics(a);
            emitAll(a);
        });
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

extern "C" void app_main() {
    ESP_LOGI(TAG, "Booting %s (ESP-IDF)", APP_NAME);
    esp_log_level_set("NimBLE", ESP_LOG_WARN);  // NimBLE logs every notify at INFO — a flood
    initNvs();

    if (!FileSystem::init()) {
        ESP_LOGE(TAG, "Filesystem mount failed");
    }
    feature_service::printFeatureConfiguration();
    servoSettingsService.begin();

    xTaskCreate(serviceLoop, "service", 8192, nullptr, 2, nullptr);
    xTaskCreatePinnedToCore(controlLoop, "control", 8192, nullptr, 5, nullptr, 1);
}
